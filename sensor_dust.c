/*
 * Copyright (c) 2006-2015, Ari Suutari <ari@stonepile.fi>.
 * All rights reserved. 
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *  3. The name of the author may not be used to endorse or promote
 *     products derived from this software without specific prior written
 *     permission. 
 * 
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT,
 * INDIRECT,  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <picoos.h>
#include <picoos-u.h>
#include <stdbool.h>

#include <string.h>

#include "bsp.h"
#include "mrfi.h"
#include "nwk_types.h"
#include "nwk_api.h"
#include "nwk_frame.h"
#include "nwk.h"
#include "mrfi_defs.h"

#include "sensor_msg.h"
#include "calib_data.h"

#define MAIN_STACK_SIZE       260
#define IDLE_STACK_SIZE       60
#define MEAS_TIMER_SECS      (60 * 5)
#define LED_RED               BIT0
#define LED_DIR               P1DIR
#define LED_OUT               P1OUT
#define MISSES_IN_A_ROW       3
#define PULSE_AVG_SIZE        5

#include <HAL_TLV.h>

/*
 * Radio calibration data is in info D segment,
 */
#if __GNUC__ == 4
extern int __infod[];
#else
int* __infod = 0x1800;
#endif

unsigned char rf_freqoffset;

static POSSEMA_t adcSem;
static POSSEMA_t timerSem;
static POSTIMER_t mainTimer;
static POSMUTEX_t pulseMutex;

void adcInterrupt(void);
void counterIrqHandler(void);
static bool joinAp(void);
static void readSensors(void);
static int sendPacket(void);
static void mainTask(void *arg);

struct s_TLV_ADC_Cal_Data * pADCCAL;
struct s_TLV_Die_Record* die;

typedef struct {

  uint32_t downTime;
  uint32_t upTime;
} DustPulses;


static unsigned char bADCCAL_bytes;
static float mref;
static float nref;
static linkID_t sLinkID1 = 0;
static volatile unsigned int adcResult;
static volatile DustPulses current;
static DustPulses history[PULSE_AVG_SIZE];
static volatile uint16_t prevCounter = 0;
static SensorMsg msg;
static SensorMsgData* msgData;

#define MSG_HAS_ROOM (msgData - msg.data < MSG_MAX_SENSOR_VALUES)

/*
 * Accumulate elapsed timer cycles when
 * signal changes to expected direction.
 */
static void counterAccum(uint16_t now)
{
  int16_t elapsed;

  elapsed = now - prevCounter;
  if ((TA1CCTL1 & (CM0 | CM1)) == CM_1) // Rising edge -> pulse ended
    current.downTime += elapsed;
  else
    current.upTime += elapsed;

  prevCounter = now;
}

/*
 * Interrupt from TimerA1 capture.
 */
void PORT_NAKED __attribute__((interrupt(TIMER1_A1_VECTOR))) counterIrqHandler()
{
  portSaveContext();
  c_pos_intEnter();

  switch (TA1IV) {
  case 2:

    if (TA1CCTL1 & COV)
      TA1CCTL1 &= ~COV;
    else {
      counterAccum(TA1CCR1);

      if ((TA1CCTL1 & (CM0 | CM1)) == CM_1) // Rising edge -> pulse ended
        LED_OUT &= ~LED_RED;
      else
        LED_OUT |= LED_RED;

      TA1CCTL1 ^= (CM0 | CM1);
    }
    break;
  }

  c_pos_intExit();
  portRestoreContext();
}

/*
 * Maintain history about captured pulse widths
 * so we can calculate moving avg when data is sent.
 */
static void pulseTask(void* arg)
{
  P2SEL |= BIT2;

  TA1CTL = 0;                         // Stop timer.
  prevCounter = 0;
  TA1CTL = TASSEL__ACLK | ID__8;      // Use ACLK / 8.
  TA1CTL |= TACLR;                    // Clear everything.

  // capture falling edge, sync, CCIxA
  TA1CCTL1 = CM_2 | SCS | CCIS_0 | CAP | CCIE;
  TA1EX0 = TAIDEX_2;                  // Divide more by /3 -> ACLK / 24
  TA1CTL |= MC_2;                     // Continuous mode.

  while (1) {

    POS_LOCKFLAGS;

    // Sensor is sampled each 30 sec (1 second delay is later in loop).
    posTaskSleep(MS(30000));

    posMutexLock(pulseMutex);

    // Move stuff in avg buffer.
    memmove(history + 1, history, sizeof(DustPulses) * (PULSE_AVG_SIZE - 1));

    POS_SCHED_LOCK;

    // Remember to accumulate pulse that has not yet completed.
    counterAccum(TA1R);

    // Save currently accumulated values to avg buffer 
    // and clear them.
    history[0] = current;
    current.downTime = 0;
    current.upTime = 0;

    POS_SCHED_UNLOCK;

    posMutexUnlock(pulseMutex);
  }
}

/*
 * ADC interrupt handler.
 */
void PORT_NAKED __attribute__((interrupt(ADC12_VECTOR))) adcInterrupt()
{
  portSaveContext();
  c_pos_intEnter();

  if (ADC12IV == 6) {

    adcResult = ADC12MEM0;
    posSemaSignal(adcSem);

  }

  c_pos_intExit();
  portRestoreContext();
}

/*
 * Handle ADC conversion for given channel.
 */
static unsigned int adcConvert(unsigned int chan, unsigned int sht, unsigned int ref)
{
  REFCTL0 |= REFMSTR + ref + REFON; // Enable internal reference (1.5V or 2.5V)

  // Initialize ADC12_A
  ADC12CTL0  = sht + ADC12ON;       // Set sample time
  ADC12CTL1  = ADC12SHP;            // Enable sample timer
  ADC12MCTL0 = ADC12SREF_1 + chan;  // ADC input channel
  ADC12IE    = 0x001;               // Interrupt when conversion is done

  __delay_cycles(128 * PORTCFG_CPU_CLOCK_MHZ);

  ADC12CTL0 |= ADC12ENC;

  ADC12CTL0 |= ADC12SC;             // Start conversion

  // Wait for ADC to complete.
  posSemaGet(adcSem);

  // Disable ADC.
  ADC12CTL0 &= ~(ADC12ENC | ADC12SC | sht);
  ADC12CTL0 &= ~ADC12ON;

  // Shut down reference voltage
  REFCTL0 &= ~(REFMSTR + ref + REFON);

  ADC12IE = 0;

  return adcResult;
}

/*
 * Save result in measurement packet.
 */
static void saveData(uint8_t id, uint8_t type, int16_t value)
{
  if (MSG_HAS_ROOM) {

    msgData->id = id;
    msgData->type = type;
    msgData->value = value;
    msgData++;
    nosPrint(" OK\n");
  }
  else
    nosPrint(" Full\n");
}

/*
 * Read all sensors and put data to packet.
 */
static void readSensors()
{
  int degC, volt;
  int result;

  // Get temperature.
  result = adcConvert(ADC12INCH_10, ADC12SHT0_8, REFVSEL_0);
  degC = (result - nref) / mref * 10;

  nosPrintf("Temp %d.%d", degC / 10, degC % 10);
  saveData(0, SENSOR_TEMPERATURE, degC);

  // Get battery voltage.
  result = adcConvert(ADC12INCH_11, ADC12SHT0_10, REFVSEL_1);

  // Convert ADC value to voltage.
  volt = (result * 2 * 2) / 41;
  volt = volt / 10;

  nosPrintf("Batt %d.%d", volt / 10, volt % 10);
  saveData(0, SENSOR_BATTVOLTAGE, volt);

  int i;
  uint32_t down = 0;
  uint32_t total = 0;
  int ratio;

  posMutexLock(pulseMutex);

  // Calculate sums of dust sensor pulse history for avg.
  for (i = 0; i < PULSE_AVG_SIZE; i++) {

    down += history[i].downTime;
    total += history[i].downTime + history[i].upTime;
  }

  posMutexUnlock(pulseMutex);

  // Calculate pulse ratio. It correlates to detected particle amount.
  ratio = (int)(10000.0 * down / total);

  nosPrintf("Downratio %d.%d", ratio / 100, ratio % 100);
  saveData(0, SENSOR_PULSES, ratio);
}

/*
 * Simpliciti network join.
 */
static bool joinAp()
{
  uint8_t misses;

  // Keep trying to join (a side effect of successful initialization) until
  // successful. 
  misses = 0;
  while (SMPL_SUCCESS != SMPL_Init(0) && misses < 3) {

    posTaskSleep(MS(3000));
    ++misses;
  }

  if (misses < 3) {

    nosPrint("Joined.\n");

    // After join, establish link.
    misses = 0;
    while (SMPL_SUCCESS != SMPL_Link(&sLinkID1) && misses < 10) {

      posTaskSleep(MS(3000));
      ++misses;
    }

    if (misses < 10) {

      nosPrint("Linked.\n");
      return 1;
    }
  }

  return 0;
}

/*
 * Send measurement packet.
 */
static int sendPacket()
{
  uint8_t misses;
  uint8_t noAck;
  smplStatus_t rc;

  noAck = 0;

  // Try sending message MISSES_IN_A_ROW times looking for ack.
  for (misses = 0; misses < MISSES_IN_A_ROW; ++misses) {

    if (SMPL_SUCCESS == (rc = SMPL_SendOpt(sLinkID1, (uint8_t*) &msg, sizeof(msg), SMPL_TXOPTION_ACKREQ))) {

      return SMPL_SUCCESS;
    }

    if (SMPL_NO_ACK == rc) {

      // Count ack failures. Could also fail becuase of CCA and
      // we don't want to scan in this case.
      noAck++;
    }
  }

  if (MISSES_IN_A_ROW == noAck) {

    nosPrint("AP lost.\n");
    SMPL_Ioctl(IOCTL_OBJ_CONNOBJ, IOCTL_ACT_DELETE, &sLinkID1);
    return SMPL_NO_JOIN;
  }

  return SMPL_NO_ACK;

}

static void mainTask(void *memstart)
{
  bool joined;
  bool lastAcked;
  smplStatus_t rc;
  uint8_t pass;

  LED_DIR |= LED_RED;
  LED_OUT &= ~LED_RED;

  uint16_t actlevel;

  // Radio needs at least PMMCOREV_2.
  actlevel = (PMMCTL0 & PMMCOREV_3);
  if (actlevel < PMMCOREV_2)
    portSetVCore(PMMCOREV_2);

  // Activate watchdog.
  WDTCTL = WDTPW + WDTIS__128M + WDTSSEL__ACLK + WDTCNTCL;

  nosPrint("Sensor start.\n");
  uosBootDiag();

  // Get radio calibration data.
  if (__infod[0] == CALIB_DATA_FINGERPRINT)
    rf_freqoffset = __infod[1];

  memset(&msg, '\0', sizeof(msg));

  // Get temperature calibation data from TLV.
  Get_TLV_Info(TLV_ADCCAL, 0, &bADCCAL_bytes, (unsigned int **) &pADCCAL);
  if (pADCCAL->adc_ref15_30_temp == pADCCAL->adc_ref20_30_temp
      || pADCCAL->adc_ref15_30_temp == pADCCAL->adc_ref25_30_temp) { // Calibration data wrong, see device errata datasheet

    mref = ((float) (pADCCAL->adc_ref15_85_temp - 2198)) / (85 - 30);
    nosPrint("This chip has calibration data error.\n");
  }
  else {

    mref = ((float) (pADCCAL->adc_ref15_85_temp - pADCCAL->adc_ref15_30_temp)) / (85 - 30);
  }

  nref = pADCCAL->adc_ref15_85_temp - mref * 85;

  // Get die info from TLV and use parts from it as station address.
  Get_TLV_Info(TLV_DIERECORD, 0, &bADCCAL_bytes, (unsigned int**) &die);
  if (die != NULL) {

    addr_t myAddr;

    myAddr.addr[0] = die->die_y_position & 0xff;
    myAddr.addr[1] = die->die_x_position & 0xff;
    myAddr.addr[2] = die->wafer_id & 0xff;
    myAddr.addr[3] = die->wafer_id >> 8;

    SMPL_Ioctl(IOCTL_OBJ_ADDR, IOCTL_ACT_SET, &myAddr);
  }

  msg.pktVersion = 3;
  pulseMutex = posMutexCreate();
  adcSem = posSemaCreate(0);
  timerSem = posSemaCreate(0);
  mainTimer = posTimerCreate();

  P_ASSERT("mainSema", adcSem != NULL && timerSem != NULL && mainTimer != NULL);

  posTimerSet(mainTimer, timerSem, MS(10000), MS(MEAS_TIMER_SECS * 1000L));
  posTimerStart(mainTimer);

  joined = false;
  lastAcked = false;
  msg.seq = 0;

  // Try to join AP. Put radio to sleep after that.
  joined = joinAp();
  SMPL_Ioctl(IOCTL_OBJ_RADIO, IOCTL_ACT_RADIO_SLEEP, 0);

  // Task to maintain pulse averages.
  posTaskCreate(pulseTask, NULL, 11, 100);

  pass = 0;
  while (true) {

    lastAcked = false;

    msg.seq++;
    msgData = msg.data;

    readSensors();

    // Clear tail of the packet.
    while (MSG_HAS_ROOM) {

      memset(msgData, '\0', sizeof (SensorMsgData));
      ++msgData;
    }

    if (pass < 2)
      ++pass;

    // Turn radio on and check if we need to join again.
    SMPL_Ioctl(IOCTL_OBJ_RADIO, IOCTL_ACT_RADIO_AWAKE, 0);
    if (!joined) {

      joined = joinAp();
    }

    if (joined) {

      rc = sendPacket();
      if (rc == SMPL_SUCCESS)
        lastAcked = true;
      else if (rc == SMPL_NO_JOIN)
        joined = false;
    }

    // Put radio back to sleep.
    SMPL_Ioctl(IOCTL_OBJ_RADIO, IOCTL_ACT_RADIO_SLEEP, 0);

    // If we got ack from AP, feed watchdog.
    if (lastAcked)
      WDTCTL = WDTPW + WDTIS__128M + WDTSSEL__ACLK + WDTCNTCL;

    uosResourceDiag();
    posSemaGet(timerSem);
  }
}

/*
 * Initialize board pins.
 */

static void initPins(void)
{

  // To save power, set unused IO pins to output.

  // PULL-UP   P5.0 XIN
  // PULL-DOWN P5.1 XOUT

  // OUT       P0.1 LED
  // OUT       P1.1 button
  // PULL-DOWN P1.2-1.4 unused
  // PULL-DOWN P1.5-1.6 rs232
  // IN        P1.7 STE, on jo pull-up

  P1REN = BIT2 + BIT3 + BIT4;

  // IN        P2.0-2.1 SCL / SDA

  P2DIR = 0;
  P2SEL = 0;
  P2REN = 0xff & ~(BIT0 + BIT1 + BIT2); // P2.2 = dust sensor pulse

  P3REN = 0xff;

#ifndef _DBG
  // Pull down JTAG pins.
  PJDIR = 0;
  PJOUT = 0;
  PJREN = 0xFF;
#endif

  P5OUT = 0x00;
  P5SEL |= BIT1 + BIT0;

  UCSCTL6 &= (~XTS);        // Select Low-frequency mode.
  UCSCTL6 |= XCAP_3;        // Internal load cap.

/*
 * Program * TX & RX pin for usart use.
 */

  UCA0CTL1 |= UCSWRST;

  P1DIR |= BIT6;                            // Set P1.6 as TX output
  P1SEL |= BIT5 + BIT6;                     // Select P1.5 & P1.6 to UART function
}

int main(int argc, char **argv)
{
  initPins();
  uosInit();
  nosInit(mainTask, NULL, 10, MAIN_STACK_SIZE, IDLE_STACK_SIZE);
  return 0;
}
