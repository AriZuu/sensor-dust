Wireless dust sensor node
=========================

This is a wireless dust sensor node using
olimex MSP430-CCRF board. It works with
DSM501A or PPD42NS dust sensor, which outputs a PWM signal
based on detected particle count.

Sensor output must be connected to P2.2 pin. Application configures
the pin to trigger timer1 captures to get accurate measurements.

