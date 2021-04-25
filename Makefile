#
# Copyright (c) 2015, Ari Suutari <ari@stonepile.fi>.
# All rights reserved. 
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
# 
#  1. Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#  2. Redistributions in binary form must reproduce the above copyright
#     notice, this list of conditions and the following disclaimer in the
#     documentation and/or other materials provided with the distribution.
#  3. The name of the author may not be used to endorse or promote
#     products derived from this software without specific prior written
#     permission. 
# 
# THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS
# OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT,
# INDIRECT,  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
# STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
# OF THE POSSIBILITY OF SUCH DAMAGE.

RELROOT = ../picoos/

PORT = msp430
#
# This is for Olimex MSP430-CCRF card
#
MCU ?= cc430f5137 
COMPILER=GCC

BUILD ?= RELEASE
RADIO=MRFI_CC430
TEXAS_CONF=../sensor-ap/texas_ccrf

export MCU

SIMPLICITI_DEFINES = NUM_CONNECTIONS=1 \
	SIZE_INFRAME_Q=1 \
	SIZE_OUTFRAME_Q=1 \
	THIS_DEVICE_ADDRESS="{{0x79,0x56,0x34,0x12}}" \
	END_DEVICE \
	MAX_HOPS=2 \
	MAX_HOPS_FROM_AP=1 \
	MAX_NWK_PAYLOAD=9 \
	MAX_APP_PAYLOAD=18 \
	DEFAULT_LINK_TOKEN=0x01020304 \
	DEFAULT_JOIN_TOKEN=0x05060708 \
	xEXTENDED_API APP_AUTO_ACK \
	ISM_EU LINK_SPEED=1 \
	$(RADIO) 

export SIMPLICITI_DEFINES

include $(RELROOT)make/common.mak

NANO = 1
TARGET = sensor-dust
TEXAS_SRC=../simpliciti
export TEXAS_CONF
#
SRC_TXT =	sensor_dust.c
SRC_HDR = 
SRC_OBJ =
SRC_LIB =
CDEFINES += $(SIMPLICITI_DEFINES)

DIR_USRINC += $(TEXAS_CONF) \
		$(TEXAS_SRC)/bsp/ \
		$(TEXAS_SRC)/mrfi/ \
		$(TEXAS_SRC)/simpliciti/nwk/ \
		$(TEXAS_SRC)/simpliciti/nwk_applications/ \
		../picoos-micro ../picoos-micro/ports/msp430/hal5xx6xx ../sensor-ap

DIR_CONFIG = $(CURRENTDIR)
DIR_OUTPUT = $(CURRENTDIR)/bin
MODULES += ../simpliciti ../picoos-micro

POSTLINK1 = msp430-size $(TARGETOUT)

# ---------------------------------------------------------------------------
# BUILD THE EXECUTABLE
# ---------------------------------------------------------------------------

include $(MAKE_OUT)

