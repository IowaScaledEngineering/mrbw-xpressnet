/*************************************************************************
Title:    MRBW-XPRESSNET MRBee XpressNet Interface
Authors:  Michael D. Petersen <railfan@drgw.net>
          Nathan D. Holmes <maverick@drgw.net>
File:     mrbw-xpressnet.c
License:  GNU General Public License v3

LICENSE:
    Copyright (C) 2017 Michael Petersen & Nathan Holmes
    
    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
*************************************************************************/

#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <util/delay.h>
#include <util/atomic.h>
#include "mrbee.h"
#include "xpressnet.h"

#define XPRESSNET_ADDRESS 5

#ifdef DEBUG
void debugInit(void)
{
	DDRD |= _BV(PD5) | _BV(PD6);
	PORTD &= ~(_BV(PD5) | _BV(PD6));
}

void debug5(uint8_t val)
{
	if(val)
		PORTD |= _BV(PD5);
	else
		PORTD &= ~_BV(PD5);
}

void debug6(uint8_t val)
{
	if(val)
		PORTD |= _BV(PD6);
	else
		PORTD &= ~_BV(PD6);
}
#endif

#define MRBUS_TX_BUFFER_DEPTH 16
#define MRBUS_RX_BUFFER_DEPTH 16

MRBusPacket mrbusTxPktBufferArray[MRBUS_TX_BUFFER_DEPTH];
MRBusPacket mrbusRxPktBufferArray[MRBUS_RX_BUFFER_DEPTH];

#define XPRESSNET_TX_BUFFER_DEPTH 16

XpressNetPacket xpressnetTxPktBufferArray[XPRESSNET_TX_BUFFER_DEPTH];


uint8_t mrbus_dev_addr = 0;



void createVersionPacket(uint8_t destAddr, uint8_t *buf)
{
	buf[MRBUS_PKT_DEST] = destAddr;
	buf[MRBUS_PKT_SRC] = mrbus_dev_addr;
	buf[MRBUS_PKT_LEN] = 15;
	buf[MRBUS_PKT_TYPE] = 'v';
	buf[6]  = MRBUS_VERSION_WIRELESS;
	// Software Revision
	buf[7]  = 0xFF & ((uint32_t)(GIT_REV))>>16; // Software Revision
	buf[8]  = 0xFF & ((uint32_t)(GIT_REV))>>8; // Software Revision
	buf[9]  = 0xFF & (GIT_REV); // Software Revision
	buf[10]  = HWREV_MAJOR; // Hardware Major Revision
	buf[11]  = HWREV_MINOR; // Hardware Minor Revision
	buf[12] = 'C';
	buf[13] = 'S';
	buf[14] = 'T';
}

volatile uint8_t ticks;
volatile uint16_t decisecs = 0;

void initialize100HzTimer(void)
{
	// Set up timer 0 for 100Hz interrupts
	TCNT0 = 0;
	OCR0A = 195;  // 20MHz / 1024 / 195 = 100.16Hz
	ticks = 0;
	decisecs = 0;
	TCCR0A = _BV(WGM01);
	TCCR0B = _BV(CS02) | _BV(CS00);
	TIMSK0 |= _BV(OCIE0A);
}

ISR(TIMER0_COMPA_vect)
{
	if (++ticks >= 10)  // 100ms
	{
		ticks = 0;
		decisecs++;
	}
}

void init(void)
{
	// Clear watchdog (in the case of an 'X' packet reset)
	MCUSR = 0;
#ifdef ENABLE_WATCHDOG
	// If you don't want the watchdog to do system reset, remove this chunk of code
	wdt_reset();
	WDTCSR |= _BV(WDE) | _BV(WDCE);
	WDTCSR = _BV(WDE) | _BV(WDP2) | _BV(WDP1); // Set the WDT to system reset and 1s timeout
	wdt_reset();
#else
	wdt_reset();
	wdt_disable();
#endif

	initialize100HzTimer();
}

int main(void)
{
	uint8_t headlightOn = 0;
	uint16_t decisecs_tmp = 0;

	uint8_t mrbusBuffer[MRBUS_BUFFER_SIZE];
	uint8_t xpressnetBuffer[XPRESSNET_BUFFER_SIZE];
	
	init();

	wdt_reset();

	// Initialize MRBus core
	mrbusPktQueueInitialize(&mrbeeTxQueue, mrbusTxPktBufferArray, MRBUS_TX_BUFFER_DEPTH);
	mrbusPktQueueInitialize(&mrbeeRxQueue, mrbusRxPktBufferArray, MRBUS_RX_BUFFER_DEPTH);
	mrbeeInit();

	xpressnetPktQueueInitialize(&xpressnetTxQueue, xpressnetTxPktBufferArray, XPRESSNET_TX_BUFFER_DEPTH);
	xpressnetInit(XPRESSNET_ADDRESS);

	sei();	

	wdt_reset();

	// Fire off initial reset version packet
	createVersionPacket(0xFF, mrbusBuffer);
	mrbusPktQueuePush(&mrbeeTxQueue, mrbusBuffer, mrbusBuffer[MRBUS_PKT_LEN]);

	wdt_reset();

#ifdef DEBUG
	debugInit();
#endif

	while(1)
	{
		wdt_reset();
		
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
		{
			decisecs_tmp = decisecs;
		}
		if(decisecs_tmp >= 10)
		{
			ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
			{
				decisecs = 0;
			}

			headlightOn ^= 0x01;

			xpressnetBuffer[0] = 0xE4;  // Loco 152, FN1
			xpressnetBuffer[1] = 0x20;
			xpressnetBuffer[2] = 0xC0;
			xpressnetBuffer[3] = 0x98;
			if(headlightOn)
			{
				xpressnetBuffer[4] = 0x01;
			}
			else
			{
				xpressnetBuffer[4] = 0x00;
			}
			xpressnetPktQueuePush(&xpressnetTxQueue, xpressnetBuffer, 5);

			xpressnetBuffer[0] = 0xE4;  // Loco 340, FN0
			xpressnetBuffer[1] = 0x20;
			xpressnetBuffer[2] = 0xC1;
			xpressnetBuffer[3] = 0x54;
			if(headlightOn)
			{
				xpressnetBuffer[4] = 0x10;
			}
			else
			{
				xpressnetBuffer[4] = 0x00;
			}
			xpressnetPktQueuePush(&xpressnetTxQueue, xpressnetBuffer, 5);
		}

		if (mrbusPktQueueDepth(&mrbeeTxQueue))
		{
			wdt_reset();
#ifndef DEBUG
			mrbeeTransmit();
#endif
		}

		if(xpressnetPktQueueDepth(&xpressnetTxQueue))
		{
			// Packet pending, so queue it up for transmit
			wdt_reset();
			xpressnetTransmit();
		}
	}

}


