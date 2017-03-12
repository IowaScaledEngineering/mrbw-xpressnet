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

#define MY_ADDRESS 1

#define MRBUS_TX_BUFFER_DEPTH 16
#define MRBUS_RX_BUFFER_DEPTH 16

MRBusPacket mrbusTxPktBufferArray[MRBUS_TX_BUFFER_DEPTH];
MRBusPacket mrbusRxPktBufferArray[MRBUS_RX_BUFFER_DEPTH];

#define XPRESSNET_TX_BUFFER_DEPTH 16

XpressNetPacket xpressnetTxPktBufferArray[XPRESSNET_TX_BUFFER_DEPTH];


uint8_t mrbus_dev_addr = 0;


#define QUEUE_DEPTH 64

uint16_t rxBuffer[QUEUE_DEPTH];
uint8_t rxHeadIdx, rxTailIdx, rxBufferFull;

void rxBufferInitialize(void)
{
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    rxHeadIdx = rxTailIdx = 0;
    rxBufferFull = 0;
  }
}

uint8_t rxBufferDepth(void)
{
  uint8_t result;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    if(rxBufferFull)
      return(QUEUE_DEPTH);
    result = ((uint8_t)(rxHeadIdx - rxTailIdx) % QUEUE_DEPTH);
  }
  return(result);
}

uint8_t rxBufferPush(uint16_t data)
{
    // If full, bail with a false
    if (rxBufferFull)
      return(0);
  
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    rxBuffer[rxHeadIdx] = data;
  
    if( ++rxHeadIdx >= QUEUE_DEPTH )
      rxHeadIdx = 0;
    if (rxHeadIdx == rxTailIdx)
      rxBufferFull = 1;
  }
  return(1);
}

uint16_t rxBufferPop(uint8_t snoop)
{
  uint16_t data;
    if (0 == rxBufferDepth())
      return(0);
  
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    data = rxBuffer[rxTailIdx];
    if(!snoop)
    {
      if( ++rxTailIdx >= QUEUE_DEPTH )
        rxTailIdx = 0;
      rxBufferFull = 0;
    }
  }
  return(data);
}

ISR(USART0_RX_vect)
{
  uint16_t data = 0;

  if(XPRESSNET_UART_CSR_B & _BV(XPRESSNET_RXB8))
    data |= 0x0100;  // bit 9 set

  data |= XPRESSNET_UART_DATA;
  
  rxBufferPush(data);
}


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
	uint16_t data;
	uint8_t headlightOn = 0;
	uint16_t decisecs_tmp = 0;

	uint8_t mrbusBuffer[MRBUS_BUFFER_SIZE];
	uint8_t xpressnetBuffer[XPRESSNET_BUFFER_SIZE];
	
	init();
	rxBufferInitialize();

	wdt_reset();

	// Initialize MRBus core
	mrbusPktQueueInitialize(&mrbeeTxQueue, mrbusTxPktBufferArray, MRBUS_TX_BUFFER_DEPTH);
	mrbusPktQueueInitialize(&mrbeeRxQueue, mrbusRxPktBufferArray, MRBUS_RX_BUFFER_DEPTH);
	mrbeeInit();

	xpressnetPktQueueInitialize(&xpressnetTxQueue, xpressnetTxPktBufferArray, XPRESSNET_TX_BUFFER_DEPTH);
	xpressnetInit();

	sei();	

	wdt_reset();

	// Fire off initial reset version packet
	createVersionPacket(0xFF, mrbusBuffer);
	mrbusPktQueuePush(&mrbeeTxQueue, mrbusBuffer, mrbusBuffer[MRBUS_PKT_LEN]);

	wdt_reset();


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

			xpressnetBuffer[0] = 0xE4;
			xpressnetBuffer[1] = 0x20;
			xpressnetBuffer[2] = 0xC0;
			xpressnetBuffer[3] = 0x98;
			if(headlightOn)
			{
				xpressnetBuffer[4] = 0x11;
			}
			else
			{
				xpressnetBuffer[4] = 0x10;
			}
			xpressnetPktQueuePush(&xpressnetTxQueue, xpressnetBuffer, 5);
		}

		if (mrbusPktQueueDepth(&mrbeeTxQueue))
		{
			wdt_reset();
			mrbeeTransmit();
		}

		if(rxBufferDepth() > 0)
		{
			data = rxBufferPop(0);
			if((0x140 + MY_ADDRESS) == data)
			{
				// FIXME: check parity
				// FIXME: move into interrupt routine to avoid loading buffer except when for us?
				// Normal Inquiry to me
				if(xpressnetPktQueueDepth(&xpressnetTxQueue))
				{
					wdt_reset();
					xpressnetTransmit();
				}
			}
		}

	}

}


