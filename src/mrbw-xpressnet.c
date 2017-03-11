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
#include "mrbee.h"

#define MY_ADDRESS 1

#define MRBUS_TX_BUFFER_DEPTH 16
#define MRBUS_RX_BUFFER_DEPTH 16

MRBusPacket mrbusTxPktBufferArray[MRBUS_TX_BUFFER_DEPTH];
MRBusPacket mrbusRxPktBufferArray[MRBUS_RX_BUFFER_DEPTH];

uint8_t mrbus_dev_addr = 0;


#define QUEUE_DEPTH 64

uint16_t rxBuffer[QUEUE_DEPTH];
uint8_t rxHeadIdx, rxTailIdx, rxBufferFull;

#define TX_BUFFER_SIZE 8

uint8_t txBuffer[TX_BUFFER_SIZE];

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


void serialInit(void)
{
#define BAUD 62500
#include <util/setbaud.h>
	UBRR0 = UBRR_VALUE;
	UCSR0A = (USE_2X)?_BV(U2X0):0;
	UCSR0B = _BV(UCSZ02);
	UCSR0C = _BV(UCSZ01) | _BV(UCSZ00);

	/* Enable USART receiver and transmitter and receive complete interrupt */
	UCSR0B |= (_BV(RXCIE0) | _BV(RXEN0) | _BV(TXEN0));

	DDRD &= ~(_BV(PD0) | _BV(PD1));  // Set RX and TX as inputs
	DDRD |= _BV(PD4);  // Set driver enable as output
	PORTD &= ~(_BV(PD4));  // Disable driver
}

ISR(USART0_RX_vect)
{
  uint16_t data = 0;

  if(UCSR0B & _BV(RXB80))
    data |= 0x0100;  // bit 9 set

  data |= UDR0;
  
  rxBufferPush(data);
}

volatile uint8_t txBufferIndex = 0;
uint8_t txBufferEnd = 0;


ISR(USART0_TX_vect)
{
	// Transmit is complete: terminate
	PORTD &= ~_BV(PD4);  // Disable driver
	// Disable the various transmit interrupts and the transmitter itself
	// Re-enable receive interrupt (might be killed if no loopback define is on...)
	UCSR0B = (UCSR0B & ~(_BV(TXCIE0) | _BV(UDRIE0))) | _BV(RXCIE0);
}

ISR(USART0_UDRE_vect)
{
	UDR0 = txBuffer[txBufferIndex++];  //  Get next byte and write to UART

	if ( (txBufferIndex >= txBufferEnd) || (txBufferIndex >= TX_BUFFER_SIZE) )
	{
		//  Done sending data to UART, disable UART interrupt
		UCSR0A |= _BV(TXC0);
		UCSR0B &= ~_BV(UDRIE0);
		UCSR0B |= _BV(TXCIE0);
	}
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

	serialInit();
}

uint8_t headlightOn = 0;

int main(void)
{
	uint16_t data;
	uint8_t txReady = 0;
	
	init();
	rxBufferInitialize();

	wdt_reset();

	// Initialize MRBus core
	mrbusPktQueueInitialize(&mrbeeTxQueue, mrbusTxPktBufferArray, MRBUS_TX_BUFFER_DEPTH);
	mrbusPktQueueInitialize(&mrbeeRxQueue, mrbusRxPktBufferArray, MRBUS_RX_BUFFER_DEPTH);
	mrbeeInit();

	sei();	

	wdt_reset();

	// Fire off initial reset version packet
	createVersionPacket(0xFF, txBuffer);
	mrbusPktQueuePush(&mrbeeTxQueue, txBuffer, txBuffer[MRBUS_PKT_LEN]);

	wdt_reset();


	while(1)
	{
		wdt_reset();
		
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
		{
			if(decisecs >= 10)
			{
				headlightOn ^= 0x01;
				decisecs = 0;

				txBuffer[0] = 0xE4;
				txBuffer[1] = 0x20;
				txBuffer[2] = 0xC0;
				txBuffer[3] = 0x98;
				if(headlightOn)
				{
					txBuffer[4] = 0x11;
					txBuffer[5] = 0x8D;
				}
				else
				{
					txBuffer[4] = 0x10;
					txBuffer[5] = 0x8C;
				}
				txBufferEnd = 6;
				txBufferIndex = 0;  //FIXME: atomic?

				txReady = 1;
			}
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
				if(txReady)
				{
					txReady = 0;
					UCSR0A |= _BV(TXC0);
					PORTD |= _BV(PD4);  // Enable driver
					UCSR0B |= _BV(UDRIE0);
				}
			}
		}

	}

}


