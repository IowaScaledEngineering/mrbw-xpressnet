/*************************************************************************
Title:    XpressNet Atmel AVR
Authors:  Michael Petersen <railfan@drgw.net>, Colorado, USA
          Nathan Holmes <maverick@drgw.net>, Colorado, USA
File:     xpressnet-avr.h
License:  GNU General Public License v3

LICENSE:
    Copyright (C) 2017 Nathan Holmes, Michael Petersen

    UART code derived from AVR UART library by Peter Fleury, and as
    modified by Tim Sharpe.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    
    You should have received a copy of the GNU General Public License along
    with this program. If not, see http://www.gnu.org/licenses/
    
*************************************************************************/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include "xpressnet.h"

static volatile uint8_t xpressnetTxBuffer[XPRESSNET_BUFFER_SIZE];
static volatile uint8_t xpressnetTxLength=0;
static volatile uint8_t xpressnetTxIndex=0;
XpressNetPktQueue xpressnetTxQueue;

ISR(USART0_TX_vect)
{
	// Transmit is complete: terminate
	XPRESSNET_PORT &= ~_BV(XPRESSNET_TXE);  // Disable driver
	// Disable the various transmit interrupts and the transmitter itself
	// Re-enable receive interrupt
	XPRESSNET_UART_CSR_B = (XPRESSNET_UART_CSR_B & ~(_BV(XPRESSNET_TXCIE) | _BV(XPRESSNET_UART_UDRIE))) | _BV(XPRESSNET_RXCIE);
}

ISR(XPRESSNET_UART_TX_INTERRUPT)
{
	XPRESSNET_UART_DATA = xpressnetTxBuffer[xpressnetTxIndex++];  //  Get next byte and write to UART

	if (xpressnetTxIndex >= XPRESSNET_BUFFER_SIZE || xpressnetTxLength == xpressnetTxIndex)
	{
		//  Done sending data to UART, disable UART interrupt
		XPRESSNET_UART_CSR_A |= _BV(XPRESSNET_TXC);
		XPRESSNET_UART_CSR_B &= ~_BV(XPRESSNET_UART_UDRIE);
		XPRESSNET_UART_CSR_B |= _BV(XPRESSNET_TXCIE);
	}
}

uint8_t xpressnetTxActive() 
{
	return(XPRESSNET_UART_CSR_B & (_BV(XPRESSNET_UART_UDRIE) | _BV(XPRESSNET_TXCIE)));
}

uint8_t xpressnetTransmit(void)
{
	uint8_t i;

	if (xpressnetPktQueueEmpty(&xpressnetTxQueue))
		return(0);

	//  Return if bus already active.
	if (xpressnetTxActive())
		return(1);

	xpressnetTxLength = xpressnetPktQueuePeek(&xpressnetTxQueue, (uint8_t*)xpressnetTxBuffer, sizeof(xpressnetTxBuffer));

	// If we have no packet length, or it's less than the header, just silently say we transmitted it
	// On the AVRs, if you don't have any packet length, it'll never clear up on the interrupt routine
	// and you'll get stuck in indefinite transmit busy
	if (0 == xpressnetTxLength)
	{
		xpressnetPktQueueDrop(&xpressnetTxQueue);
		return(0);
	}

	// First Calculate XOR
	uint8_t xor_byte = 0;
	for (i=0; i<=xpressnetTxLength; i++)
	{
		xor_byte ^= xpressnetTxBuffer[i];
	}
	xpressnetTxBuffer[xpressnetTxLength++] = xor_byte;  // Put it at the end

	xpressnetTxIndex = 0;

	ATOMIC_BLOCK(ATOMIC_FORCEON)
	{
		/* Enable transmitter since control over bus is assumed */
//		MRBUS_UART_CSR_B |= _BV(MRBUS_TXEN);
		XPRESSNET_UART_CSR_A |= _BV(XPRESSNET_TXC);
		XPRESSNET_PORT |= _BV(XPRESSNET_TXE);

		// Disable receive interrupt while transmitting
		XPRESSNET_UART_CSR_B &= ~_BV(XPRESSNET_RXCIE);

		// Enable transmit interrupt
		XPRESSNET_UART_CSR_B |= _BV(XPRESSNET_UART_UDRIE);
	}

	xpressnetPktQueueDrop(&xpressnetTxQueue);

	return(0);
}


void xpressnetInit(void)
{
#undef BAUD
#define BAUD XPRESSNET_BAUD
#include <util/setbaud.h>

#if defined( XPRESSNET_AT90_UART )
	// FIXME - probably need more stuff here
	UBRR = (uint8_t)UBRRL_VALUE;

#elif defined( XPRESSNET_ATMEGA_USART_SIMPLE )
	XPRESSNET_UART_UBRR = UBRR_VALUE;
	XPRESSNET_UART_CSR_A = (USE_2X)?_BV(U2X):0;
	XPRESSNET_UART_CSR_B = 0;
	XPRESSNET_UART_CSR_C = _BV(URSEL) | _BV(UCSZ1) | _BV(UCSZ0);
	
#elif defined( XPRESSNET_ATMEGA_USART0_SIMPLE )
	XPRESSNET_UART_UBRR = UBRR_VALUE;
	XPRESSNET_UART_CSR_A = (USE_2X)?_BV(U2X0):0;
	XPRESSNET_UART_CSR_B = 0;
	XPRESSNET_UART_CSR_C = _BV(URSEL0) | _BV(UCSZ01) | _BV(UCSZ00);
	
#elif defined( XPRESSNET_ATMEGA_USART ) || defined ( XPRESSNET_ATMEGA_USART0 )
	XPRESSNET_UART_UBRR = UBRR_VALUE;
	XPRESSNET_UART_CSR_A = (USE_2X)?_BV(U2X0):0;
	XPRESSNET_UART_CSR_B = _BV(UCSZ02);
	XPRESSNET_UART_CSR_C = _BV(UCSZ01) | _BV(UCSZ00);

#elif defined( XPRESSNET_ATTINY_USART )
	// Top four bits are reserved and must always be zero - see ATtiny2313 datasheet
	// Also, H and L must be written independently, since they're non-adjacent registers
	// on the attiny parts
	XPRESSNET_UART_UBRRH = 0x0F & UBRRH_VALUE;
	XPRESSNET_UART_UBRRL = UBRRL_VALUE;
	XPRESSNET_UART_CSR_A = (USE_2X)?_BV(U2X):0;
	XPRESSNET_UART_CSR_B = 0;
	XPRESSNET_UART_CSR_C = _BV(UCSZ1) | _BV(UCSZ0);

#elif defined ( XPRESSNET_ATMEGA_USART1 )
	XPRESSNET_UART_UBRR = UBRR_VALUE;
	XPRESSNET_UART_CSR_A = (USE_2X)?_BV(U2X1):0;
	XPRESSNET_UART_CSR_B = _BV(UCSZ12);
	XPRESSNET_UART_CSR_C = _BV(UCSZ11) | _BV(UCSZ10);
#else
#error "UART for your selected part is not yet defined..."
#endif

#undef BAUD

	/* Enable USART receiver and transmitter and receive complete interrupt */
	UCSR0B |= (_BV(RXCIE0) | _BV(RXEN0) | _BV(TXEN0));

	XPRESSNET_DDR &= ~(_BV(XPRESSNET_RX) | _BV(XPRESSNET_TX));  // Set RX and TX as inputs
	XPRESSNET_DDR |= _BV(XPRESSNET_TXE);  // Set driver enable as output
	XPRESSNET_PORT &= ~(_BV(XPRESSNET_TXE));  // Disable driver
}


