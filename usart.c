/*
 * usart.c
 *
 * Created: 21/10/2020 12:21:17
 *  Author: Pablo
 */ 

#include "include_app/conversor_i2c_420ma.h"


#define USART0_BAUD_RATE(BAUD_RATE) ((float)(F_CPU * 64 / (16 * (float)BAUD_RATE)) + 0.5);

// ---------------------------------------------------------------
void USART0_init(void)
{
	PORTA.DIR &= ~PIN1_bm;
	PORTA.DIR |= PIN0_bm;
	
	USART0.BAUD = (uint16_t)USART0_BAUD_RATE(115200);
	USART0.CTRLB |= USART_RXEN_bm | USART_TXEN_bm;
}
// ---------------------------------------------------------------
void USART0_sendChar(char c)
{
	while (!(USART0.STATUS & USART_DREIF_bm))
	{
		;
	}
	USART0.TXDATAL = c;	
}
// ---------------------------------------------------------------
void USART0_sendString(char *str)
{
	for(size_t i = 0; i < strlen(str); i++)
	{
		USART0_sendChar(str[i]);
	}
}
// ---------------------------------------------------------------
char USART0_readChar(void)
{
	while (!(USART0.STATUS & USART_RXCIF_bm))
	{
		;
	}
	return USART0.RXDATAL;
}
// ---------------------------------------------------------------

