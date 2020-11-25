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
	// Configuro para 115200
	
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
char USART0_readChar(bool echo)
{
char c;

	while (!(USART0.STATUS & USART_RXCIF_bm))
	{
		wdt_reset();
		;
	}
	c = USART0.RXDATAL;
	if ( echo)
		USART0_sendChar(c);
	return(c);
}
// ---------------------------------------------------------------
bool USART0_getChar( char *c )
{

	if ( USART0.STATUS & USART_RXCIF_bm) {
		*c = USART0.RXDATAL;
		return(true);
	}
	return(false);
}
// ---------------------------------------------------------------
void uart_test(void)
{
	uint8_t count = 0;
	
	while (1)
	{

		wdt_reset();
		led_flash();
		xprintf("UART test: %03d\n\r", count++);
		_delay_ms(1000);
	}
}
// ---------------------------------------------------------------
