/*
 * printf.c
 *
 * Created: 21/10/2020 15:23:32
 *  Author: Pablo
 */ 

#include "include_app/conversor_i2c_420ma.h"

#define PRINTF_BUFFER_SIZE        256U
static uint8_t stdout_buff[PRINTF_BUFFER_SIZE];

//-----------------------------------------------------------------------------------
int xprintf( const char *fmt, ...)
{

	va_list args;
	int i = 0;

	memset(stdout_buff,'\0',PRINTF_BUFFER_SIZE);
	va_start(args, fmt);
	vsnprintf( (char *)stdout_buff,sizeof(stdout_buff),fmt,args);
	USART0_sendString( (char *)stdout_buff);

	return(i);

}
//-----------------------------------------------------------------------------------