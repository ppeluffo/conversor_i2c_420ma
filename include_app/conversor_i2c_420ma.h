/*
 * conversor_i2c_420ma.h
 *
 * Created: 23/10/2020 11:23:46
 *  Author: Pablo
 */ 


#ifndef CONVERSOR_I2C_420MA_H_
#define CONVERSOR_I2C_420MA_H_

#define VERSION "R001 @ 2020-10-28" 

#define F_CPU 24000000

#include <avr/io.h>
#include <util/delay.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "../i2c/i2c_simple_master.h"

int8_t CLKCTRL_init(void);

#define APAGAR_LED() ( PORTD.OUT |= PIN7_bm )
#define PRENDER_LED() ( PORTD.OUT &= ~PIN7_bm )
void CONFIG_led(void);
void test_led(void);

void USART0_init(void);
void USART0_sendChar(char c);
void USART0_sendString(char *str);
char USART0_readChar(void);
void test_uart(void);

int xprintf( const char *fmt, ...);

void DAC0_init(void);
void VREF_init(void);
void DAC0_setVal(uint16_t value);
void test_dac(void);

void i2c_init(void);
void executeCommand_i2c(char *command);
void test_i2c(void);
bool bps120_read( float *presion );


#define MAX_COMMAND_LEN	8

#endif /* CONVERSOR_I2C_420MA_H_ */