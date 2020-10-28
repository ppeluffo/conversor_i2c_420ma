/*
 * conversor_i2c_420ma.c
 *
 * Created: 23/10/2020 11:19:27
 * Author : Pablo
 */ 

#include "include_app/conversor_i2c_420ma.h"

// ---------------------------------------------------------------
int main(void)
{
    // Inicializo
	CLKCTRL_init();	// Configuro el clock para 24Mhz
	CONFIG_led();
	USART0_init();	// Configuro para 115200
	VREF_init();
	DAC0_init();
	i2c_init();
	
	APAGAR_LED();
	
	xprintf("conversor_i2c_420mA %s\r\n",  VERSION);
	
	//test_led();
	//test_uart();
	//test_dac();
	test_i2c();
}
// ---------------------------------------------------------------

