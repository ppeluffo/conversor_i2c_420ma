/*
 * dac.c
 *
 * Created: 22/10/2020 12:33:52
 *  Author: Pablo
 */ 

#include "include_app/conversor_i2c_420ma.h"

/* VREF start-up time */
#define VREF_STARTUP_TIME        (50)
/* Mask needed to get the 2 LSb for DAC Data Register */
#define LSB_MASK                 (0x03)

// ---------------------------------------------------------------
void DAC0_init(void)
{
	/* Disable digital input buffer */
	PORTD.PIN6CTRL &= ~PORT_ISC_gm;
	PORTD.PIN6CTRL |= PORT_ISC_INPUT_DISABLE_gc;
	/* Disable pull-up resistor */
	PORTD.PIN6CTRL &= ~PORT_PULLUPEN_bm;
	/* Enable DAC */
	/* Enable output buffer */
	/* Enable Run in Standby mode */
	DAC0.CTRLA = DAC_ENABLE_bm | DAC_OUTEN_bm | DAC_RUNSTDBY_bm;  
	    
	DAC0_setVal(0); 
}
// ---------------------------------------------------------------
void VREF_init(void)
{
	// Seteo el valor de referencia que va a usar el DAC
	// Como referencia usamos VREFA que el el voltaje en el pin PD7 y lo
	// genera el XTR111
	
	//VREF.DAC0REF = VREF_REFSEL_2V048_gc /* Select the 2.048V Internal Voltage Reference for DAC */
	 /* Set the Voltage Reference in Always On mode */
	//VREF.DAC0REF = VREF_REFSEL_VDD_gc | VREF_ALWAYSON_bm;   
	VREF.DAC0REF = VREF_REFSEL_VREFA_gc | VREF_ALWAYSON_bm;   
	/* Wait VREF start-up time */
	_delay_us(VREF_STARTUP_TIME);
}
// ---------------------------------------------------------------
void DAC0_setVal(uint16_t value)
{
	/* Store the two LSbs in DAC0.DATAL */
	DAC0.DATAL = (value & LSB_MASK) << 6;
	/* Store the eight MSbs in DAC0.DATAH */
	DAC0.DATAH = value >> 2;
}
//---------------------------------------------------------------
void dac_test(void)
{
	// Lee un valor por consola y genera el output correspondiente
	
char command[MAX_COMMAND_LEN];
uint8_t index = 0;
uint16_t dac_value = 0;
char c;

	while(1) {
		
		xprintf("DAC Test: Ingrese el valor dac(0-1023):?\r\n");
		xprintf("q-Salir\r\n");

		while (1) {
			c = USART0_readChar(true);
			if ( (c == 'q') || (c == 'Q')) {
				xprintf("\r\n");
				return;
			}
				
			if(c != '\n' && c != '\r') {
				command[index++] = c;
				if(index > MAX_COMMAND_LEN) {
					index = 0;
				}
			}
			
			if(c == '\r') {
				command[index] = '\0';
				index = 0;
				dac_value = atoi(command);
				if ( dac_value > 1023 ) {
					dac_value = 1023;
				}
				xprintf("Valor: %d\r\n", dac_value);
				DAC0_setVal(dac_value);
			}
		}
	}
}
//---------------------------------------------------------------