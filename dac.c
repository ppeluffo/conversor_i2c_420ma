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
	DAC0.CTRLA = DAC_ENABLE_bm          /* Enable DAC */
	| DAC_OUTEN_bm           /* Enable output buffer */
	| DAC_RUNSTDBY_bm;       /* Enable Run in Standby mode */
}
// ---------------------------------------------------------------
void VREF_init(void)
{
	//VREF.DAC0REF = VREF_REFSEL_2V048_gc /* Select the 2.048V Internal Voltage Reference for DAC */
	VREF.DAC0REF = VREF_REFSEL_VDD_gc
	| VREF_ALWAYSON_bm;    /* Set the Voltage Reference in Always On mode */
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
// ---------------------------------------------------------------