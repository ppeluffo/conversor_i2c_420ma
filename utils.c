/*
 * conf_utils.c
 *
 * Created: 23/10/2020 11:22:57
 *  Author: Pablo
 */ 

#include "include_app/conversor_i2c_420ma.h"
#include "include_os/protected_io.h"

// ---------------------------------------------------------------
static inline void ccp_write_io(void *addr, uint8_t value)
{
	protected_write_io(addr, CCP_IOREG_gc, value);
}
// ---------------------------------------------------------------
int8_t CLKCTRL_init(void)
{
	// Configuro el clock para 24Mhz
	
	ccp_write_io((void *)&(CLKCTRL.OSCHFCTRLA),
	CLKCTRL_FREQSEL_24M_gc         /* 24 */
	| 0 << CLKCTRL_AUTOTUNE_bp /* Auto-Tune enable: disabled */
	| 0 << CLKCTRL_RUNSTDBY_bp /* Run standby: disabled */);

	// ccp_write_io((void*)&(CLKCTRL.MCLKCTRLA),CLKCTRL_CLKSEL_OSCHF_gc /* Internal high-frequency oscillator */
	//		 | 0 << CLKCTRL_CLKOUT_bp /* System clock out: disabled */);

	// ccp_write_io((void*)&(CLKCTRL.MCLKLOCK),0 << CLKCTRL_LOCKEN_bp /* lock enable: disabled */);

	return 0;
}
// ---------------------------------------------------------------
void reset(void)
{
	/* Issue a Software Reset to initilize the CPU */
	ccp_write_io( (void *)&(RSTCTRL.SWRR), RSTCTRL_SWRF_bm );   
}
// ---------------------------------------------------------------
int8_t WDT_init()
{
	/* 8K cycles (8.2s) */
	/* Off */
	ccp_write_io((void *)&(WDT.CTRLA), WDT_PERIOD_8KCLK_gc | WDT_WINDOW_OFF_gc );
	return 0;
}
// ---------------------------------------------------------------
void cls(void)
{
	xprintf("\x1B[2J\0");
}
// ---------------------------------------------------------------
void IDSENSOR_init(void)
{
	/* Configura los pines de ID como entradas con pull-ups
		PA4 - IDC0
		PA5 - IDC1
		PA6 - IDC2
	*/
	
	PORTA.DIR &= ~(1 << PIN4_bp);
	PORTA.PIN4CTRL |= PORT_PULLUPEN_bm;
		
	PORTA.DIR &= ~(1 << PIN5_bp);
	PORTA.PIN5CTRL |= PORT_PULLUPEN_bm;
	
	PORTA.DIR &= ~(1 << PIN6_bp);
	PORTA.PIN6CTRL |= PORT_PULLUPEN_bm;
	
}
// ---------------------------------------------------------------
uint8_t idsensor_read(void)
{
uint8_t idc_pos = 0;

	idc_pos = (PORTA.IN & 0x70) >> 4;
	return(idc_pos);
}
// ---------------------------------------------------------------
void idsensor_test(void)
{
	// Lee un letra y si es 'r' lee los idc sensor pins

char c;
uint8_t idc_sensor;

	xprintf("IDC Test: Presione cualquier tecla...\r\n");

	while (1) {
		c = USART0_readChar(true);
		idc_sensor = idsensor_read();
		xprintf("IDC=%02x\r\n",idc_sensor);
	}
}
// ---------------------------------------------------------------
bool mag2current(float mag4, float mag20, float mag )
{
	// Convierte un valor a 4-20mA
	
bool retS = false;
float dac;
	
	dac = 204.0 + (mag - mag4) * ( 1023.0 - 204.0 ) / (mag20 - mag4);
	if ( DEBUG_MED )
		xprintf("dac:%d\r\n", (uint16_t)dac);
		
	if ( dac > 1023) {
		xprintf("ERROR dac.!!\r\n");
	} else {
		DAC0_setVal((uint16_t)dac);
		retS = true;
	}
	return(retS);
}
// ---------------------------------------------------------------
void config_default(void)
{
	debug_level = DEBUG_LEVEL_OFF;
}
// ---------------------------------------------------------------
