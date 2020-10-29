/*
 * conf_utils.c
 *
 * Created: 23/10/2020 11:22:57
 *  Author: Pablo
 */ 

#include "include_app/conversor_i2c_420ma.h"
#include "include_os/protected_io.h"

void executeCommand(char *command);
bool pressure2current(bool debug);

// ---------------------------------------------------------------
static inline void ccp_write_io(void *addr, uint8_t value)
{
	protected_write_io(addr, CCP_IOREG_gc, value);
}
// ---------------------------------------------------------------
int8_t CLKCTRL_init(void)
{
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
void CONFIG_led(void)
{
	// LED es output
	PORTD.DIR = PIN7_bm;	// Seteo PD7 como output
}
// ---------------------------------------------------------------
void flash_led(void)
{
	PRENDER_LED();
	_delay_ms( 50 );
	APAGAR_LED();
}
// ---------------------------------------------------------------
void reset(void)
{
	
	ccp_write_io( (void *)&(RSTCTRL.SWRR), RSTCTRL_SWRF_bm );   /* Issue a Software Reset to initilize the CPU */
}
// ---------------------------------------------------------------
int8_t WDT_init()
{

	ccp_write_io((void *)&(WDT.CTRLA),
	WDT_PERIOD_8KCLK_gc /* 8K cycles (8.2s) */
	| WDT_WINDOW_OFF_gc /* Off */);

	return 0;
}
// ---------------------------------------------------------------
void cls(void)
{
	xprintf("\x1B[2J\0");
}
// ---------------------------------------------------------------
void test_led(void)
{
	while(1) {
		
		flash_led();
		_delay_ms( 950 );
	}
}
// ---------------------------------------------------------------
void test_uart(void)
{
	uint8_t count = 0;
	
	while (1)
	{
		//USART0_sendString("Hello World!\r\n");
		xprintf("Counter value is: %d\n\r", count++);
		//xprintf("Counter value is:\n\r");
		_delay_ms(500);
	}
}
// ---------------------------------------------------------------
void test_dac(void)
{
	
char command[MAX_COMMAND_LEN];
uint8_t index = 0;
uint16_t dacvalue = 0;
char c;

	DAC0_setVal(0);
	while(1) {
		xprintf("Ingrese el valor dac(0-1023):?\r\n");

		while (1) {
			c = USART0_readChar();
			//USART0_sendChar(c);
			if(c != '\n' && c != '\r') {
				command[index++] = c;
				if(index > MAX_COMMAND_LEN) {
					index = 0;
				}
			}
				
			if(c == '\r') {
				command[index] = '\0';
				index = 0;
				//executeCommand(command);
				dacvalue = atoi(command);
				xprintf("Valor: %d\r\n", dacvalue);
				DAC0_setVal(dacvalue);
			}
		}
	}
}
// ---------------------------------------------------------------
bool pressure2current(bool debug)
{
float presion;
bool retS = false;
float dac;
	
#ifdef BPS120
	if ( bps120_read(&presion,debug )) {
		xprintf("Presion:%.2f\r\n", presion);
		dac = ( 1023 - 204 )/70;
		dac *= presion;
		dac += 204;
		xprintf("dac:%d\r\n", (uint16_t)dac);
		if ( dac > 1023) {
			xprintf("ERROR dac.!!\r\n");
		} else {
			DAC0_setVal((uint16_t)dac);
			retS = true;
		}
	}
#endif
	return(retS);

}
// ---------------------------------------------------------------
void menu(void)
{
	
	char command[MAX_COMMAND_LEN];
	uint8_t index = 0;
	char c;

	xprintf("Spymovil: %s\r\n", VERSION);
	xprintf("conversor_i2c_420mA\r\n");
	
	while(1) {
		
		xprintf("cmds {presion,dpresion,led,cls,reset}?\r\n");

		while (1) {
			c = USART0_readChar();
			if(c != '\n' && c != '\r') {
				command[index++] = c;
				if(index > MAX_COMMAND_LEN) {
					index = 0;
				}
			}
			
			if(c == '\r') {
				command[index] = '\0';
				index = 0;
				executeCommand(command);
			}
		}
	}
}
// ---------------------------------------------------------------
void executeCommand(char *command)
{

	if (strcmp( strupr(command), "PRESION") == 0) {
		xprintf("Comando PRESION\n\r");
		if (!pressure2current(false)) {
			xprintf("Error !!\r\n");
		}	
	
	} else if (strcmp(strupr(command), "DPRESION") == 0) {
		xprintf("Comando DEBUG PRESION\n\r");
		if (!pressure2current(false)) {
			xprintf("Error !!\r\n");
		}
		
	} else if (strcmp(strupr(command), "QUIT") == 0) {
		xprintf("Comando QUIT\n\r");

	} else if (strcmp(strupr(command), "CLS") == 0) {
		xprintf("Comando CLS\n\r");
		cls();
	
	} else if (strcmp(strupr(command), "LED") == 0) {
		xprintf("Comando LED\n\r");
		flash_led();
		
	} else if (strcmp(strupr(command), "RESET") == 0) {
		xprintf("Comando RESET\n\r");
		cls();
		while(1) {
			xprintf(".");
			_delay_ms(1000);
		}
		
	} else {
		xprintf("Incorrect command.\r\n");
	}
	
	_delay_ms(500);
}
// ---------------------------------------------------------------


