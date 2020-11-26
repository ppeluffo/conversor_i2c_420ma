/*
 * conversor_i2c_420ma.c
 *
 * Created: 23/10/2020 11:19:27
 * Author : Pablo
 */ 

#include "include_app/conversor_i2c_420ma.h"

void tk_medir(void);
void medirMagnitud(void);
void tk_control(void);
t_command_codes cmdstr2code ( char *command);
void executeCommand(t_command_codes cmd_code);
void menu_debug(void);
void print_banner(void);
void SENSOR_init(void);

// ---------------------------------------------------------------
int main(void)
{
uint8_t tc_H, tc_L;

    // Inicializo
	WDT_init();
	CLKCTRL_init();
	LED_init();
	USART0_init();
	VREF_init();
	DAC0_init();
	
	IDSENSOR_init();
	sensor_id = idsensor_read();
	
	I2C_init();
	SENSOR_init();

	//led_test();
	//uart_test();
	//dac_test();
	//idsensor_test();
	//bps120_test();
	//npa730_test();
	
	debug_level = FLASH_0_read_eeprom_byte(0);
	tc_H = FLASH_0_read_eeprom_byte(1);
	tc_L = FLASH_0_read_eeprom_byte(2);
	xprintf("tc_H=0x%02x, tc_L=0x%02x", tc_H, tc_L);
	
	bps120_offset = (tc_H << 8) + tc_L;
	//bps120_offset = 100;
	print_banner();

	switch( debug_level ) {
		case DEBUG_LEVEL_OFF:
			xprintf("Log level OFF.\r\n");
			break;
		case DEBUG_LEVEL_LOW:
			xprintf("Log level LOW.\r\n");
			break;
		case DEBUG_LEVEL_MED:
			xprintf("Log level MEDIO.\r\n");
			break;
		case DEBUG_LEVEL_HIGH:
			xprintf("Log level HIGH.\r\n");
			break;
		default:
			debug_level = DEBUG_LEVEL_LOW;
			xprintf("Log level ERROR. Set to LOW !!\r\n");
			break;
	}

	//debug_level = DEBUG_LEVEL_MED;
	
	while(1) {
		tk_medir();
		tk_control();
	}
}
// ---------------------------------------------------------------
void tk_medir(void)
{
	// Mido en forma continua c/1sec.
	// Si presiono ESC paso al menu de control
	
char rxChar;
float presion;
float h_cms;

	xprintf("Ingreso a 'tk_medir'.\r\n");
	
	while(1) {
		wdt_reset();
		_delay_ms(2000);
		medirMagnitud();
		
		// Si presiono ESC salgo y voy a tk_control
		if ( USART0_getChar( &rxChar)) {
			xprintf("RX=0x%02X\r\n",rxChar);
			if (rxChar == ESC)
				return;
		}
	}
}
// ---------------------------------------------------------------
void medirMagnitud(void)
{
float magnitud;
float h_cms, temp;
uint16_t pcounts;

	switch (sensor_id) {
		case SENSOR_BPS120:
			bps120_read( &magnitud, &pcounts );
			h_cms = PSI_2_CMS * magnitud;
			if ( DEBUG_LOW ) {
				xprintf("Presion: %.02f(psi)\r\n", magnitud);
				xprintf("         %.02f(cms)\r\n", h_cms);
			}
			//
			mag2current( 0, 70.3 , h_cms );
			break;
		case SENSOR_NPA730:
			npa730_read( &magnitud );
			h_cms = PSI_2_CMS * magnitud;
			if ( DEBUG_LOW ) {
				xprintf("Presion: %.02f(psi)\r\n", magnitud );
				xprintf("         %.02f(cms)\r\n", h_cms);
			}
			//
			mag2current( 210.8, 1054.6 , h_cms );
			break;
		case SENSOR_MAX30205:
			max30205_read( &magnitud );
			temp = magnitud;
			if ( DEBUG_LOW ) {
				xprintf("Temp: %.02f(oC)\r\n", temp);
			}
			//
			mag2current( 0, 50 , temp );
			break;
		default:
			DAC0_setVal(0);
			break;
	}	
}
// ---------------------------------------------------------------
void tk_control(void)
{
	
char command[MAX_COMMAND_LEN];
uint8_t index = 0;
char c;

	cls();
	xprintf("Ingreso a 'tk_control'.\r\n");
	
	while(1) {
		
		xprintf("Comandos: (1)help,(2)medir,(3)led,(4)cls,(5)reset,(6)debug,(7)calibrar, (8)dac ?\r\n");

		while (1) {
			c = USART0_readChar(true);	
			if(c != '\n' && c != '\r') {
				command[index++] = c;
				if(index > MAX_COMMAND_LEN) {
					index = 0;
				}
			}
			
			if(c == '\r') {
				command[index] = '\0';
				index = 0;
				executeCommand( cmdstr2code(command) );
				_delay_ms(500);
				xprintf("\r\nComandos: (1)help,(2)medir,(3)led,(4)cls,(5)reset,(6)debug,(7)calibrar, (8)dac ?\r\n");
			}
		}
	}
}
// ---------------------------------------------------------------
t_command_codes cmdstr2code ( char *command)
{
	// Convierte un string que representa un comando al codigo correspondiente
	
	// Verifico si el codigo es numerico
	switch ( command[0] - '0' ) {
	case CMDC_HELP:
		return(CMDC_HELP);
		break;
	case CMDC_MEDIR:
		return(CMDC_MEDIR);
		break;
	case CMDC_CLS:
		return(CMDC_CLS);
		break;
	case CMDC_LED:
		return(CMDC_LED);
		break;
	case CMDC_RESET:
		return(CMDC_RESET);
		break;
	case CMDC_DEBUG:
		return(CMDC_DEBUG);
		break;
	case CMDC_CALIBRAR:
		return(CMDC_CALIBRAR);
		break;
	case CMDC_DAC:
		return(CMDC_DAC);
		break;
	}
	
	// Controlo x strings.
	if (strcmp( strupr(command), "HELP") == 0) {
		return(CMDC_HELP);
	} else if (strcmp( strupr(command), "MEDIR") == 0) {
		return(CMDC_MEDIR);
	} else if (strcmp(strupr(command), "CLS") == 0) {
		return(CMDC_CLS);
	} else if (strcmp(strupr(command), "LED") == 0) {
		return(CMDC_LED);
	} else if (strcmp(strupr(command), "RESET") == 0) {
		return(CMDC_RESET);
	} else if (strcmp(strupr(command), "DEBUG") == 0) {
		return(CMDC_DEBUG);
	} else if (strcmp(strupr(command), "CALIBRAR") == 0) {
		return(CMDC_CALIBRAR);
	}  else if (strcmp(strupr(command), "DAC") == 0) {
		return(CMDC_DAC);
	}
			
	return(0);			
}
// ---------------------------------------------------------------
void executeCommand(t_command_codes cmd_code)
{

uint8_t i;
 
	switch(cmd_code) {
		case  CMDC_HELP:
			print_banner();
			xprintf("Comandos:\r\n");
			xprintf("  1-help:     Muestra esta ayuda\r\n");
			xprintf("  2-medir:    Lee el sensor\r\n");
			xprintf("  3-led:      Flashea el led 3 veces\r\n");
			xprintf("  4-cls:      Borra la pantalla\r\n");
			xprintf("  5-reset:    Reinicia el dispositivo en 8secs.\r\n");
			xprintf("  6-debug:    Setea el nivel de debug\r\n");
			xprintf("  7-calibrar: Calibra el offset y span del conversor\r\n");
			xprintf("  8-dac:      Prueba el DAC.\r\n");
			xprintf("OK\r\n");
			return;
			break;
		
		case CMDC_MEDIR:
			//xprintf("Comando PRESION\n\r");
			medirMagnitud();
			xprintf("OK\r\n");
			return;
			break;
			
		case CMDC_LED:
			// Flashea 3 veces el led
			//xprintf("Comando LED\n\r");
			for (i=0;i<3;i++) {
				led_flash();
				_delay_ms(1000);
			}
			xprintf("OK\r\n");
			return;
			break;
			
		case CMDC_CLS:
			// Borro la pantalla
			//xprintf("Comando CLS\n\r");
			cls();
			return;
			break;		
			
		case CMDC_RESET:
			// Me reseteo en 8s al expirar el wdg
			//xprintf("Comando RESET\n\r");
			cls();
			while(1) {
				xprintf(".");
				_delay_ms(1000);
			}
			return;
			break;
			
		case CMDC_DEBUG:
			//xprintf("Comando DEBUG\n\r");
			menu_debug();
			xprintf("OK\r\n");
			return;
			break;
			
		case CMDC_CALIBRAR:
			//xprintf("Comando CALIBRAR\n\r");
			modo_calibrar();
			xprintf("OK\r\n");
			return;
			break;

		case CMDC_DAC:
		xprintf("Comando DAC\n\r");
		dac_test();
		xprintf("OK\r\n");
		return;
		break;
					
		default:
			xprintf("Incorrect command.\r\n");
			return;
			break;
	}

}
// ---------------------------------------------------------------
void menu_debug(void)
{
char c;

	xprintf("Configuracion de nivel de debug.\r\n");
	xprintf("Nivel actual de debug: ");
	switch( debug_level ) {
		case DEBUG_LEVEL_OFF:
			xprintf("OFF\r\n");
			break;
		case DEBUG_LEVEL_LOW:
			xprintf("LOW\r\n");
			break;
		case DEBUG_LEVEL_MED:
			xprintf("MEDIO\r\n");
			break;
		case DEBUG_LEVEL_HIGH:
			xprintf("HIGH\r\n");
			break;
		default:
			debug_level = DEBUG_LEVEL_OFF;
			xprintf("LOW\r\n");	
			break;	
	}
	
	while(1) {
		xprintf("Seleccione nuevo nivel: 0 (off), 1 (low), 2 (medio), 3 (high)\r\n");
		c = USART0_readChar(true);
		xprintf("\r\n");	
		switch( c - '0') {
		case 0:
			debug_level = DEBUG_LEVEL_OFF;
			FLASH_0_write_eeprom_byte(0, debug_level );
			return;
		case 1:
			debug_level = DEBUG_LEVEL_LOW;
			FLASH_0_write_eeprom_byte(0, debug_level );
			return;
		case 2:
			debug_level = DEBUG_LEVEL_MED;
			FLASH_0_write_eeprom_byte(0, debug_level );
			return;
		case 3:
			debug_level = DEBUG_LEVEL_HIGH;
			FLASH_0_write_eeprom_byte(0, debug_level );
			return;
		default:
			xprintf("ERROR: el valor debe estar entre 0..3\r\n");
			break;
		}
	}
}
// ---------------------------------------------------------------
void modo_calibrar(void)
{
char c;

	xprintf("Calibracion de span.\r\n");
	while(1) {
		xprintf("(c)cero, (s)span, (b)bps120, (q)salir\r\n");
		c = USART0_readChar(true);
		xprintf("\r\n");
		switch( toupper(c)) {
		case 'C':
			DAC0_setVal(0);
			break;
		case 'S':
			DAC0_setVal(1023);
			break;
		case 'B':
			bps120_calibrar();
			break;
		case 'Q':
			xprintf("Exit modo calibrar.\r\n");
			return;
		}
	}	
}
// ---------------------------------------------------------------
void print_banner(void)
{
	xprintf("\r\n");
	xprintf("I2C to 4-20ma converter.\r\n");
	xprintf("Version: %s @ %s\r\n", FW_VERSION, FW_FECHA);
	xprintf("Log_level 0x%02x\r\n", debug_level);

	switch ( sensor_id ) {
		case SENSOR_BPS120:
		xprintf("Converter init. Sensor Pressure BPS120\r\n");
		xprintf("BPS120 Offset: %d\r\n", bps120_offset);
		break;
	case SENSOR_NPA730:
		xprintf("Converter init. Sensor Pressure NPA730\r\n");
		break;
	case SENSOR_MAX30205:
		xprintf("Converter init. Sensor Temp. MAX3025\r\n");
		break;
	default:
		xprintf("Sensor no detectado (id=%d)\r\n", sensor_id );
		break;
	}
}
// ---------------------------------------------------------------
void SENSOR_init(void)
{	
	switch ( sensor_id ) {
	case SENSOR_BPS120:
		BPS120_init();
		break;
	case SENSOR_NPA730:
		NPA730_init();
		break;
	case SENSOR_MAX30205:
		MAX30205_init();
		break;
	default:
		xprintf("Sensor no detectado (id=%d)\r\n", sensor_id );
	break;
	}
}
// ---------------------------------------------------------------