/*
 * bps120.c
 *
 * Created: 23/10/2020 15:50:13
 *  Author: Pablo
 */ 

#include "./include_app/conversor_i2c_420ma.h"
#include "./include_os/atmel_start_pins.h"

#define BUSADDR_BPS120		0x28

#define MAXCOUNT_BPS120	16384
#define PMAX_BPS120		1.00
#define PMIN_BPS120		0.15
#define OUTMAX_BPS120	( MAXCOUNT_BPS120 * 0.9 )
#define OUTMIN_BPS120	( MAXCOUNT_BPS120 * 0.1 )

int8_t bps120_raw_read( char *data );

// ---------------------------------------------------------------
void BPS120_init(void)
{
	I2C_reset();
}
// ---------------------------------------------------------------
bool bps120_read( float *presion, uint16_t *counts )
{
	// Lee el sensor usando bps120_raw_read que devuelve 2 bytes.
	// Los convierte a presion.
	// p(psi) = (pmax - pmin ) * ( counts - 0.1Max) / ( 0.8Max) + pmin
	// pmin = 0
	// Max = 16384 ( 14 bits )
	// 0.1xMax = 1638
	// 0.8xMax = 13107
	// counts es el valor leido del sensor.
	// PMAX = 1.0 psi
	// PMIN = 0 psi
		
int8_t xBytes = 0;
char buffer[2] = { 0 };
uint8_t msbPres = 0;
uint8_t lsbPres = 0;
uint16_t pcounts;
float psi = 0;
float h_cms = 0;

	xBytes = bps120_raw_read( buffer );

	if ( xBytes == -1 ) {
		xprintf("ERROR: bps120_read: psensor_read\r\n\0");
		*presion = -99;
		return(false);
	}

	if ( xBytes > 0 ) {
		msbPres = buffer[0]  & 0x3F;
		lsbPres = buffer[1];
		pcounts = (msbPres << 8) + lsbPres;
		*counts = pcounts;
		//psi = 1.0 * ( pcounts - OUTMIN_BPS120 ) * ( PMAX_BPS120 - PMIN_BPS120) / ( OUTMAX_BPS120 - OUTMIN_BPS120 ) + PMIN_BPS120;
		psi = PMAX_BPS120 * ( pcounts - bps120_offset) / OUTMAX_BPS120;
		h_cms = PSI_2_CMS * psi;
		
		if ( DEBUG_MED ) {
			xprintf("Presion: MSB[%d] LSB[%d]\r\n", msbPres, lsbPres);
			xprintf("         pcounts=%d\r\n", pcounts);
			xprintf("         psi=%.2f\r\n", psi);
			xprintf("         cms=%.2f\r\n", h_cms);
		}
		
		*presion = psi;
		return(true);
	} else {
		return(false);
	}

	return(true);

}
//----------------------------------------------------------------
int8_t bps120_raw_read( char *data )
{
	// EL sensor BOURNS bps120 mide presion diferencial.
	// Por lo tanto retorna PRESION de acuerdo a la funcion de transferencia
	// con la pMax la de la hoja de datos.
	// De acuerdo a la hoja de datos, la funcion de transferencia es:
	// p(psi) = (pmax - pmin ) * ( counts - 0.1Max) / ( 0.8Max) + pmin
	// pmin = 0
	// Max = 16384 ( 14 bits )
	// 0.1xMax = 1638
	// 0.8xMax = 13107
	// counts es el valor leido del sensor.
	// PMAX = 1.0 psi
	// PMIN = 0 psi

int8_t rcode = 0;

	//I2C_reset(debug);
	
	//rcode =  I2C_read( BUSADDR_BPS120, 0x00, 0x02, data );
	rcode =  I2C_master_read_NI  ( BUSADDR_BPS120, 0x00, 0, data, 0x02 );
	if ( rcode == -1 ) {
		// Hubo error: trato de reparar el bus y reintentar la operacion
		// Espero 1s que se termine la fuente de ruido.
		_delay_ms(1000);
		// Reconfiguro los dispositivos I2C del bus que pueden haberse afectado
		xprintf("ERROR: BPS120_raw_read recovering i2c bus \r\n\0" );
		I2C_reset();
	}
	return( rcode );
}
//----------------------------------------------------------------
void bps120_test(void)
{
char c;
float presion;
float h_cms;
uint16_t pcounts;

	xprintf("BPS120 Test: Presione cualquier tecla...\r\n");

	while (1) {
		c = USART0_readChar(true);
		if ( bps120_read(&presion, &pcounts)) {
			xprintf("Presion:%.2f(psi)\r\n", presion);
			h_cms = HMAX_CMS_BPS120 * presion;
			xprintf("Presion: %.02f(psi)\r\n", presion);
			xprintf("Presion: %.02f(cms)\r\n", h_cms);
		} else {
			xprintf("bps120 read error!!\r\n");
			BPS120_init();
		}
	}
	
}
//----------------------------------------------------------------
void bps120_calibrar(void)
{
float presion;
uint16_t pcounts, totalcounts;
uint8_t i, tc_H, tc_L;


	xprintf("BPS120: Calibracion en vacio...\r\n");
	totalcounts = 0;
	for (i=0; i<10; i++) {
		bps120_read( &presion, &pcounts );
		xprintf(".");
		wdt_reset();
		totalcounts += pcounts;
		_delay_ms(500);
	}
	totalcounts /= 10;
	xprintf("\r\nBPS120 Offset = %d\r\n", totalcounts);
	tc_H = (uint8_t)( (totalcounts>>8) & 0xFF);
	tc_L = (uint8_t)( totalcounts  &0xFF );
	xprintf("tc_H=0x%02x, tc_L=0x%02x", tc_H, tc_L);
	
	FLASH_0_write_eeprom_byte(1, tc_H  );
	FLASH_0_write_eeprom_byte(2, tc_L  );	
}
//----------------------------------------------------------------
