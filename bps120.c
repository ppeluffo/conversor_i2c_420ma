/*
 * bps120.c
 *
 * Created: 23/10/2020 15:50:13
 *  Author: Pablo
 */ 

#include "./include_app/conversor_i2c_420ma.h"
#include "./include_os/atmel_start_pins.h"

#define BUSADDR_BPS120					0x28

int8_t bps120_raw_read( char *data, bool debug );

// ---------------------------------------------------------------
void BPS120_init(void)
{
	I2C_reset(false);
}
// ---------------------------------------------------------------
bool bps120_read( float *presion, bool debug )
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
float h_cms = 0;

	xBytes = bps120_raw_read( buffer, debug );

	if ( xBytes == -1 ) {
		xprintf("ERROR: bps120_read: psensor_read\r\n\0");
		*presion = -99;
		return(false);
	}

	if ( xBytes > 0 ) {
		msbPres = buffer[0]  & 0x3F;
		lsbPres = buffer[1];
		pcounts = (msbPres << 8) + lsbPres;
		h_cms = ( pcounts - 1638 );
		h_cms /= 13107;
		h_cms *= 70;
		//if ( psi < 0 )
		//	psi = 0.0;
		xprintf("Presion: MSB[%d] LSB[%d]\r\n", msbPres, lsbPres);
		xprintf("Presion: pcounts=%d h(cms)=%.2f\r\n", pcounts, h_cms);

		*presion = h_cms;
		
		
		return(true);
	}

	return(true);

}
//----------------------------------------------------------------
int8_t bps120_raw_read( char *data, bool debug )
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

	I2C_reset(debug);
	
	rcode =  I2C_read( debug, BUSADDR_BPS120, 0x00, 0x02, data );
	if ( rcode == -1 ) {
		// Hubo error: trato de reparar el bus y reintentar la operacion
		// Espero 1s que se termine la fuente de ruido.
		_delay_ms(1000);
		// Reconfiguro los dispositivos I2C del bus que pueden haberse afectado
		xprintf("ERROR: BPS120_raw_read recovering i2c bus \r\n\0" );
		I2C_reset(debug);
	}
	return( rcode );
}
//----------------------------------------------------------------
