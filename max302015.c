/*
 * max302015.c
 *
 * Created: 24/11/2020 15:30:23
 *  Author: Pablo
 */ 
#include "./include_app/conversor_i2c_420ma.h"
#include "./include_os/atmel_start_pins.h"

#define BUSADDR_MAX30205	0x48

#define RAW2TEMP 0.00390625	

int8_t max30205_raw_read( char *data );

// ---------------------------------------------------------------
void MAX30205_init(void)
{
	I2C_reset();
}
// ---------------------------------------------------------------
bool max30205_read( float *temperatura )
{
	
int8_t xBytes = 0;
char buffer[2] = { 0 };
uint8_t msbPres = 0;
uint8_t lsbPres = 0;
uint16_t pcounts;
float temp;

	xBytes = max30205_raw_read( buffer );

	if ( xBytes == -1 ) {
		xprintf("ERROR: max30205_read: tsensor_read\r\n\0");
		*temperatura = -999;
		return(false);
	}

	if ( xBytes > 0 ) {
		msbPres = buffer[0]; //  & 0x3F;
		lsbPres = buffer[1];
		pcounts = (msbPres << 8) + lsbPres;
		temp = RAW2TEMP * pcounts;
		
		if ( DEBUG_MED ) {
			xprintf("Temperatura: MSB[%d] LSB[%d]\r\n", msbPres, lsbPres);
			xprintf("             pcounts=%d\r\n", pcounts);
			xprintf("             temp=%.02f\r\n", temp);
		}
		
		*temperatura =temp;
		return(true);
	} else {
		return(false);
	}

	return(true);

}
//----------------------------------------------------------------
int8_t max30205_raw_read( char *data )
{

	int8_t rcode = 0;

	//I2C_reset(debug);
	
	//rcode =  I2C_read( BUSADDR_MAX30205, 0x00, 0x02, data );
	rcode =  I2C_master_read_NI  ( BUSADDR_MAX30205, 0x00, 1, data, 0x02 );
	if ( rcode == -1 ) {
		// Hubo error: trato de reparar el bus y reintentar la operacion
		// Espero 1s que se termine la fuente de ruido.
		_delay_ms(1000);
		// Reconfiguro los dispositivos I2C del bus que pueden haberse afectado
		xprintf("ERROR: MAX30205_raw_read recovering i2c bus \r\n\0" );
		I2C_reset();
	}
	return( rcode );
}
//----------------------------------------------------------------
void max30205_test(void)
{
	
char c;
float temperatura;

	xprintf("MAX30205 Test: Presione cualquier tecla...\r\n");

	while (1) {
		c = USART0_readChar(true);
		if ( max30205_read(&temperatura)) {
			xprintf("Temperatura:%.2f(oC)\r\n", temperatura );
		} else {
			xprintf("max30205 read error!!\r\n");
			MAX30205_init();
		}
	}
	
}
//----------------------------------------------------------------
