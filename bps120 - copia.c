/*
 * bps120.c
 *
 * Created: 23/10/2020 15:50:13
 *  Author: Pablo
 */ 

#include "./include_app/conversor_i2c_420ma.h"
#include "./i2c/i2c_simple_master.h"
#include "./i2c/i2c_master.h"
#include "./i2c/atmel_start_pins.h"

#define BUSADDR_BPS120 0x28

#define ioctl_I2C_SET_DEVICE_ADDRESS		1
#define ioctl_I2C_SET_BYTE_ADDRESS			2
#define ioctl_I2C_SET_BYTE_ADDRESS_LENGTH	3
#define ioctl_I2C_GET_LAST_ERROR		4
#define ioctl_I2C_SCAN					5
#define ioctl_SET_TIMEOUT				6

#define I2C_OK			0
#define I2C_RD_ERROR	1
#define I2C_WR_ERROR	2

typedef struct {
	uint8_t devAddress;
	uint16_t dataStartAddress;
	uint8_t dataLength;
	uint8_t i2c_error_code;
} periferico_i2c_port_t;

periferico_i2c_port_t I2C_sctl;

#define SCL		0
#define SDA		1
#define I2C_MAXTRIES	5

#define ACK 	0
#define NACK 	1

int8_t bps120_raw_read( char *data );
int8_t I2C_read( uint8_t i2c_dev_address, uint16_t i2c_rdStartAddress, uint8_t dataLength, char *rdBuffer );
int drv_i2c_master_read  ( char *pvBuffer );

bool pvI2C_set_bus_idle(void);
bool pvI2C_write_slave_devaddress(const uint8_t devAddress);

bool pvI2C_write_slave_reg( const uint8_t devAddressLength, const uint16_t byteAddress );
bool pvI2C_write_data( const char txbyte);
bool pvI2C_read_slave( uint8_t response_flag, char *rxByte );
void i2c_reset(void);
bool  pvI2C_waitForComplete(void);

#define I2C_DIRECTION_BIT_WRITE                         0
#define I2C_DIRECTION_BIT_READ                          1
#define I2C_SLAVE_ADDRESS                               0x28

// ---------------------------------------------------------------
bool bps120_read( float *presion )
{
	// Lee el sensor usando bps120_raw_read que devuelve 2 bytes.
	// Los convierte a presion.
	
int8_t xBytes = 0;
char buffer[2] = { 0 };
uint8_t msbPres = 0;
uint8_t lsbPres = 0;

	xBytes = bps120_raw_read( buffer );

	if ( xBytes == -1 ) {
		xprintf("ERROR: bps120_read: psensor_read\r\n\0");
		*presion = -99;
		return(false);
	}

	if ( xBytes > 0 ) {
		msbPres = buffer[0]  & 0x3F;
		lsbPres = buffer[1];
		*presion = (msbPres << 8) + lsbPres;
		xprintf("Presion: MSB[%d] LSB[%d] PRES%.2f]\r\n", msbPres, lsbPres, *presion);
		return(true);
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

	rcode =  I2C_read( BUSADDR_BPS120, 0x00, 0x02, data );
	if ( rcode == -1 ) {
		// Hubo error: trato de reparar el bus y reintentar la operacion
		// Espero 1s que se termine la fuente de ruido.
		_delay_ms(1000);
		// Reconfiguro los dispositivos I2C del bus que pueden haberse afectado
		xprintf("ERROR: BPS120_raw_read recovering i2c bus \r\n\0" );
		i2c_reset();
	}
	return( rcode );
}
//----------------------------------------------------------------
void TWI_init(void)
{
	
	PF3_set_level(false);
	PF3_set_dir(PORT_DIR_OUT);
	PF3_set_pull_mode(PORT_PULL_OFF);
	PF3_set_inverted(false);
	PF3_set_isc(PORT_ISC_INTDISABLE_gc);

	PF2_set_level(false);
	PF2_set_dir(PORT_DIR_OUT);
	PF2_set_pull_mode(PORT_PULL_OFF);
	PF2_set_inverted(false);
	PF2_set_isc(PORT_ISC_INTDISABLE_gc);

	TWI1.MBAUD = (uint8_t)TWI1_BAUD(100000, 0); /* set MBAUD register */

	TWI1.MCTRLA = 1 << TWI_ENABLE_bp        /* Enable TWI Master: enabled */
	| 0 << TWI_QCEN_bp        /* Quick Command Enable: disabled */
	| 0 << TWI_RIEN_bp        /* Read Interrupt Enable: disabled */
	| 0 << TWI_SMEN_bp        /* Smart Mode Enable: disabled */
	| TWI_TIMEOUT_DISABLED_gc /* Bus Timeout Disabled */
	| 0 << TWI_WIEN_bp;       /* Write Interrupt Enable: disabled */
	
	TWI1.MCTRLB |= TWI_FLUSH_bm;
	TWI1.MSTATUS |= TWI_BUSSTATE_IDLE_gc;
	// Reset module
	TWI1.MSTATUS |= (TWI_RIF_bm | TWI_WIF_bm);

}
//----------------------------------------------------------------
int8_t I2C_read( uint8_t i2c_dev_address, uint16_t i2c_rdStartAddress, uint8_t dataLength, char *rdBuffer )
{

size_t xReturn = 0U;
uint8_t i2c_error_code;

	I2C_sctl.devAddress = i2c_dev_address;
	I2C_sctl.dataStartAddress = i2c_rdStartAddress;
	I2C_sctl.dataLength = dataLength;
	I2C_sctl.i2c_error_code = I2C_OK;

	xReturn = drv_i2c_master_read( rdBuffer );

	// Controlamos errores.
	i2c_error_code = I2C_sctl.i2c_error_code;
	if (i2c_error_code != I2C_OK ) {
		xprintf("ERROR: I2C_RD err = 0x0%X.\r\n\0", i2c_error_code );
		return(-1);
	}

	if (xReturn !=  dataLength ) {
		xprintf("ERROR: I2C_RD err: xbytes=%d, xReturn=%d.\r\n\0", dataLength, xReturn  );
		return(-1);
	}

	return(xReturn);

}
//------------------------------------------------------------------------------------
int i2c_open( void )
{
	TWI1.MCTRLB |= TWI_FLUSH_bm;
	TWI1.MSTATUS |= TWI_BUSSTATE_IDLE_gc;
	// Reset module
	TWI1.MSTATUS |= (TWI_RIF_bm | TWI_WIF_bm);
	return(1);
}
//------------------------------------------------------------------------------------
void i2c_reset(void)
{
	xprintf("i2c_reset\r\n");
	TWI1.MCTRLB |= TWI_FLUSH_bm;
	TWI1.MSTATUS |= TWI_BUSSTATE_IDLE_gc;
	// Reset module
	TWI1.MSTATUS |= (TWI_RIF_bm | TWI_WIF_bm);

}
//------------------------------------------------------------------------------------
void i2c_test(void)
{
	
uint8_t txByte;
uint8_t ticks_to_wait = 10;

	// 1: Configuro el periferico
	TWI1.MBAUD = (uint8_t)TWI1_BAUD(100000, 0);
	TWI1.MCTRLA = TWI_ENABLE_bm;
	TWI1.MSTATUS = TWI_BUSSTATE_IDLE_gc;

	// 2: Envio la slave address + SLA_w
	txByte = (I2C_SLAVE_ADDRESS <<1) + I2C_DIRECTION_BIT_READ; 
	xprintf("i2c: STATUS=0x%02x,\r\n", TWI1.MSTATUS );
	xprintf("i2c: TXBYTE=0x%02x,\r\n", txByte );	
	TWI1.MADDR = txByte;
	// 3: Espero el ACK/NACK
	while ( ticks_to_wait-- > 0 ) {
		xprintf("i2c: STATUS=0x%02x,\r\n", TWI1.MSTATUS );
		_delay_ms(50);
	}


}
// DRIVER I2C
//------------------------------------------------------------------------------------
int drv_i2c_master_read  ( char *pvBuffer )
{
bool retV = false;
char rxByte = ' ';
uint8_t i = 0;
int xReturn = -1;

uint8_t devAddress;


	// Fuerzo al bus al estado idle.
	if ( ! pvI2C_set_bus_idle() ) 
		goto i2c_quit;

	// Paso 1: Envio START + SLA_address + SLA_W
	devAddress = (I2C_sctl.devAddress << 1 ) +  I2C_DIRECTION_BIT_READ;
	xprintf("drv_i2c_master_read: SLA_0: 0x%02x\r\n\0", devAddress );
	retV = pvI2C_write_slave_devaddress(devAddress);
	if (!retV) 
		goto i2c_quit;

	// Pass4: Leo todos los bytes requeridos y respondo a c/u con ACK.
	for ( i=0; i < (I2C_sctl.dataLength - 1); i++ ) {
		if ( ! pvI2C_read_slave(ACK, &rxByte) ) goto i2c_quit;
		*pvBuffer++ = rxByte;
	}
	xReturn = i;

	// Ultimo byte.
	if ( ! pvI2C_read_slave(NACK, &rxByte) ) {
		xReturn = -1;
		goto i2c_quit;
		} else {
		*pvBuffer++ = rxByte;
		xReturn++;
	}

	// I2C read OK.
	retV = true;

i2c_quit:

	// Pass6) STOP
	//TWIE.MASTER.CTRLC = TWI_MASTER_CMD_STOP_gc;

	// En caso de error libero la interface forzando el bus al estado IDLE
	//if ( !retV )
	//	pvI2C_reset();

	return(xReturn);

}
//------------------------------------------------------------------------------------
// FUNCIONES AUXILIARES PRIVADAS DE I2C
//------------------------------------------------------------------------------------
bool pvI2C_set_bus_idle(void)
{
	// Para comenzar una operacion el bus debe estar en IDLE o OWENED.
	// Intento pasarlo a IDLE hasta 3 veces antes de abortar, esperando 100ms
	// entre c/intento.

uint8_t	reintentos = I2C_MAXTRIES;

	while ( reintentos-- > 0 ) {
		i2c_reset();
		_delay_ms(100);
		
		if ( (TWI1.MSTATUS &  TWI_BUSSTATE_IDLE_gc) != 0 ) {
			xprintf("pvI2C_set_bus_idle: STATUS: 0x%02x\r\n", TWI1.MSTATUS);
			return (true);
		} else {
			_delay_ms(100);
		}
	}
	// No pude pasarlo a IDLE: Error !!!
	xprintf( "pvI2C_set_bus_idle FAIL: STATUS: 0x%02x", TWI1.MSTATUS);
	return(false);
}
//------------------------------------------------------------------------------------
bool pvI2C_write_slave_devaddress(const uint8_t devAddress)
{
	// Pass1: Mando un START y el SLAVE_ADDRESS (SLA_W/R)
	// El start se genera automaticamente al escribir en el reg MADDR.

bool ret_code = false;

	TWI1.MADDR = devAddress;
	if ( ! pvI2C_waitForComplete() ) goto i2c_exit;
	
	xprintf("pvI2C_write_slave_devaddress: TXB=0x%02x, STATUS=0x%02x\r\n\0", devAddress, TWI1.MSTATUS );

	// Primero evaluo no tener errores.
	if ( (TWI1.MSTATUS & TWI_ARBLOST_bm) != 0 ) goto i2c_exit;
	if ( (TWI1.MSTATUS & TWI_BUSERR_bm) != 0 ) goto i2c_exit;

	// ACK o NACK ?
	if  (!(TWI_RXACK_bm & TWI0.MSTATUS)) {
		// ACK
		ret_code = true;
		xprintf("pvI2C_write_slave_devaddress: ACK\r\n" );
		goto i2c_exit;
	} else {
		xprintf("pvI2C_write_slave_devaddress: NACK\r\n" );
		// NACK
		goto i2c_exit;
	} 

i2c_exit:

	if ( !ret_code ) {
		xprintf("pvI2C_write_slave_devaddress: ERROR: 0x%02x,0x%02x\r\n\0",devAddress,TWI1.MSTATUS );
	}

	return(ret_code);
}
//------------------------------------------------------------------------------------
bool pvI2C_read_slave( uint8_t response_flag, char *rxByte )
{

bool ret_code = false;

	// Espero 1 byte enviado por el slave
	if ( ! pvI2C_waitForComplete() ) goto i2c_exit;

	*rxByte = TWI1.MDATA;

	if (response_flag == ACK) {
		TWI1.MCTRLB &= ~(1 << TWI_ACKACT_bp);		// ACK Mas bytes
	} else {
		TWI1.MCTRLB |= TWI_ACKACT_NACK_gc;			// NACK STOP Ultimo byte
		TWI1.MCTRLB |= TWI_MCMD_STOP_gc;
	}

	xprintf("pvI2C_read_slave: rx=0x%02x, status=0x%02x\r\n\0",*rxByte, TWI1.MSTATUS );

	// Primero evaluo no tener errores.
	if ( (TWI1.MSTATUS & TWI_ARBLOST_bm) != 0 ) goto i2c_exit;
	if ( (TWI1.MSTATUS & TWI_BUSERR_bm) != 0 ) goto i2c_exit;

	ret_code = true;

	i2c_exit:

	if ( !ret_code ) {
		xprintf("pvI2C_read_slave: ERROR: 0x%02x(%c),0x%02x\r\n\0",*rxByte,*rxByte,TWI1.MSTATUS );
	}
	return(ret_code);
}
//------------------------------------------------------------------------------------
bool  pvI2C_waitForComplete(void)
{
uint8_t ticks_to_wait = 5;		// 3 ticks ( 30ms es el maximo tiempo que espero )

	// wait for i2c interface to complete write operation ( MASTER WRITE INTERRUPT )
	while ( ticks_to_wait-- > 0 ) {
		if ( ( (TWI1.MSTATUS & TWI_WIF_bm) != 0 ) || ( (TWI1.MSTATUS & TWI_RIF_bm) != 0 ) ) {
			xprintf( "pvI2C_waitForComplete: true: 0x%02x\r\n\0",TWI1.MSTATUS );
			return(true);
		}
		_delay_ms(10);
	}

	xprintf( "pvI2C_waitForComplete: false: 0x%02x\r\n\0",TWI1.MSTATUS );
	return(false);	
}
//------------------------------------------------------------------------------------
