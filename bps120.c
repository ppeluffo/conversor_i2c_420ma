/*
 * bps120.c
 *
 * Created: 23/10/2020 15:50:13
 *  Author: Pablo
 */ 

#include "./include_app/conversor_i2c_420ma.h"

#define BUSADDR_BPS120 0x28

#define ioctl_I2C_SET_DEVICE_ADDRESS		1
#define ioctl_I2C_SET_BYTE_ADDRESS			2
#define ioctl_I2C_SET_BYTE_ADDRESS_LENGTH	3
#define ioctl_I2C_GET_LAST_ERROR		4
#define ioctl_I2C_SCAN					5
#define ioctl_SET_TIMEOUT				6

#define TWI_MASTER_ARBLOST_bm	0x08
#define TWI_MASTER_BUSERR_bm	0x04
#define TWI_MASTER_RXACK_bm		0x0F
#define TWI_MASTER_WIF_bm		0x01
#define TWI_MASTER_RIF_bm		0x02

#define I2C_OK			0
#define I2C_RD_ERROR	1
#define I2C_WR_ERROR	2

typedef struct {
	uint8_t devAddress;
	uint16_t byteAddress;
	uint8_t byteAddressLength;
	uint8_t i2c_error_code;
} periferico_i2c_port_t;

periferico_i2c_port_t xBusI2C;

#define SCL		0
#define SDA		1
#define I2C_MAXTRIES	5

#define ACK 	0
#define NACK 	1

int8_t bps120_raw_read( char *data );
int8_t I2C_read( uint8_t i2c_dev_address, uint32_t i2c_rdAddress, char *rdBuffer, uint8_t length );
int os_i2c_ioctl( uint32_t ulRequest, void *pvValue );
int os_i2c_read ( char *pvBuffer, uint16_t xBytes );

int drv_I2C_master_read  ( const uint8_t devAddress, const uint8_t devAddressLength, const uint16_t byteAddress, char *pvBuffer, size_t xBytes );

bool pvI2C_set_bus_idle(void);
bool pvI2C_write_slave_address(const uint8_t devAddress);
bool pvI2C_write_slave_reg( const uint8_t devAddressLength, const uint16_t byteAddress );
bool pvI2C_write_data( const char txbyte);
bool pvI2C_read_slave( uint8_t response_flag, char *rxByte );
void pvI2C_reset(void);
bool  pvI2C_waitForComplete(void);

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
		//*presion = -100;
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
uint8_t times = 3;

	while ( times-- > 0 ) {

		// Leo 2 bytes del sensor de presion.
		// device_address: BUSADDR_BPS120
		// read_address: 0x00
		// read_buffer: data
		// nro_bytes_to_read: 0x02
		rcode =  I2C_read( BUSADDR_BPS120, 0, data, 0x02 );

		if ( rcode == -1 ) {
			// Hubo error: trato de reparar el bus y reintentar la operacion
			// Espero 1s que se termine la fuente de ruido.
			_delay_ms(1000);
			// Reconfiguro los dispositivos I2C del bus que pueden haberse afectado
			xprintf("ERROR: BPS120_raw_read recovering i2c bus (%d)\r\n\0", times );
	
		} else {
			// No hubo error: salgo normalmente
			break;
		}
	}
	return( rcode );
}
//----------------------------------------------------------------
int8_t I2C_read( uint8_t i2c_dev_address, uint32_t i2c_rdAddress, char *rdBuffer, uint8_t length )
{
	// Funcion de libreria para acceder al bus I2C en modo generico.
	// utiliza los servicios de os_i2c para setear los parametros de la operacion y luego
	// acceder a los datos.
	// Lee en la EE, desde la posicion 'address', 'length' bytes
	// y los deja en el buffer apuntado por 'data'

size_t xReturn = 0U;
uint8_t dev_address = 0;
uint16_t rdAddress = 0;
uint8_t	rdAddress_length = 1;
int8_t xBytes = 0;
uint8_t i2c_error_code = 0;


	xprintf( "DEBUG: I2C_READ: BADDR=0x%02x ,RDADDR=0x%02x, LEN=0x%02x \r\n\0", i2c_dev_address, i2c_rdAddress, length );

	// 1) Indicamos el periferico i2c en el cual queremos leer ( variable de 8 bits !!! )
	dev_address = i2c_dev_address;
	os_i2c_ioctl(ioctl_I2C_SET_DEVICE_ADDRESS, &dev_address);

	// 2) Luego indicamos la direccion desde donde leer:
	// 	Direccion: El FRTOS espera siempre 2 bytes.
	//  Hago un cast para dejarla en 16 bits.
	rdAddress = (uint16_t)(i2c_rdAddress);
	os_i2c_ioctl(ioctl_I2C_SET_BYTE_ADDRESS,&rdAddress);
	//    Largo: 1 byte indica el largo. El FRTOS espera 1 byte.
	rdAddress_length = 1;	//
	os_i2c_ioctl(ioctl_I2C_SET_BYTE_ADDRESS_LENGTH, &rdAddress_length);
	
	//  3) Por ultimo leemos length bytes y los dejamos en data. No controlo fronteras.
	xBytes = length;
	xReturn = os_i2c_read( rdBuffer, xBytes);

	//	memset(buffer,'\0', 10);
	//	strcpy_P(buffer, (PGM_P)pgm_read_word(&(I2C_names[pv_i2_addr_2_idx( i2c_bus_address )])));
	//	xprintf_P(PSTR("I2C RD 0x0%X, %s.\r\n\0"), i2c_bus_address, buffer );

	// Controlamos errores.
	i2c_error_code = os_i2c_ioctl( ioctl_I2C_GET_LAST_ERROR, NULL );
	if (i2c_error_code != I2C_OK ) {
		xprintf("ERROR: I2C_RD err 0x0%X, err_code= 0x0%X.\r\n\0", dev_address, i2c_error_code );
	}

	if (xReturn != xBytes ) {
		xprintf("ERROR: I2C_RD err 0x0%X, xbytes=%d, xReturn=%d.\r\n\0", dev_address, xBytes, xReturn  );
		xReturn = -1;
	}

	return(xReturn);

}
//------------------------------------------------------------------------------------
int os_i2c_ioctl( uint32_t ulRequest, void *pvValue )
{
	// Funcion del so. para realizar funciones de seteo de valores y preparar las
	// estructuras para operaciones de los drivers.
	
int xReturn = 0;
uint16_t *p = NULL;

	p = pvValue;

	xprintf("OS_I2C_IOCTL: CODE[0x%02x] VAL[0x%02x]\r\n\0" ,(uint8_t)ulRequest, (uint8_t)(*p));

	switch( ulRequest ) {;
		case ioctl_I2C_SET_DEVICE_ADDRESS:
			xBusI2C.devAddress = (int8_t)(*p);
			break;
		case ioctl_I2C_SET_BYTE_ADDRESS:
			xBusI2C.byteAddress = (uint16_t)(*p);
			break;
		case ioctl_I2C_SET_BYTE_ADDRESS_LENGTH:
			xBusI2C.byteAddressLength = (int8_t)(*p);
			break;
		case ioctl_I2C_GET_LAST_ERROR:
			xReturn = xBusI2C.i2c_error_code;;
			break;
		default :
			xReturn = -1;
			break;
	}

	return (xReturn);

}
//------------------------------------------------------------------------------------
int os_i2c_open( void )
{
	xBusI2C.i2c_error_code = I2C_OK;
	return(1);
}
//------------------------------------------------------------------------------------
int os_i2c_read(  char *pvBuffer, uint16_t xBytes )
{
	// Funcion del so. para leer del bus I2C.
	// Da una capa de abstraccion al programa.
	// Utiliza los servicios del driver.
	
	int xReturn = 0U;

	xprintf("OS_I2C_RD: devAddr:0x%02x,addrLen:0x%02x,byteAddr:0x%02x,xbytes: 0x%02x\r\n\0",xBusI2C.devAddress, xBusI2C.byteAddressLength,xBusI2C.byteAddress, xBytes);

	if ( ( xReturn = drv_I2C_master_read(xBusI2C.devAddress, xBusI2C.byteAddressLength, xBusI2C.byteAddress, (char *)pvBuffer, xBytes)) > 0 ) {
		xBusI2C.i2c_error_code = I2C_OK;
		} else {
		// Error de lectura indicado por el driver.
		xBusI2C.i2c_error_code = I2C_RD_ERROR;
	}

	return(xReturn);
}
//------------------------------------------------------------------------------------
// DRIVER I2C
//------------------------------------------------------------------------------------
int drv_I2C_master_read  ( const uint8_t devAddress, const uint8_t devAddressLength, const uint16_t byteAddress, char *pvBuffer, size_t xBytes )
{
	

bool retV = false;
char rxByte = ' ';
uint8_t i = 0;
int xReturn = -1;

	
	xprintf("DEBUG: drv_i2c_MR: 0x%02x,0x%02x,0x%02x,0x%02x\r\n\0", devAddress, devAddressLength, (uint16_t)(byteAddress), xBytes );

	if ( xBytes < 1 ) return(false);

	// Fuerzo al bus al estado idle.
	if ( ! pvI2C_set_bus_idle() ) goto i2c_quit;

	if ( devAddressLength > 0 ) {

		xprintf("DEBUG: drv_i2c_MR: SLA_0: 0x%02x,\r\n\0", devAddress & ~0x01 );

		// Pass1: Mando un START y el SLAVE_ADDRESS (SLA_W)
		retV = pvI2C_write_slave_address(devAddress & ~0x01);
		if (!retV) goto i2c_quit;

		// Pass2: Mando la direccion interna del slave desde donde voy a leer.
		if ( ! pvI2C_write_slave_reg( devAddressLength, byteAddress ) ) goto i2c_quit;
	}

	// Pass3: Mando un REPEATED START y el SLAVE_ADDRESS (SLA_R)
	// Lo mando una sola vez ya que estoy en medio del ciclo.
	xprintf("DEBUG: drv_i2c_MR: SLA_1: 0x%02x,\r\n\0",devAddress | 0x01 );

	if ( ! pvI2C_write_slave_address(devAddress | 0x01) ) goto i2c_quit;

	// Pass4: Leo todos los bytes requeridos y respondo a c/u con ACK.
	for ( i=0; i < (xBytes-1); i++ ) {
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
	if ( !retV )
		pvI2C_reset();

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

	uint8_t bus_status = 0;
	uint8_t	reintentos = I2C_MAXTRIES;

	while ( reintentos-- > 0 ) {

		TWI1.MCTRLB |= TWI_FLUSH_bm;
		TWI1.MSTATUS |= TWI_BUSSTATE_IDLE_gc;
		// Reset module
		TWI1.MSTATUS |= (TWI_RIF_bm | TWI_WIF_bm);

		bus_status = TWI1.MSTATUS & 0x03; 	// Me quedo con los bits de BUSSTATE;

		xprintf("drv_i2c: I2C_BUSIDLE(%d): 0x%02x\r\n\0",reintentos, TWI1.MSTATUS );

		if (  bus_status == TWI_BUSSTATE_IDLE_gc ) {
			return(true);
			_delay_ms(100);
		}
	}

	// No pude pasarlo a IDLE: Error !!!
	xprintf( "drv_i2c: I2C_BUSIDLE ERROR!!: 0x%02x\r\n\0",TWI1.MSTATUS );
	return(false);
}
//------------------------------------------------------------------------------------
bool pvI2C_write_slave_address(const uint8_t devAddress)
{
	// Pass1: Mando un START y el SLAVE_ADDRESS (SLA_W/R)
	// El start se genera automaticamente al escribir en el reg MASTER.ADDR.
	// Esto tambien resetea todas las flags.
	// La salida correcta es con STATUS = 0x62.
	// El escribir el ADDR borra todas las flags.

	char txbyte = devAddress;	// (SLA_W/R) Send slave address
	bool ret_code = false;
	uint8_t currentStatus = 0;

	xprintf("drv_i2c: I2C_SLA: DEV_ADDR=0x%02x,\r\n\0",devAddress );

	//TWI1.MADDR = devAddress << 1 | 1;
	TWI1.MADDR = txbyte;

	if ( ! pvI2C_waitForComplete() ) goto i2c_exit;
	
	currentStatus = TWI1.MSTATUS;

	xprintf("drv_i2c: I2C_SLA: 0x%02x,0x%02x\r\n\0",txbyte, currentStatus );

	// Primero evaluo no tener errores.
	if ( (currentStatus & TWI_MASTER_ARBLOST_bm) != 0 ) goto i2c_exit;
	if ( (currentStatus & TWI_MASTER_BUSERR_bm) != 0 ) goto i2c_exit;

	// ACK o NACK ?
	if ( (currentStatus & TWI_MASTER_RXACK_bm) != 0 ) {
		// NACK
		goto i2c_exit;
	} else {
		// ACK
		ret_code = true;
		goto i2c_exit;
	}

	i2c_exit:

	if ( !ret_code ) {
		xprintf("drv_i2c: I2C_ADDR_ERROR: 0x%02x,0x%02x\r\n\0",txbyte,currentStatus );
	}

	return(ret_code);
}
//------------------------------------------------------------------------------------
bool pvI2C_write_slave_reg( const uint8_t devAddressLength, const uint16_t byteAddress )
{
	// Mando la direccion interna del slave donde voy a escribir.
	// Pueden ser 0, 1 o 2 bytes.
	// En las memorias es una direccion de 2 bytes.En el DS1344 o el MCP es de 1 byte
	// En los ADC suele ser 0.
	// Envio primero el High 8 bit i2c address.
	// Luego envio el Low 8 byte i2c address.

	char txbyte = ' ';
	bool ret_code = false;
	uint8_t currentStatus = 0;

	// HIGH address
	if ( devAddressLength == 2 ) {
		txbyte = (byteAddress) >> 8;
		TWI1.MADDR = txbyte;		// send byte
		if ( ! pvI2C_waitForComplete() )  goto i2c_exit;
		currentStatus = TWI1.MSTATUS;

		xprintf("drv_i2c: I2C_SR_H: 0x%02x,0x%02x\r\n\0",txbyte,currentStatus );
		// ACK o NACK ?
		if ( (currentStatus & TWI_MASTER_RXACK_bm) != 0 )  goto i2c_exit;
	}

	// LOW address
	if ( devAddressLength >= 1 ) {
		txbyte = (byteAddress) & 0x00FF;
		TWI1.MADDR = txbyte;		// send byte
		if ( ! pvI2C_waitForComplete() )  goto i2c_exit;
		currentStatus = TWI1.MSTATUS;

		xprintf("drv_i2c: I2C_SL_L: 0x%02x,0x%02x\r\n\0",txbyte,currentStatus );
		
		if ( (currentStatus & TWI_MASTER_RXACK_bm) != 0 )  goto i2c_exit;
	}

	ret_code = true;

	i2c_exit:
	if ( !ret_code ) {
		xprintf("drv_i2c: I2C_REG_ERROR: 0x%02x,0x%02x\r\n\0",txbyte,currentStatus );
	}
	
	return(ret_code);
}
//------------------------------------------------------------------------------------
bool pvI2C_read_slave( uint8_t response_flag, char *rxByte )
{

	bool ret_code = false;
	uint8_t currentStatus = 0;

	// Espero 1 byte enviado por el slave
	if ( ! pvI2C_waitForComplete() ) goto i2c_exit;
	currentStatus = TWI1.MSTATUS;

	*rxByte = TWI1.MDATA;

	if (response_flag == ACK) {
		TWI1.MCTRLB &= ~(1 << TWI_ACKACT_bp);		// ACK Mas bytes
	} else {
		TWI1.MCTRLB |= TWI_ACKACT_NACK_gc;			// NACK STOP Ultimo byte
		TWI1.MCTRLB |= TWI_MCMD_STOP_gc;
	}

	xprintf("drv_i2c: I2C_SLR: 0x%02x(%c),0x%02x\r\n\0",*rxByte,*rxByte,currentStatus );

	// Primero evaluo no tener errores.
	if ( (currentStatus & TWI_MASTER_ARBLOST_bm) != 0 ) goto i2c_exit;
	if ( (currentStatus & TWI_MASTER_BUSERR_bm) != 0 ) goto i2c_exit;

	ret_code = true;

	i2c_exit:

	if ( !ret_code ) {
		xprintf("drv_i2c: I2C_SLR_ERROR: 0x%02x(%c),0x%02x\r\n\0",*rxByte,*rxByte,currentStatus );
	}
	return(ret_code);
}
//------------------------------------------------------------------------------------
void pvI2C_reset(void)
{

}
//------------------------------------------------------------------------------------
bool  pvI2C_waitForComplete(void)
{
uint8_t ticks_to_wait = 5;		// 3 ticks ( 30ms es el maximo tiempo que espero )

	// wait for i2c interface to complete write operation ( MASTER WRITE INTERRUPT )
	while ( ticks_to_wait-- > 0 ) {
		if ( ( (TWI1.MSTATUS & TWI_MASTER_WIF_bm) != 0 ) || ( (TWI1.MSTATUS & TWI_MASTER_RIF_bm) != 0 ) ) {
			return(true);
		}
		_delay_ms(10);
	}

	xprintf( "drv_i2c: WFC: TO 0x%02x\r\n\0",TWI1.MSTATUS );
	return(false);	
}
//------------------------------------------------------------------------------------
