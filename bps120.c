/*
 * bps120.c
 *
 * Created: 23/10/2020 15:50:13
 *  Author: Pablo
 */ 

#include "./include_app/conversor_i2c_420ma.h"
#include "./i2c/atmel_start_pins.h"

#define BUSADDR_BPS120					0x28
#define I2C_DIRECTION_BIT_WRITE            0
#define I2C_DIRECTION_BIT_READ             1

#define I2C_OK			0
#define I2C_RD_ERROR	1
#define I2C_WR_ERROR	2

#define I2C_REINTENTOS_STATUS	5

typedef enum { I2C_ARBLOST=0, I2C_BUSERR, I2C_ACK, I2C_NACK, I2C_TO  } t_i2c_status_code;
	
typedef struct {
	uint8_t devAddress;
	uint16_t dataStartAddress;
	uint8_t dataLength;
	uint8_t i2c_error_code;
} periferico_i2c_port_t;

periferico_i2c_port_t I2C_sctl;

int8_t bps120_raw_read( char *data );
int8_t i2c_read( uint8_t i2c_dev_address, uint16_t i2c_rdStartAddress, uint8_t dataLength, char *rdBuffer );
int i2c_master_read  ( char *pvBuffer );

void i2c_reset(void);
bool i2c_wait_bus_idle(void);
void i2c_write_slave_devaddress(const uint8_t devAddress);
bool i2c_waitForComplete(void);
t_i2c_status_code i2c_check_status(void);
void i2c_receive_byte(bool ackFlag);
uint8_t i2c_get_received_byte(void);

#define I2C_DIRECTION_BIT_WRITE                         0
#define I2C_DIRECTION_BIT_READ                          1
#define I2C_SET_ADDR_POSITION(address)                  (address << 1)
#define I2C_SLAVE_RESPONSE_ACKED                        (!(TWI_RXACK_bm & TWI1.MSTATUS))
#define I2C_DATA_RECEIVED                               (TWI_RIF_bm & TWI1.MSTATUS)

static void I2C_0_transmittingAddrPacket(uint8_t slaveAddres, uint8_t directionBit);
static uint8_t I2C_0_receivingDataPacket(void);
static void I2C_0_sendMasterCommand(uint8_t newCommand);
static void I2C_0_setACKAction(void);
static void I2C_0_setNACKAction(void);

void I2C0_test(char *rdBuffer);

void i2c_tx_slave_devaddress(uint8_t slaveAddres, uint8_t directionBit);
bool i2c_await_ack(void);
bool i2c_rx_slave_byte( char *rxChar);
void i2c_setACKAction(void);
void i2c_setNACKAction(void);
void i2c_sendMasterCommand(uint8_t newCommand);

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

	rcode =  i2c_read( BUSADDR_BPS120, 0x00, 0x02, data );
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
int8_t i2c_read_( uint8_t i2c_dev_address, uint16_t i2c_rdStartAddress, uint8_t dataLength, char *rdBuffer )
{

	size_t xReturn = 0U;
	uint8_t i2c_error_code;

	I2C_sctl.devAddress = i2c_dev_address;
	I2C_sctl.dataStartAddress = i2c_rdStartAddress;
	I2C_sctl.dataLength = dataLength;
	I2C_sctl.i2c_error_code = I2C_OK;

	I2C0_test(rdBuffer);
	return(2);
	
	xReturn = i2c_master_read( rdBuffer );

	// Controlamos errores.
	i2c_error_code = I2C_sctl.i2c_error_code;
	if (i2c_error_code != I2C_OK ) {
		xprintf("i2c_read ERROR: err_code=0x0%X.\r\n\0", i2c_error_code );
		return(-1);
	}

	if (xReturn !=  dataLength ) {
		xprintf("i2c_read ERROR: xbytes=%d, xReturn=%d.\r\n\0", dataLength, xReturn  );
		return(-1);
	}

	return(xReturn);

}
//------------------------------------------------------------------------------------
int8_t i2c_read( uint8_t i2c_dev_address, uint16_t i2c_rdStartAddress, uint8_t dataLength, char *rdBuffer )
{

uint8_t length = dataLength;
uint8_t rxChar;

	// Paso 1: Confirmo que el bus este idle
	if ( !  i2c_wait_bus_idle() ) {
		xprintf("i2c_read NOT IDLE.\r\n");
		return(-1);
	}

	// Paso 2: Envio START + SLA_address + SLA_W
	i2c_tx_slave_devaddress(i2c_dev_address, I2C_DIRECTION_BIT_READ);
	if ( ! i2c_await_ack()) {
		xprintf("i2c_read NOT ACK.\r\n");
		return(-1);		
	}
	
	// Paso 3: Leo el resto de los bytes.
	while( length > 1) {
		i2c_rx_slave_byte(&rxChar);
		*rdBuffer++ = rxChar;
		xprintf("i2c_read a: status=0x%02x\r\n", TWI1.MSTATUS);
		xprintf("i2c_read b: rxchar=0x%02x\r\n", rxChar);
	
		i2c_setACKAction();
		xprintf("i2c_read c: status=0x%02x\r\n", TWI1.MSTATUS);
		i2c_sendMasterCommand(TWI_MCMD_RECVTRANS_gc);
		xprintf("i2c_read d: status=0x%02x\r\n", TWI1.MSTATUS);	
	}
	
	// Paso 4: Ultimo byte: NACK,STOP
	i2c_rx_slave_byte(&rxChar);
	*rdBuffer++ = rxChar;
	xprintf("i2c_read e: status=0x%02x\r\n", TWI1.MSTATUS);
	xprintf("i2c_read f: rxchar=0x%02x\r\n", rxChar);
		
	i2c_setNACKAction();
	xprintf("i2c_read g: status=0x%02x\r\n", TWI1.MSTATUS);
	i2c_sendMasterCommand(TWI_MCMD_STOP_gc);
	xprintf("i2c_read h: status=0x%02x\r\n", TWI1.MSTATUS);

}
//------------------------------------------------------------------------------------
int i2c_master_read( char *pvBuffer )
{
	// Lee de un dispositivo i2c del bus en modo MASTER con poleo.
	
int xReturn = 0;
uint8_t devAddress;
uint8_t length;
uint8_t rxChar;
	
	// Confirmo que el bus este idle
	if ( !  i2c_wait_bus_idle() ) {
		xprintf("i2c_master_read NOT IDLE.\r\n");
		return(-1);
	}
	
	// Paso 1: Envio START + SLA_address + SLA_W
	devAddress = (I2C_sctl.devAddress << 1 ) +  I2C_DIRECTION_BIT_READ;
	xprintf("i2c_master_read: SLA_0: 0x%02x\r\n\0", devAddress );
	i2c_write_slave_devaddress(devAddress);
	if (  ! i2c_waitForComplete() ) {
		xprintf("i2c_master_read TIMEOUT\r\n");
		return(-1);
	}
	if ( i2c_check_status() != I2C_ACK ) {
		xprintf("i2c_master_read SLA not acked.\r\n");
		return(-1);		
	}
	xprintf("i2c_master_read: ACK rcvd\r\n");
	
	// Pass2: Leo todos los bytes requeridos y respondo a c/u con ACK y al ultimo con NACK.
	length = I2C_sctl.dataLength;
	xprintf("i2c_master_read: reading %d bytes\r\n", length);
	
	while ( length > 1 ) {
		i2c_receive_byte(true);
		if (  ! i2c_waitForComplete() ) {
			xprintf("i2c_master_read TIMEOUT\r\n");
			return(-1);
		}
		rxChar = i2c_get_received_byte();
		*pvBuffer++ = rxChar;
		xReturn++;
		// decrement length
		length--;
		xprintf("i2c_master_read: RXCHAR=0x%02x, STATUS=0x%02x, length=%d\r\n",rxChar,TWI1.MSTATUS, length);
	}
	
	xprintf("i2c_master_read: Last byte(nack)\r\n");
	// Last byte. NACK
	i2c_receive_byte(false);
	if (  ! i2c_waitForComplete() ) {
		xprintf("i2c_master_read TIMEOUT\r\n");
		return(-1);
	}
	*pvBuffer++ = i2c_get_received_byte();
	xReturn++;


	return(xReturn);

}
//------------------------------------------------------------------------------------
void i2c_init(void)
{
	xprintf("i2c_init\r\n");
	
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

	// set i2c bit rate to 100KHz
	TWI1.MBAUD = (uint8_t)TWI1_BAUD(100000, 0);
	
	// enable TWI (two-wire interface)	
	TWI1.MCTRLA = 1 << TWI_ENABLE_bp        /* Enable TWI Master: enabled */
	| 0 << TWI_QCEN_bp						/* Quick Command Enable: disabled */
	| 0 << TWI_RIEN_bp						/* Read Interrupt Enable: disabled */
	| 0 << TWI_SMEN_bp						/* Smart Mode Enable: disabled */
	| TWI_TIMEOUT_DISABLED_gc				/* Bus Timeout Disabled */
	| 0 << TWI_WIEN_bp;						/* Write Interrupt Enable: disabled */

	i2c_reset();

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
bool i2c_wait_bus_idle(void)
{
	// Para comenzar una operacion el bus debe estar en IDLE o OWENED.
	// Intento pasarlo a IDLE hasta 3 veces antes de abortar, esperando 100ms
	// entre c/intento.

uint8_t	reintentos = 3;

	while ( reintentos-- > 0 ) {
		i2c_reset();
		_delay_ms(100);
		
		if ( (TWI1.MSTATUS &  TWI_BUSSTATE_IDLE_gc) != 0 ) {
			xprintf("i2c_set_bus_idle: STATUS=0x%02x\r\n", TWI1.MSTATUS);
			return (true);
		} else {
			_delay_ms(100);
		}
	}
	
	// No pude pasarlo a IDLE: Error !!!
	xprintf( "i2c_set_bus_idle FAIL: STATUS=0x%02x", TWI1.MSTATUS);
	return(false);
}
//------------------------------------------------------------------------------------
void i2c_write_slave_devaddress(const uint8_t devAddress)
{
	// Pass1: Mando un START y el SLAVE_ADDRESS (SLA_W/R)
	// El START se genera automaticamente al escribir en el reg MADDR.


	xprintf("i2c_write_slave_devaddress\r\n");
	// Escribo la direccion del slave ( START + SLAVE_ADDR + R/W )
	TWI1.MADDR = devAddress;

}
//------------------------------------------------------------------------------------
t_i2c_status_code i2c_check_status(void)
{

t_i2c_status_code ret_code;
uint8_t reintentos = I2C_REINTENTOS_STATUS;

	xprintf("i2c_check_status\r\n");
	
	// Espero la respuesta: ACK/NACK/TIMEOUT/ERROR
	while ( reintentos-- > 0 ) {
		_delay_ms(100);
		// ERRORS.
		if ( (TWI1.MSTATUS & TWI_ARBLOST_bm) != 0 ) {
			ret_code = I2C_ARBLOST;
			goto i2c_exit;
		}
		if ( (TWI1.MSTATUS & TWI_BUSERR_bm) != 0 ) {
			ret_code = I2C_BUSERR;
			goto i2c_exit;
		}
		// ACK o NACK ?
		if  ( (TWI1.MSTATUS & TWI_RXACK_bm) == 0 ) {
			// ACK
			ret_code = I2C_ACK;
			goto i2c_exit;
		} else {
			ret_code = I2C_NACK;
			// NACK
			goto i2c_exit;
		}
	}
	
	ret_code = I2C_TO;
	
i2c_exit:

	xprintf("i2c_check_status: STATUS=0x%02x, reintentos=%d, ret_code=%d\r\n\0",TWI1.MSTATUS, reintentos, ret_code );
	return(ret_code);
}
//------------------------------------------------------------------------------------
bool i2c_waitForComplete(void)
{
	// Espera que termine la operacion TWI en curso
	
uint8_t timeout = 10;		

	// wait for i2c interface to complete write operation ( MASTER WRITE INTERRUPT )
	while ( timeout-- > 0 ) {
		xprintf( "i2c_waitForComplete: STATUS 0x%02x\r\n\0",TWI1.MSTATUS );
		if ( ( (TWI1.MSTATUS & TWI_WIF_bm) != 0 ) || ( (TWI1.MSTATUS & TWI_RIF_bm) != 0 ) ) {
			xprintf( "i2c_waitForComplete OK: STATUS=0x%02x\r\n\0",TWI1.MSTATUS );
			return(true);
		}
		_delay_ms(10);
	}

	xprintf( "i2c_waitForComplete FAIL: STATUS 0x%02x\r\n\0",TWI1.MSTATUS );
	return(false);	
}
//------------------------------------------------------------------------------------
void i2c_receive_byte(bool ackFlag)
{
	// begin receive over i2c
	if( ackFlag ) {
		// ackFlag = TRUE: ACK the recevied data
		TWI1.MCTRLB &= !TWI_ACKACT_bm;
	} else {
		// ackFlag = FALSE: NACK the recevied data
		TWI1.MCTRLB |= TWI_ACKACT_bm;
	}	
}
//------------------------------------------------------------------------------------
uint8_t i2c_get_received_byte(void)
{
	// retieve received data byte from i2c TWDR
	return( TWI1.MDATA );
}
//------------------------------------------------------------------------------------

void I2C0_test(char *rdBuffer)
{
	
uint8_t rxChar;

	I2C_0_transmittingAddrPacket(BUSADDR_BPS120, I2C_DIRECTION_BIT_READ);
	xprintf("I2C0_test 1: status=0x%02x\r\n", TWI1.MSTATUS);
	
	rxChar = I2C_0_receivingDataPacket();
	*rdBuffer++ = rxChar;
	xprintf("I2C0_test 2: status=0x%02x\r\n", TWI1.MSTATUS);
	xprintf("I2C0_test 4: rxchar=0x%02x\r\n", rxChar);
	
	I2C_0_setACKAction();
	xprintf("I2C0_test 4: status=0x%02x\r\n", TWI1.MSTATUS);
	
	I2C_0_sendMasterCommand(TWI_MCMD_RECVTRANS_gc);
	xprintf("I2C0_test 5: status=0x%02x\r\n", TWI1.MSTATUS);
	
	rxChar = I2C_0_receivingDataPacket();
	*rdBuffer++ = rxChar;
	xprintf("I2C0_test 6: status=0x%02x\r\n", TWI1.MSTATUS);
	xprintf("I2C0_test 7: rxchar=0x%02x\r\n", rxChar);
	
	I2C_0_setNACKAction();
	xprintf("I2C0_test 8: status=0x%02x\r\n", TWI1.MSTATUS);
	
	I2C_0_sendMasterCommand(TWI_MCMD_STOP_gc);
	xprintf("I2C0_test 9: status=0x%02x\r\n", TWI1.MSTATUS);
}

//------------------------------------------------------------------------------------
static void I2C_0_transmittingAddrPacket(uint8_t slaveAddres, uint8_t directionBit)
{
	TWI1.MADDR = I2C_SET_ADDR_POSITION(slaveAddres) + directionBit;
	while (!I2C_SLAVE_RESPONSE_ACKED)
	{
		;
	}
}

static uint8_t I2C_0_receivingDataPacket(void)
{
	while (!I2C_DATA_RECEIVED)
	{
		;
	}

	return TWI1.MDATA;
}

static void I2C_0_sendMasterCommand(uint8_t newCommand)
{
	TWI1.MCTRLB |=  newCommand;
}

static void I2C_0_setACKAction(void)
{
	TWI1.MCTRLB &= !TWI_ACKACT_bm;
}

static void I2C_0_setNACKAction(void)
{
	TWI1.MCTRLB |= TWI_ACKACT_bm;
}

//------------------------------------------------------------------------------------
void i2c_tx_slave_devaddress(uint8_t slaveAddres, uint8_t directionBit)
{
	// Pass1: Mando un START y el SLAVE_ADDRESS (SLA_W/R)
	// El START se genera automaticamente al escribir en el reg MADDR.

uint8_t txAddress;

	txAddress = (slaveAddres << 1) + directionBit;
	TWI1.MADDR = txAddress;
	xprintf("i2c_tx_slave_devaddress: ADDR=0x%02x\r\n",txAddress);

}
//------------------------------------------------------------------------------------
bool i2c_await_ack(void)
{
uint8_t tryes = 10;

	while(tryes-- > 0) {
		if ((TWI1.MSTATUS & TWI_RXACK_bm) != 0) {
			xprintf("i2c_await_ack ACK: STATUS=0x%02x\r\n",TWI1.MSTATUS);
			return(true);
		}
	}
	xprintf("i2c_await_ack TO: STATUS=0x%02x\r\n",TWI1.MSTATUS);
	return(false);
}
//------------------------------------------------------------------------------------
bool i2c_rx_slave_byte( char *rxChar)
{
	// Espera con timeout recibir un byte del slave
	
uint8_t timeout = 5;

	while(timeout-- > 0) {
		if ( (TWI_RIF_bm & TWI1.MSTATUS) != 0 ) {
			*rxChar = TWI1.MDATA;
			xprintf("i2c_rx_slave_byte: RXBYTE=0x%02x\r\n", *rxChar );
			return(true);
		}
	}
	xprintf("i2c_rx_slave_byte: TO. STATUS=0x%02x\r\n", TWI1.MSTATUS );
	return(false);
}
//------------------------------------------------------------------------------------
void i2c_setACKAction(void)
{
	TWI1.MCTRLB &= !TWI_ACKACT_bm;
}
//------------------------------------------------------------------------------------
void i2c_setNACKAction(void)
{
	TWI1.MCTRLB |= TWI_ACKACT_bm;
}
//------------------------------------------------------------------------------------
void i2c_sendMasterCommand(uint8_t newCommand)
{
	TWI1.MCTRLB |=  newCommand;
}
//------------------------------------------------------------------------------------

