/*
 * i2c.c
 *
 * Created: 29/10/2020 8:16:51
 *  Author: Pablo
 */ 

#include "./include_app/conversor_i2c_420ma.h"
#include "./include_os/atmel_start_pins.h"

#define I2C_DIRECTION_BIT_WRITE                         0
#define I2C_DIRECTION_BIT_READ                          1
#define I2C_SET_ADDR_POSITION(address)                  (address << 1)
#define I2C_SLAVE_RESPONSE_ACKED                        (!(TWI_RXACK_bm & TWI1.MSTATUS))
#define I2C_DATA_RECEIVED                               (TWI_RIF_bm & TWI1.MSTATUS)

void i2c_tx_slave_devaddress(bool debug, uint8_t slaveAddres, uint8_t directionBit);
bool i2c_await_ack( bool debug);
bool i2c_rx_slave_byte( bool debug, char *rxChar);
void i2c_setACKAction(void);
void i2c_setNACKAction(void);
void i2c_sendMasterCommand(uint8_t newCommand);

//------------------------------------------------------------------------------------
int8_t I2C_read( bool debug, uint8_t i2c_dev_address, uint16_t i2c_rdStartAddress, uint8_t dataLength, char *rdBuffer )
{

	uint8_t length = dataLength;
	char rxChar;

	// Paso 2: Envio START + SLA_address + SLA_W
	i2c_tx_slave_devaddress(debug, i2c_dev_address, I2C_DIRECTION_BIT_READ);
	if ( ! i2c_await_ack(debug)) {
		if(debug)
			xprintf("i2c_read NOT ACK.\r\n");
		return(-1);
	}
	
	// Paso 3: Leo el resto de los bytes.
	while( length-- > 1) {
		i2c_rx_slave_byte(debug, &rxChar);
		*rdBuffer++ = rxChar;
		if (debug ) {
			xprintf("i2c_read a: status=0x%02x\r\n", TWI1.MSTATUS);
			xprintf("i2c_read b: rxchar=0x%02x\r\n", rxChar);
		}
		i2c_setACKAction();
		if (debug)
			xprintf("i2c_read c: status=0x%02x\r\n", TWI1.MSTATUS);
		i2c_sendMasterCommand(TWI_MCMD_RECVTRANS_gc);
		if(debug)
			xprintf("i2c_read d: status=0x%02x\r\n", TWI1.MSTATUS);
	}
	
	// Paso 4: Ultimo byte: NACK,STOP
	i2c_rx_slave_byte(debug, &rxChar);
	*rdBuffer++ = rxChar;
	if(debug) {
		xprintf("i2c_read e: status=0x%02x\r\n", TWI1.MSTATUS);
		xprintf("i2c_read f: rxchar=0x%02x\r\n", rxChar);
	}
	i2c_setNACKAction();
	if(debug)
		xprintf("i2c_read g: status=0x%02x\r\n", TWI1.MSTATUS);
	i2c_sendMasterCommand(TWI_MCMD_STOP_gc);
	if(debug)
		xprintf("i2c_read h: status=0x%02x\r\n", TWI1.MSTATUS);
	return(dataLength);
	
}
//------------------------------------------------------------------------------------
void I2C_init(void)
{
	//xprintf("i2c_init\r\n");
	
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

	//I2C_reset(true);

}
//------------------------------------------------------------------------------------
void I2C_reset(bool debug)
{
	if (debug)
		xprintf("i2c_reset\r\n");
	TWI1.MCTRLB |= TWI_FLUSH_bm;
	TWI1.MSTATUS |= TWI_BUSSTATE_IDLE_gc;
	// Reset module
	TWI1.MSTATUS |= (TWI_RIF_bm | TWI_WIF_bm);

}
//------------------------------------------------------------------------------------
// Funciones Privadas
//------------------------------------------------------------------------------------
void i2c_tx_slave_devaddress(bool debug, uint8_t slaveAddres, uint8_t directionBit)
{
	// Pass1: Mando un START y el SLAVE_ADDRESS (SLA_W/R)
	// El START se genera automaticamente al escribir en el reg MADDR.

	uint8_t txAddress;

	txAddress = (slaveAddres << 1) + directionBit;
	TWI1.MADDR = txAddress;
	if (debug)
		xprintf("i2c_tx_slave_devaddress: ADDR=0x%02x\r\n",txAddress);

}
//------------------------------------------------------------------------------------
bool i2c_await_ack(bool debug)
{

uint8_t tryes = 10;

	while(tryes-- > 0) {
		if ((TWI1.MSTATUS & TWI_RXACK_bm) == 0) {
			//if (debug) {
				xprintf("i2c_await_ack ACK: STATUS=0x%02x\r\n",TWI1.MSTATUS);
			//} else {
			//	_delay_ms(3);
			//}
			return(true);
		}
		_delay_ms(10);
	}
	if(debug)
		xprintf("i2c_await_ack TO: STATUS=0x%02x\r\n",TWI1.MSTATUS);
		
	return(false);
}
//------------------------------------------------------------------------------------
bool i2c_rx_slave_byte( bool debug, char *rxChar)
{
	// Espera con timeout recibir un byte del slave
	
	uint8_t timeout = 5;

	while(timeout-- > 0) {
		if ( (TWI_RIF_bm & TWI1.MSTATUS) != 0 ) {
			*rxChar = TWI1.MDATA;
			if(debug)
				xprintf("i2c_rx_slave_byte: RXBYTE=0x%02x\r\n", *rxChar );
			return(true);
		}
	}
	if(debug)
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
