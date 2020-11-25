/*
 * led.c
 *
 * Created: 17/11/2020 9:05:05
 *  Author: Pablo
 */ 
#include "./include_app/conversor_i2c_420ma.h"
#include "./include_os/atmel_start_pins.h"

#define LED_PORT	PORTC
#define LED_PIN_bm	PIN2_bm

#define APAGAR_LED() ( LED_PORT.OUT |= LED_PIN_bm )
#define PRENDER_LED() ( LED_PORT.OUT &= ~LED_PIN_bm )

// ---------------------------------------------------------------
void LED_init(void)
{
	// Configura el pin del led como output
	LED_PORT.DIR |= LED_PIN_bm;	
	APAGAR_LED();
}
// ---------------------------------------------------------------
void led_flash(void)
{
	// Prende el led 50ms y lo apaga
	PRENDER_LED();
	_delay_ms( 50 );
	APAGAR_LED();
}
// ---------------------------------------------------------------
void led_test(void)
{
	// En un loop infinito, flashea el led c/1s
	while(1) {
		
		led_flash();
		_delay_ms( 950 );
		wdt_reset();
	}
}
// ---------------------------------------------------------------

