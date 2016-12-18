/*
 * Automatile2.c
 *
 * Created: 12/18/2016 3:22:13 PM
 * Author : passp
 */ 

#include <avr/io.h>
#include <avr/delay.h>

#define F_CLK 1000000       // Running at 1MHz

#include "APA102C.h"

void setup() {
    LED_setup();
}


void loop() {

    LED_powerUp();
       

    LED_sendColor( 0x0000ff );
    _delay_ms(500);


    LED_sendColor( 0xff0000 );    
    _delay_ms(500);



    LED_powerDown();
    _delay_ms(1000);

    LED_sendColor( 0x00ff00 );
    _delay_ms(500);

    LED_sendColor( 0x0000ff );
    _delay_ms(500);

            
}


int main(void)
{
    
     DDRA |= _BV(6);
     while (1)  {
     PORTA |= _BV(6);
         _delay_ms(500);

     PORTA &= ~_BV(6);
         _delay_ms(500);
     }
    /* Replace with your application code */
    while (1) 
    {
        loop();
    }
}

