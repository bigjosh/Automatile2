/*
 * APA102C.c
 *
 * Created: 7/21/2015 14:51:49
 *  Author: Joshua
 */ 
#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "Pins.h"
#include "APA102C.h"



void LED_setup() {
    // Set clock and data to output mode    
    LED_DDR |= LED_CLK | LED_DAT;    
    POWER_PORT |= POWER_X;             // Turn off power initially
    POWER_DDR |= POWER_X;   
}

void LED_powerUp() {
    
    POWER_PORT &= ~POWER_X;             // Activate the 5 volt boost converter (inverted logic though PNP)
}

void LED_powerDown() {
    
    POWER_PORT |= POWER_X;             // Activate the 5 volt boost converter (inverted logic though PNP)
}


#define set(port,pin) (port |= pin) // set port pin
#define clear(port,pin) (port &= (~pin)) // clear port pin
#define bit_val(byte,bit) (byte & (1 << bit)) // test for bit set

// TODO:Also combine the clock set and the data set into a single operation?

//bit bangs an SPI signal to the specified pins of the given data
static inline void sendBit(uint8_t data){
    
	if(data){        
        LED_PORT |= LED_DAT;
	}else{
        LED_PORT &= ~LED_DAT;
	}
    
    LED_PORT |= LED_CLK;
    LED_PORT &= ~LED_CLK;
    
}

//TODO: Clean this up with a loop. 

//bit bangs an SPI signal to the specified pins of the given data
static void sendByte(uint8_t data){
    
	sendBit(bit_val(data,7));
	sendBit(bit_val(data,6));
	sendBit(bit_val(data,5));
	sendBit(bit_val(data,4));
	sendBit(bit_val(data,3));
	sendBit(bit_val(data,2));
	sendBit(bit_val(data,1));
	sendBit(bit_val(data,0));
}
//bit bangs an SPI signal to the specified pins that generates the specified color 
//	formatted for the APA102, provided as a byte array of R,G,B

void LED_sendColor(const uint8_t color[3]){

	//Start Frame
	sendByte(0x00);
	sendByte(0x00);
	sendByte(0x00);
	sendByte(0x00);
	//Data
	sendByte(0xE1);//Set brightness to current to minimum TODO: Add setBrightness function (0xE1...0xFF)
	sendByte(color[2]);
	sendByte(color[1]);
	sendByte(color[0]);
}