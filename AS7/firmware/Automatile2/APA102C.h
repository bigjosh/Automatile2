/*
 * APA102C.h
 *
 * Custom Library for writing values to the APA102C RGB LED
 *
 * Created: 7/21/2015 15:11:37
 *  Author: Joshua
 */ 


#ifndef APA102C_H_
#define APA102C_H_

// Set up pins, call once on power-up
void LED_init(void);         

// Powerup the LED. Do before sending a color. Stays on until powerDown()
void LED_powerUp(void);

//Show a color on the LED. Blocking. 
void LED_sendColor(const uint8_t color[3]);

// Powerdown the LED. DO before sleeping. 
void LED_powerDown(void);

#endif /* APA102C_H_ */