/*
 * TestDualDiode.c
 *
 * Created: 12/29/2016 10:21:46 PM
 * Author : passp
 */ 

#include <avr/io.h>
#include <avr/delay.h>


#define SETBIT(p,b) (p|=_BV(b))
#define CLRBIT(p,b) (p&=~_BV(b))

#define REF_V (1.1)                  // Value of voltage reference
#define THRESHOLD_V (0.600)          // Value of a detected signal from an LED in volts - changes with LEDs of different wavelength. 

#define EDGE_COUNT 6                 // Number of connected edges. Must be 1-6. Edges connected to pins PA0-PA5

uint8_t threshold;              // Value for detected signal measured at an 8-bit integer relative to Vcc. Updated periodically based on current Vcc to reflect dropping battery.

// Print as a serial bit stream on PB2 - Each bit is 17us, just about 38400 baud - LSB first - for debugging. 

void sprint( uint8_t x ) {
    
    PORTB |= _BV(PORTB2);    // Idle on serial port
    _delay_us(22);
    

    PORTB &= ~_BV(PORTB2);    // start bit
    
    _delay_us(22);
    
    for(uint8_t b=8;b>0;b--) {
         
        if (x&0b00000001)    {
            
            PORTB |= _BV(PORTB2);    // start bit
            
        } else {
            PORTB &= ~_BV(PORTB2);    // stop bit
            
            
            
            //_delay_us(1);
            _delay_us(3);
            
        }
        
        _delay_us(15);
                        
        x>>=1;
                 
    }

    PORTB |= _BV(PORTB2);    // stop bit
    _delay_us(26*3);
    
    
}

// Assumes that you will set ADMUX back to use a different reference, and do a long conversion after the change
// This is typical case if you are going to sleep next since waking and enabling the ADC will cause the 1st conversion to be long. 

void setThreshold(void) {       // Set the threshold value based on current Vcc. Assumes ADC is enabled
    
    PORTB |= _BV(PORTB0);
        
    ADMUX = 0b00100001;     // Select internal 1.1V reference to measure
    
    _delay_ms(1);           // After switching to internal voltage reference the ADC requires a settling time of 1ms before
                            // measurements are stable. Conversions starting before this may not be reliable. The ADC must
                            // be enabled during the settling time.
     
    ADCSRA |=  _BV(ADSC);        // Start conversion, clear previous complete flag, preserve enable
              
    while ( ADCSRA & _BV(ADSC));      // Wait for conversion to complete;
    
    uint8_t a=ADCH;
        
    // a = value returned from ADC when measuring internal 1.1V source. 0-255.
    
    // a= (VREF/Vcc)*255 
    // a/255 = VREF/Vcc
    // (a*Vcc)/255 = VREF
    // a*Vcc = VREF * 255
    // Vcc = (VREF*255)/a
       
    // float vcc = (REF_V*255.0)/a;
    // TODO: Might want this someday for low battery warning
    
    // t = the ADC value that maps to the voltage THRESHOLD_V
    
    // t = (THRESHOLD_V/Vcc)*255
    // t = (THRESHOLD_V/((VREF*255)/a))*255
    // t = (THRESHOLD_V * 255) / ((VREF*255)/a)
    // t = (THRESHOLD_V * 255) * ( a / (VREF*255) )
    // t = (THRESHOLD_V * 255 *  a)  / (VREF*255) )
    // t = (THRESHOLD_V *  a)  / VREF )
    // t = (THRESHOLD_V/VREF) *  a
    
    uint8_t t = ((THRESHOLD_V/REF_V) * a);          // Note that we have to do this math in floats since otherwise the scaling factor would round down to 0. 
    
    // TODO: Floats slow. Do in ints. Worth it? We only do this very, very rarely. 
    
    threshold = t;                                  // Save in global variable - we will use it on every scan.    
   
}

uint16_t refreshThresholdCountdown; // TODO: Put countdown in dedicated register during sleep so no memory load needed. Maybe ok to use an int8? 255* 250ms = ~ once per minute

// Quickly check if any input pins are above the threshold
// Should be fastest when answer is none. 
// Leaves ADC enabled on return if one is found (assume you will need it again)

inline uint8_t quickscan(void) {
    
    PORTB |= _BV(PORTB0);
    
    //ADCSRA|= _BV(ADEN);        // Enable ADC
    
    // TODO: Use freerunning mode, unroll loop, lockstep if we need it, tuck countdown calculation into the first 25 cycle delay
        
    uint8_t edge = EDGE_COUNT;
    
    while (edge--) {

        ADMUX= edge;          // Select pin, use Vcc as reference
            
        ADCSRA |=  _BV(ADSC);        // Start conversion
    

        while ( ADCSRA & _BV(ADSC));      // Wait for conversion to complete;
                
        uint8_t v = ADCH;
        
        //sprint(edge);
        //sprint(v);
            
        if ( v > threshold ) {
           return 1;
        }
    }

    //ADCSRA &= ~_BV(ADEN);        // Disable ADC
    
    /*
    if (!--refreshThresholdCountdown) {     
        setThreshold();
    }
    */
    
    PORTB &= ~_BV(PORTB0);      // Turn off indication LED
        
    return(0);
                
}

uint8_t rx_bits;

// Scan the 6 input lines and see which ones are above threshold

void scan(void) {
    
    //PORTB |= _BV(PORTB0);
          
    rx_bits = 0;
    
    // TODO: Reverse order so bit0 is PA0
    // TODO: Run in continuous mode 
    
    uint8_t edge = EDGE_COUNT;
    
    while (edge--) {
                
        rx_bits <<=1;
                
        ADMUX=edge;
        
        ADCSRA |=  _BV(ADSC);        // Start conversion
        
        //PORTB |= _BV(PORTB0);      // Turn off indication LED    
        //PORTB &= ~_BV(PORTB0);      // Turn off indication LED
        
        
        while ( ADCSRA & _BV(ADSC));      // Wait for conversion to complete;
        
        uint8_t v = ADCH;
                
        if ( v > threshold ) {
            rx_bits |= 1;
        }
        
    }
    
    // TODO: Unroll this loop, use T bit to rotate the new bit in. 
    
    
    /*
    if (!--refreshThresholdCountdown) {
        setThreshold();
    }
    
    */
    
    //PORTB &= ~_BV(PORTB0);      // Turn off indication LED
        
}


int main(void)
{
    
    // Output bitstream on PB1, PB0 trigger
    
    DDRB |= _BV(0) | _BV(1) | _BV(2);
    
    PORTB |= _BV(1);    // Output to drive LED
    
    DIDR0 = 0b00111111;     // Disable digital input on all IR LED pins to save power since we will be floating
        
    ADCSRB |= _BV( ADLAR);  // Left justify result so we only need to read the high 8 bits.
            
    ADCSRA|= _BV(ADEN);        // Enable ADC 
    
    setThreshold();
                
    /*
    while (1) {
        PORTA |=_BV(1);
        _delay_ms(200);
        PORTA &= ~_BV(1);
        _delay_ms(200);
    }
    */
      
    while (1) 
    {
        /*
          PORTB |= _BV(PORTB0);      // Turn off indication LED

         if (quickscan()) {
             //PORTB |= _BV(PORTB1);
         } else {
             //PORTB &= !_BV(PORTB1)             ;
             
         //    scan();
         //    sprint( rx_bits );             
             
         }
         
         PORTB &= ~_BV(PORTB0);      // Turn off indication LED
         */
        
        PORTB |= _BV(PORTB1);       // IR ON
        _delay_ms(400);
        
        //_delay_us(10);

        DDRA = 0b111111;             // Purge stranded charge
        DDRA = 0;
        
        scan();
        
        if ( rx_bits & _BV(0) ) {
          PORTB |= _BV(PORTB0);         // Turn off indication LED                                   
        } else {
          PORTB &= ~_BV(PORTB0);        // Turn off indication LED
        }            
        
        PORTB &= ~_BV(PORTB1);       // IR OFF
        _delay_ms(200);
        
        DDRA = 0b111111;             // Purge stranded charge
        DDRA = 0;
        
        //_delay_us(10);
        
        scan();
        
        if ( rx_bits & _BV(0) ) {
            PORTB |= _BV(PORTB0);         // Turn off indication LED
            } else {
            PORTB &= ~_BV(PORTB0);        // Turn off indication LED
        }
       
        //sprint(threshold);
        //sprint(rx_bits);
        
        //setThreshold();
        
    }
    
    ADCSRA &= ~_BV(ADEN);        // Disable ADC  

}

