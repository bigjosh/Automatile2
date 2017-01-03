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
    
    ADMUX = 0b00100001;     // Select internal 1.1V reference to measure
   
    //ADMUX = 0b00000000;     // Select internal 1.1V reference to measure
        
    ADCSRA |=  _BV(ADSC);        // Start conversion, clear previous complete flag, preserve enable
              
    while ( ADCSRA & _BV(ADSC));      // Wait for conversion to complete;
            
    // Do it again because we switched reference and need time to stabilize. 
    // TODO: Check if we care for the number of bits we need.

    ADCSRA |=  _BV(ADSC);        // Start conversion, clear previous complete flag, preserve enable

    while ( ADCSRA & _BV(ADSC));      // Wait for conversion to complete;

    //sprint(ADCH);
                           
    uint8_t a = ADCH;       // V = 1.1/Vcc
        
    //if (a>vh) vh=a;
    //if (a<vl) vl=a;
    
    //sprint(vh);
    //sprint(vl);
        
    threshold = ((uint8_t) ( (THRESHOLD_V/REF_V) * 255.0 )) * a;       // This is an int multiply. Float calculation is static. Don't loose precision to rounding errors.
    
    ADCSRA &= ~_BV(ADEN);        // Disable ADC  - Force a long conversion on next pass becuase we changed the REF
    
    //sprint( threshold);
}

uint16_t refreshThresholdCountdown; // TODO: Put countdown in dedicated register during sleep so no memory load needed. Maybe ok to use an int8? 255* 250ms = ~ once per minute

// Quickly check if any input pins are above the threshold
// Should be fastest when answer is none. 
// Leaves ADC enabled on return if one is found (assume you will need it again)

inline uint8_t quickscan(void) {
    
    //PORTB |= _BV(PORTB0);
    
    ADCSRA|= _BV(ADEN);        // Enable ADC
    
    // TODO: Use freerunning mode, unroll loop, lockstep if we need it, tuck countdown calculation into the first 25 cycle delay
        
    for( uint8_t edge=0; edge<6; edge++) {     

        ADMUX= edge;          // Select pin, use Vcc as reference
            
        ADCSRA |=  _BV(ADSC);        // Start conversion
    
        PORTB |= _BV(PORTB1);

        while ( ADCSRA & _BV(ADSC));      // Wait for conversion to complete;
        PORTB &= ~_BV(PORTB1);      // Turn off indication LED   
        
        
        uint8_t v = ADCH;
        
        //sprint(edge);
        //sprint(v);
            
        if ( v > threshold ) {
           return 1;
        }
    }

    ADCSRA &= ~_BV(ADEN);        // Disable ADC
    
    /*
    if (!--refreshThresholdCountdown) {     
        setThreshold();
    }
    */
    
//    PORTB &= ~_BV(PORTB0);      // Turn off indication LED
        
    return(0);
                
}

uint8_t rx_bits;

// Scan the 6 input lines and see which ones are above threshold
// Assumes ADC already enabled

void scan(void) {
    
    PORTB |= _BV(PORTB0);
          
    rx_bits = 0;
    
    for( uint8_t edge=0; edge<3; edge++) {     // TODO: CHeck more than 1

        ADMUX= edge;          // Select pin, use Vcc as reference
        
        ADCSRA |=  _BV(ADSC);        // Start conversion
        
        while ( ADCSRA & _BV(ADSC));      // Wait for conversion to complete;
        
        uint8_t v = ADCH;
        
        sprint(edge);
        sprint(v);
        
        if ( v > threshold ) {
            rx_bits |= 1;
        }
        
        rx_bits <<=1;
    }
    
    
    /*
    if (!--refreshThresholdCountdown) {
        setThreshold();
    }
    
    */
    
    PORTB &= ~_BV(PORTB0);      // Turn off indication LED
        
}


int main(void)
{
    
    // Output bitstream on PB1, PB0 trigger
    
    DDRB |= _BV(0) | _BV(1) | _BV(2);
    
    PORTB |= _BV(1);    // Output to drive LED
    
    DIDR0 = 0b00111111;     // Disable digital input on all IR LED pins to save power since we will be floating
        
    ADCSRB |= _BV( ADLAR);  // Left justify result so we only need to read the high 8 bits.
        
    setThreshold();
    
    /*
    while (1) {
        PORTA |=_BV(1);
        _delay_ms(200);
        PORTA &= ~_BV(1);
        _delay_ms(200);
    }
    */
    //PORTA |= _BV(2);        // Pull-up n pin PA2
    
    
    // Input D to A on PA0
        
      
    while (1) 
    {
          PORTB |= _BV(PORTB0);      // Turn off indication LED

         if (quickscan()) {
             //PORTB |= _BV(PORTB1);
         } else {
             //PORTB &= !_BV(PORTB1)             ;
             
         //    scan();
         //    sprint( rx_bits );             
             
         }
         
         PORTB &= ~_BV(PORTB0);      // Turn off indication LED

        
    }
}

