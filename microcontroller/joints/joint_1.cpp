//[AP]

#include <util/delay.h>
#include <stdio.h>
#include <stdint.h>
#include <avr/io.h>
#include "../avr_common/uart.h"


int main(void)
{
    const uint8_t mask=(1<<3);
    int angle;
    DDRH |= 0xFF;       // configure the 6 pin as output

    TCCR4A= (1<<WGM41)|(1<<COM4A1)|(1<<COM4A0); //conf. bits for fast PWM, 8bit, TOP ICR, 
    TCCR4B= (1<<WGM42)|(1<<WGM43)|(1<<CS41);   //invertible (output compare set High), no prescaling
    
    ICR4 = 19999;
    
    angle = 2000;       //range: [2000, 19999] as [0°, 180°]

    OCR4A = ICR4 - angle;
    
    
    while(1)
    {        
        OCR4A = ICR4 - 800;
        _delay_ms(100);
        OCR4A = ICR4 - 2200;
        _delay_ms(100); 
        //write the output compare register with the angle
    }
}