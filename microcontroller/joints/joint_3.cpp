//[AP]

#include <util/delay.h>
#include <stdio.h>
#include <stdint.h>
#include <avr/io.h>
#include "../avr_common/uart.h"


int main(void)
{
    const uint8_t mask=(1<<5);
    int angle;
    DDRH |= mask;       // configure the 6 pin as output

    TCCR4A= (1<<WGM41)|(1<<COM4C1)|(1<<COM4C0); //conf. bits for fast PWM, 8bit, non invertible (output compare set low)
    TCCR4B= (1<<WGM42)|(1<<WGM43)| (1<<CS40);   //no prescaling
    
    //ICR4 = 19999;
    
    angle = 2000;       //range: [2000, 19999] as[0°, 180°]

    OCR4C= angle;
    
    
}
