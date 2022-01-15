//[AP]

#include <util/delay.h>
#include <stdio.h>
#include <stdint.h>
#include <avr/io.h>
#include "../avr_common/uart.h"


int joint_2(void)
{
    const uint8_t mask=(1<<4);
    int angle;
    DDRH |= mask;       // configure the 7 pin as output

    TCCR4A= (1<<WGM41)|(1<<COM4B1)|(1<<COM4B0); //conf. bits for fast PWM, 8bit, inverted
    TCCR4B= (1<<WGM42)|(1<<WGM43)| (1<<CS40);   //no prescaling
    
    ICR4 = 19999;
    
    angle = 2000;       //range: [2000, 19999] as[0°, 180°]

    OCR4B= angle;
    
    
    while(1)
    {
        OCR4A= angle;         //write the output compare register with the angle
        //printf('v %u\n', (int)OCR4A)
        _delay_ms(100); //delay of 1s
        //angle+=200;
    }
}