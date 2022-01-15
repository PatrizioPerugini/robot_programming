//[AP]

#include <util/delay.h>
#include <stdio.h>
#include <stdint.h>
#include <avr/io.h>
#include "../avr_common/uart.h"


int joint_1(void)
{
    const uint8_t mask=(1<<3);
    int angle;
    DDRE |= mask; //pin 5 (PE3)

    TCCR3A= (1<<WGM31)|(1<<COM3A1)|(1<<COM3A0); 
    TCCR3B= (1<<WGM32)|(1<<WGM33)|(1<<CS31);

    ICR3 = 19999;
    
    angle = 2000;       //range: [2000, 19999] as [0°, 180°]

    OCR3A = ICR3 - angle;
    
    
    while(1)
    {        
      
    }
}