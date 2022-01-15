#include <util/delay.h>
#include <stdio.h>
#include <stdint.h>
#include <avr/io.h>
#include "../avr_common/uart.h"


int main(void)
{
    const uint8_t mask=(1<<5);      //range: [2000, 19999] as[0°, 180°]
    int angle = 19999;      
    DDRH |= mask;       // configure the 6 pin as output

    TCCR4A= (1<<WGM41)|(1<<COM4C1)|(1<<COM4C0); //conf. bits for fast PWM, 8bit, non invertible (output compare set low)
    TCCR4B= (1<<WGM42)|(1<<WGM43)| (1<<CS41);   //no prescaling
    
    ICR4 = 19999;
    
    while(1){
        OCR4C= ICR4 - angle;
    }
}