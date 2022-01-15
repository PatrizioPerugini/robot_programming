#include <util/delay.h>
#include <stdio.h>
#include <stdint.h>
#include <avr/io.h>
#include "../avr_common/uart.h"


int main(void)
{
    const uint8_t mask=(1<<5);
    int angle;
    DDRB |= mask; //pin 11

    TCCR1A= (1<<WGM11)|(1<<COM1A1)|(1<<COM1A0); //conf. bits for fast PWM, 8bit, TOP ICR, 
    TCCR1B= (1<<WGM12)|(1<<WGM13)|(1<<CS11);
    
    
    ICR1 = 19999;

    OCR1A =  ICR1 - 2000;
    while(1){

    }
}