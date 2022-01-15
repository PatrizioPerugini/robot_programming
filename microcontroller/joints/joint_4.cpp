#include <util/delay.h>
#include <stdio.h>
#include <stdint.h>
#include <avr/io.h>
#include "../avr_common/uart.h"


int joint_4(void)
{
    const uint8_t mask=(1<<4);
    int angle;
    DDRE |= mask; //pin 2 (PE4)

    TCCR3A= (1<<WGM31)|(1<<COM3A1)|(1<<COM3A0); //conf. bits for fast PWM, 8bit, TOP ICR, 
    TCCR3B= (1<<WGM32)|(1<<WGM33)|(1<<CS31);
    
    ICR3 = 19999;

    OCR3B =  ICR1 - 2000;
    while(1){

    }
}