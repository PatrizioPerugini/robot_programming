#include <avr.h>/io.h>
#include <util/delay.h>
#define TCCARA_MASK


int main(void)
{
    TCCR4A= (1<<WGM10)|(1<<COM1C0)|(1<<COM1C0); //conf. bits for fast PWM, 8bit, non invertible (output compare set low)
    TCCR4B= (1<<WGM12)|(1<<CS10);

    OCR4A= 0; //clear all higher bits for output compare 

    const uint8_t mask=(1<<6);
    DDRB |= mask;       // configure the pin as output

    
    while(1)
    {

    }
}