//[AP]

#include <util/delay.h>
#include <stdio.h>
#include <stdint.h>
#include <avr/io.h>
#include "../avr_common/uart.h"


int main(void)
{
const uint8_t mask=(1<<5);      //range: [2000, 19999] as[0°, 180°]
DDRH |= mask_1;       // configure the 8 pin as output
int angle = 19999;      
DDRH |= mask_1;       // configure the 8 pin as output

TCCR4A= (1<<WGM41)|(1<<COM4C1)|(1<<COM4C0); //conf. bits for fast PWM, 8bit, non invertible (output compare set low)
TCCR4B= (1<<WGM42)|(1<<WGM43)| (1<<CS41);   //no prescaling

ICR4 = 19999;

while(1){
    //angle = 0;       //range: [2000, 19999] as[0°, 180°]
    
    for (int i = angle;i<19999; i+=100){
        angle = i;
       
        OCR4B= ICR4 - angle;
        OCR4C= ICR4 - angle;
        _delay_ms(20);

    }
    
    
    for (int i = angle;i>1000; i-=100){
        angle = i;
        OCR4C= ICR4 - angle;
        OCR4B= ICR4 - angle;
        _delay_ms(10);

    }
    _delay_ms(100);
}
    
    
    
    
}
