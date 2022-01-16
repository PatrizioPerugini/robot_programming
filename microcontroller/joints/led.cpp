#include <util/delay.h>
#include <stdio.h>
#include <stdint.h>
#include <avr/io.h>
#include "../avr_common/uart.h"


int main(void){
  //printf_init(); 

  // we will use timer 1
  TCCR1A=(1<<WGM10)|(1<<COM1C0)|(1<<COM1C1);
  TCCR1B=((1<<WGM12)|(1<<CS10));   
  // clear all higher bits of output compare for timer
  OCR1AH=0;
  OCR1BH=0;
  OCR1CH=0;

  OCR1CL=1;

  // the LED is connected to pin 13
  // that is the bit 7 of port b, we set it as output
  const uint8_t mask=(1<<7);
  // we configure the pin as output
  DDRB |= mask;//mask;

  uint8_t intensity=0;
  while(1){
    // we write on the output compare register a value
    // that will proportional to the opposite of the
    // duty_cycle
    OCR1CL=intensity; 
    
    //printf("v %u\n", (int) OCR1CL);
    _delay_ms(100); // from delay.h, wait 1 sec
    intensity+=8;
  }
  
}