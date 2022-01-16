#include <util/delay.h>
#include <stdio.h>
#include <stdint.h>
#include <avr/io.h>
#include "../avr_common/uart.h"
//#include <joints_.h>

float deg2set (float* deg_angle, int n_joint)
{
    float conv_factor[6] = {19000/180, 19000/270, 19000/270, 19000/180, 19000/180, 19000/180} ;
    float angle;
    
    angle= deg_angle[n_joint-1] * conv_factor[n_joint-1];
    
    return angle;
}

int main(void)
{
    // listener part: i receive a message as a list 
    //of the new positions that the joint-i has to reach 
    TCCR3A= (1<<WGM31)|(1<<COM3A1)|(1<<COM3A0); 
    TCCR3B= (1<<WGM32)|(1<<WGM33)|(1<<CS31);

    TCCR4A= (1<<WGM41)|(1<<COM4C1)|(1<<COM4C0); 
    TCCR4B= (1<<WGM42)|(1<<WGM43)| (1<<CS41);   


    //set joint pins as output
    const uint8_t mask=(1<<3)|(1<<4)|(1<<5);
    //J1 --> pin 5 (PE3)
    //J2 --> pin 7 (PH4)
    //J3 --> pin 8 (PH5)
    //J4 --> pin 2 (PE4)
    //J5 --> pin 3 (PE5)
    //EE --> pin 6 (PH3)
    DDRE |= mask;   
    DDRH |= mask;

    ICR3 = 19999;
    ICR4 = 19999;

    float deg_angles [6] = {0, 0, 0, 0, 0, 0};

    while(1)
    {
        for(float pos = 0; pos<180; pos++){
            deg_angles[5] = pos;
           
            OCR3A = ICR3 - deg2set(deg_angles, 1);  // position J1
            OCR3B = ICR3 - deg2set(deg_angles, 4);  // position J4
            OCR3C = ICR3 - deg2set(deg_angles, 5);  // position J5
                
            OCR4B = ICR4 - deg2set(deg_angles, 2);  // position J2
            OCR4C = ICR4 - deg2set(deg_angles, 3);  // position J3
            OCR4A = ICR4 - deg2set(deg_angles, 6);  // position EE
            
            _delay_ms(50);
        }

        for(float pos = 180; pos>0; pos--){
            deg_angles[5] = pos;
           
            OCR3A = ICR3 - deg2set(deg_angles, 1);  
            OCR3B = ICR3 - deg2set(deg_angles, 4);  
            OCR3C = ICR3 - deg2set(deg_angles, 5);  
                
            OCR4B = ICR4 - deg2set(deg_angles, 2);  
            OCR4C = ICR4 - deg2set(deg_angles, 3);  
            OCR4A = ICR4 - deg2set(deg_angles, 6);  
            
            _delay_ms(50);
        }
    }
}




