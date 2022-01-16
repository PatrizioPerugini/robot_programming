#include <util/delay.h>
#include <stdio.h>
#include <stdint.h>
#include <avr/io.h>
#include "../avr_common/uart.h"

float deg2set (float* deg_angle, int n_joint)
{
    float conv_factor[6] = {19000/180, 19000/270, 19000/270, 19000/180, 19000/180, 19000/180} ;
    float angle;
    
    angle= deg_angle[n_joint-1] * conv_factor[n_joint-1];
    
    return angle;
}

void setting(float * deg_angles_i){      // angles_i is the initial configuration of the joints
    //configuration of timers 3 and 4, conf. 14 (fast PWM, top ICRn), prescaling x8, invertible
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

    //value in range for J1, J4, J5: [1000, 19999] as [0°, 180°] ==> conversion factor of 1° : 19000/180 (precision 0.3° : 5700/180)
    ICR3 = 19999;
    //value in range for J2, J3: [1000, 19999] as [0°, 270°] ==> conversion factor of 1° : 19000/270 (precision 0.3° : 5700/180)
    ICR4 = 19999;

    OCR3A = ICR3 - deg2set(deg_angles_i, 1);  //initial position J1
    OCR3B = ICR3 - deg2set(deg_angles_i, 4);  //initial position J4
    OCR3C = ICR3 - deg2set(deg_angles_i, 5);  //intial position J5
        
    OCR4B = ICR4 - deg2set(deg_angles_i, 2);  //intial position J2
    OCR4C = ICR4 - deg2set(deg_angles_i, 3);  //intial position J3
    OCR4A = ICR4 - deg2set(deg_angles_i, 6);  //intial position EE
    
}


