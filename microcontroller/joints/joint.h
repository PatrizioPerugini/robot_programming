#include <util/delay.h>
#include <stdio.h>
#include <stdint.h>
#include <avr/io.h>
#include "../avr_common/uart.h"


class Joints {
public:






}

int setting(int angles_i){
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
    DDRE |= mask;   //pin 
    DDRH |= mask;

    //value in range for J1, J4, J5: [1000, 19999] as [0°, 180°] ==> conversion factor of 1° : 19000/180 (precision 0.3° : 5700/180)
    ICR3 = 19999;
    //value in range for J2, J3: [1000, 19999] as [0°, 270°] ==> conversion factor of 1° : 19000/270 (precision 0.3° : 5700/180)
    ICR4 = 19999;
}


int angle_1 (int deg_angle, int n_joint)
{
    float conv_factor = [19000/180 19000/270 19000/270 19000/180 19000/180];
    return angle = deg_angle*conv_factor[type_angle + 1];
    if n_joint
}