#include <util/delay.h>
#include <stdio.h>
#include <stdint.h>
#include <avr/io.h>
#include "../avr_common/uart.h"
#include <joints.h>

int main(void)
{
    int deg_angles_i = [0 0 0 0 0 0];
    setting(deg_angles_i);
    int active = 1;

    while(on == 1){
        // listener part: i receive a message as a list of the new positions that the joint-i has to reach 

        for(int i=0; i<6; i++){
            OCR3A = ICR3 - angle_1(deg_angles[0], 1);  //new position J1
            OCR3A = ICR3 - angle_1(deg_angles[3], 4);  //new position J4
            OCR3A = ICR3 - angle_1(deg_angles[4], 5);  //new position J5
                
            OCR4B = ICR4 - angle_1(deg_angles[1], 2);  //new position J2
            OCR4C = ICR4 - angle_1(deg_angles[2], 3);  //new position J3
            OCR4A = ICR4 - angle_1(deg_angles[5], 6);  //new position EE
        }
    }


