#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>


float deg2set (float* deg_angle, int n_joint)
{
    float conv_factor[6] = {5000/180, 5000/270, 5000/270, 5000/180, 5000/180, 5000/180} ;
    float angle;
    
    angle= (deg_angle[n_joint-1] * conv_factor[n_joint-1])-1000;
    
    return angle;
}

int main(void)
{
    const uint8_t mask=(1<<3)|(1<<4)|(1<<5)|(1<<6);        //range: [2000, 19999] as[0°, 180°]
    
    DDRE |= mask;   
    DDRH |= mask;
    
    
    TCCR4A= (1<<WGM41)|(1<<COM4C1)|(1<<COM4C0); //conf. bits for fast PWM, 8bit, non invertible (output compare set low)
    TCCR4B= (1<<WGM42)|(1<<WGM43)| (1<<CS41);   //no prescaling
    
    cli();
    TIMSK4 |= (1 << OCIE4A);  // enable the timer interrupt
   
    ICR4 = 19999;       //period of 2ms
    uint16_t servo1; //J1 --> pin 2 (PE4)
    uint16_t servo2; //J2 --> pin 3 (PE5)
    uint16_t servo3; //J3 --> pin 9 (PH6)
    uint16_t servo4; //J4 --> pin 5 (PE3)
    uint16_t servo5; //J5 --> pin 6 (PH3)
    uint16_t servo6; //EE --> pin 7 (PH4)

    int servo [6] = {servo1, servo2, servo3, servo4, servo5, servo6}
    float servo_i [6] = {90, 0, 20, 90, 90, 90}
    sei();

    for(int i =0; i<6, i++){
        servo[i] = servo_i [i];
    sei();

    //range [1000, 5000] as [0°, 180°]

    float deg_angles [6] = {0, 0, 0, 0, 0, 0};

    while(1)
    {
        
       
        if (TCNT4 >= servo1 &&  bit_is_set(PORTH, PINH4)) PORTH &= ~(1<<PH4);
        if (TCNT4 >= servo2 &&  bit_is_set(PORTH, PINH3)) PORTH &= ~(1<<PH3);
        if (TCNT4 >= servo3 &&  bit_is_set(PORTE, PINE3)) PORTE &= ~(1<<PE3);
        if (TCNT4 >= servo4 &&  bit_is_set(PORTH, PINH6)) PORTH &= ~(1<<PH6);
        if (TCNT4 >= servo5 &&  bit_is_set(PORTE, PINE5)) PORTE &= ~(1<<PE5);
        if (TCNT4 >= servo6 &&  bit_is_set(PORTE, PINE4)) PORTE &= ~(1<<PE4);

        _delay_ms(10);
        }
    }

}

ISR (TIMER4_COMPA_vect)
{
    PORTE = 0xFF;
    PORTH = 0xFF;
}

