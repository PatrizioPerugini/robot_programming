#include <iostream>
#include <cmath>

#include "static_mat.h"
#include "static_vec.h"



float move2 (float q, float dq, float time){
    if(abs(dq <= 0.30)){
        return q;
    }
    else{
        q += dq/time;
        return q;
    }

}

float pinch(float EE, float dEE){
    
    if(abs(dEE <= 0.3)){
        return EE;
    }
    else if(dEE > 0){
        EE +=0.3; 
    }
    else if(dEE > 0){
        EE -=0.3;
    }
     return EE;
}
