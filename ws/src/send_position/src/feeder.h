#include <iostream>
#include <cmath>
#include "inverse_kinematics.h"
#include "static_mat.h"
#include "static_vec.h"

using Angle_v = Vec_<float, 3>;
using Rotation_m = Mat_<float, 3, 3>;
using Homogeneous_m = Mat_<float, 4, 4>;
using Jacobian_m = Mat_<float, 6, 5>;
using Joint_v=Vec_<float, 5>;
using Joint_v_error=Vec_<float, 5>;
using Position_v=Vec_<float, 6>;
using DH_row = float[4];
using DH_table = float[5][4];
using Error_v =Vec_<float,6>;


float move2 (float& q, float& dq, float& time){
    if(abs(dq <= 0.30)){
        return q;
    }
    else{
        q += dq/time;
        return q;
    }

}

float pinch(float& EE, float& dEE){
    if(abs(dEE <= 0.3)){
        return EE;
    }
    else if(dEE > 0){
        EE +=0.3; 
    }
    else if(dEE > 0){
        EE -=0.3;
    }
    
}
