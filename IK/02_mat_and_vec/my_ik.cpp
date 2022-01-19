//#include "ros/ros.h"
#include <iostream>
#include "static_vec.h"
#include "static_mat.h"
#include <cmath>
using namespace rp;
using namespace std;

#define _USE_MATH_DEFINES

using Angle_v = Vec_<float, 3>;
using Rotation_m = Mat_<float, 3, 3>;
using Homogeneous_m = Mat_<float, 4, 4>;
using Jacobian_m = Mat_<float, 6, 5>;
using Joint_v=Vec_<float, 5>;
using Joint_v_error=Vec_<float, 6>;
using Position_v=Vec_<float, 6>;

/*
DHTABLE =
 
[ pi/2,     0,     0, q_1]
[    0,  21/2,     0, q_2]
[    0,    10,     0, q_3]
[-pi/2, 27/10,     0, q_4]
[    0,     0, 33/10, q_5]


*/

/*

Homogeneous_m get_DH(Joint_v &q){
    Homogeneous_m DH;
    float q_1 = q.at(0);
    float q_2 = q.at(1);
    float q_3 = q.at(2);
    float q_4 = q.at(3);
    float q_5 = q.at(4);
    float M[4][4] = 
    {
        {cos(q_2 + q_3 + q_4) * cos(q_1) * cos(q_5) - sin(q_1) * sin(q_5), -cos(q_5) * sin(q_1) - cos(q_2 + q_3 + q_4) * cos(q_1) * sin(q_5), -sin(q_2 + q_3 + q_4) * cos(q_1), (cos(q_1) * (27 * cos(q_2 + q_3 + q_4) + 100 * cos(q_2 + q_3) + 105 * cos(q_2))) / 10 - (33 * sin(q_2 + q_3 + q_4) * cos(q_1)) / 10},
        {cos(q_1) * sin(q_5) + cos(q_2 + q_3 + q_4) * cos(q_5) * sin(q_1), cos(q_1) * cos(q_5) - cos(q_2 + q_3 + q_4) * sin(q_1) * sin(q_5), -sin(q_2 + q_3 + q_4) * sin(q_1), (sin(q_1) * (27 * cos(q_2 + q_3 + q_4) + 100 * cos(q_2 + q_3) + 105 * cos(q_2))) / 10 - (33 * sin(q_2 + q_3 + q_4) * sin(q_1)) / 10},
        {sin(q_2 + q_3 + q_4) * cos(q_5), -sin(q_2 + q_3 + q_4) * sin(q_5), cos(q_2 + q_3 + q_4), (33 * cos(q_2 + q_3 + q_4)) / 10 + (27 * sin(q_2 + q_3 + q_4)) / 10 + 10 * sin(q_2 + q_3) + (21 * sin(q_2)) / 2},
        {0, 0, 0, 1}
    };

    for(int i = 0; i<4; i++){
        for(int j = 0; j<4; j++){
            DH.at(i,j) = M[i][j]; 
        }
    };
    return DH;
}

Rotation_m get_orientation(Joint_v &q){
    Rotation_m R;
    Homogeneous_m DH = get_DH(q);
    for(int i = 0; i<3; i++){
        for(int j = 0; j<3; j++){
            R.at(i,j) = DH.at(i,j);
        }
    };
    return R;
}

Jacobian_m get_jacobian(Joint_v &q){
    Jacobian_m J;
    float q_1 = q.at(0);
    float q_2 = q.at(1);
    float q_3 = q.at(2);
    float q_4 = q.at(3);
    float q_5 = q.at(4);

    // closed form solution from MATLAB script for geometric jacobian
    float M[6][5] = {
        {-(sin(q_1) * (27 * cos(q_2 + q_3 + q_4) - 33 * sin(q_2 + q_3 + q_4) + 100 * cos(q_2 + q_3) + 105 * cos(q_2))) / 10, -(cos(q_1) * (33 * cos(q_2 + q_3 + q_4) + 27 * sin(q_2 + q_3 + q_4) + 100 * sin(q_2 + q_3) + 105 * sin(q_2))) / 10, -(cos(q_1) * (100.0 * sin(q_2 + q_3) + 3 * 202 ^ (1 / 2) * cos(q_2 + q_3 + q_4 - atan(9 / 11)))) / 10, -(3 * cos(q_1) * (11 * cos(q_2 + q_3 + q_4) + 9 * sin(q_2 + q_3 + q_4))) / 10, 0},
        {(cos(q_1) * (27 * cos(q_2 + q_3 + q_4) - 33 * sin(q_2 + q_3 + q_4) + 100 * cos(q_2 + q_3) + 105 * cos(q_2))) / 10, -(sin(q_1) * (33 * cos(q_2 + q_3 + q_4) + 27 * sin(q_2 + q_3 + q_4) + 100 * sin(q_2 + q_3) + 105 * sin(q_2))) / 10, -(sin(q_1) * (100 * sin(q_2 + q_3) + 3 * 202 ^ (1 / 2) * cos(q_2 + q_3 + q_4 - atan(9 / 11)))) / 10, -(3 * sin(q_1) * (11 * cos(q_2 + q_3 + q_4) + 9 * sin(q_2 + q_3 + q_4))) / 10, 0},
        {0, 10 * cos(q_2 + q_3) + (21 * cos(q_2)) / 2 + (3 * sqrt(202) ^(1 / 2) * cos(q_2 + q_3 + q_4 + atan(11 / 9))) / 10, 10 * cos(q_2 + q_3) + (3 * 202 ^ (1 / 2) * cos(q_2 + q_3 + q_4 + atan(11 / 9))) / 10, (3 * 202.0 ^(1 / 2) * cos(q_2 + q_3 + q_4 + atan(11 / 9))) / 10, 0},
        {0, sin(q_1), sin(q_1), sin(q_1), -cos(q_4) * (cos(q_1) * cos(q_2) * sin(q_3) + cos(q_1) * cos(q_3) * sin(q_2)) - sin(q_4) * (cos(q_1) * cos(q_2) * cos(q_3) - cos(q_1) * sin(q_2) * sin(q_3))},
        {0, -cos(q_1), -cos(q_1), -cos(q_1), sin(q_4) * (sin(q_1) * sin(q_2) * sin(q_3) - cos(q_2) * cos(q_3) * sin(q_1)) - cos(q_4) * (cos(q_2) * sin(q_1) * sin(q_3) + cos(q_3) * sin(q_1) * sin(q_2))},
        {1, 0, 0, 0, cos(q_4) * (cos(q_2) * cos(q_3) - sin(q_2) * sin(q_3)) - sin(q_4) * (cos(q_2) * sin(q_3) + cos(q_3) * sin(q_2))}
    };
    for(int i = 0; i<6; i++){
        for(int j = 0; j<5; j++){
            J.at(i,j) = M[i][j] ;
        }
    };

    return J;
}
*/
Angle_v get_RPY(Rotation_m& R){
    Angle_v RPY;
    float R11=R.at(0,0);
    float R12=R.at(0,1);
    float R13=R.at(0,2);
    float R21=R.at(1,0);
    float R22=R.at(1,1);
    float R23=R.at(1,2);
    float R31=R.at(2,0);
    float R32=R.at(2,1);
    float R33=R.at(2,2);
    float roll=0.0;
    float pitch=0.0;
    float yaw=0.0;
    int i=1;
    
    cout << R31 << endl;
    
    

    if (R31 != 1.0 && R31 != -1.0){ 
        
       

        
        float pitch_1 = -1*asin(R31);
        float pitch_2 = M_PI - pitch_1;
        cout << "pitch 1" << endl;
        cout << pitch_1 << endl;
        float roll_1 = atan2( R32 / cos(pitch_1) , R33 /cos(pitch_1));
        float roll_2 = atan2( R32 / cos(pitch_2) , R33 /cos(pitch_2));
        cout << "roll_1" << endl;
        cout << roll_1 << endl;
        float yaw_1 = atan2( R21 / cos(pitch_1) , R11 / cos(pitch_1));
        float yaw_2 = atan2( R21 / cos(pitch_2) , R11 / cos(pitch_2));
        //MORE THEN ONE SOLUTION, CHOOSE THE MORE SUITABLE ONE
        pitch = pitch_1;
        roll = roll_1;
        yaw = yaw_1 ;
        cout << "pitch " << endl;
        cout << pitch << endl;
    }
    else{ 
   
     float yaw = 0 ;// anything (we default this to zero)
        if (R31 == -1){ 
            float pitch = M_PI/2 ;
            float roll = yaw + atan2(R12,R13) ;
        }
        else{  
            float pitch = -M_PI/2; 
            float roll = -1*yaw + atan2(-1*R12,-1*R13) ;
        }
    }
    //convert from radians to degrees
    roll = roll*180/M_PI;
    pitch = pitch*180/M_PI;
    yaw = yaw*180/M_PI ;
    RPY.at(0)=roll;
    RPY.at(1)=pitch;
    RPY.at(2)=yaw;

    return RPY;
}
/*
Position_v get_position(Joint_v &q)
{
    Position_v p;
    Homogeneous_m DH = get_DH(q);

    // p (x,y,z) = f(q) got from the DHmatrix
    p.at(0) = DH.at(0, 3);
    p.at(1) = DH.at(1, 3);
    p.at(2) = DH.at(2, 3);

    // fi(a,g,c) = inverse rpy_rotation 'xyz'
    Rotation_m R = get_orientation(q);
    Angle_v RPY = get_RPY(R);
    p.at(3) = RPY.at(0);
    p.at(4) = RPY.at(1);
    p.at(5) = RPY.at(2);

    return p;
}
*/

//NB r_d is the point where to go, q_k is set with the last position (normally the mid_position)
Joint_v inverse_kinematics(Position_v& r_d, Joint_v& q_k){ 
    
    //instantiate initial error 6 x 1
    Joint_v_error error;
    //general q_k+1, instantiated with zeros
    Joint_v q_k1;
    
    Joint_v tmp;
    
    int i;
    for(i=0;i<6;i++){
        error.at(i)=100;
    }
    //initialize q_k+1
    for(i=0;i<5;i++){
        q_k1.at(i)=0;
    }

    float alpha=0.05;
    i = 0;
    //while(i < 100){
    while(error.squaredNorm() > 0.5 && i < 10){
        //compute new error 
        for(int p=0;p<6;p++){ 
            //f_r(q_k) returns a 6 x 1 vector
          
           // error.at(p)=r_d.at(p)-get_Position(q_k).at(i);
            
            error.at(p)=r_d.at(p);//-get_Position(q_k).at(i);

        }

        //uncomment this if want to try

        q_k1= q_k -q_k*alpha;

        //comment this if willing to try it

       // q_k1= q_k + alpha*(get_Jacobian(q_k).transpose)*error;

        //just printing some stuff

        cout<< "iteration " << i << " values for previous : "<< endl;
        cout << q_k << endl;
        
        cout << "the error is " << sqrt(q_k.squaredNorm()) << endl;

        cout<< "iteration " << i << " values for actual : "<< endl;
        cout << q_k1 << endl;

        q_k=q_k1;

        i++;      
    }
    return q_k;
}


int main(){
    
    Position_v R_d;
    R_d.at(0)=23;
    R_d.at(1)=12;
    R_d.at(2)=15;
    R_d.at(3)=1.45;
    R_d.at(4)=0.34;
    R_d.at(5)=1.98;
    Joint_v q_k;
    q_k.at(0)=90;
    q_k.at(1)=0;
    q_k.at(2)=130;
    q_k.at(3)=90;
    q_k.at(4)=0;
    q_k.at(5)=0;



    //Joint_v ris=inverse_kinematics(R_d,q_k);

    //cout << ris << endl;

    Rotation_m R;
    R.at(0,0)=1;
    R.at(0,1)=0;
    R.at(0,2)=0;
    R.at(1,0)=0;
    R.at(1,1)=1;
    R.at(1,2)=0;
    R.at(2,0)=-0.7;
    R.at(2,1)=0;
    R.at(2,2)=0;

    cout << "ROTATION MATRIX IS: \n"<< R << endl;  

    Angle_v RPY=get_RPY(R);

    cout << "orientation issss :  "<< RPY << endl;  
    /*

    Joint_v q_k;
    q_k.at(0)=90;
    q_k.at(1)=0;
    q_k.at(2)=130;
    q_k.at(3)=90;
    q_k.at(4)=0;
    q_k.at(5)=0;
    */
   // Homogeneous_m DH = get_DH(q_k);
   // cout << "DHmatrix issss :  "<< DH << endl;  
    return 0;
      
}

