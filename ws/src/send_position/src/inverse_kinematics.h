#include <iostream>
#include "static_vec.h"
#include "static_mat.h"
#include <cmath>
using namespace rp;
using namespace std;

#define _USE_MATH_DEFINES


using Angle_v = Vec_<float, 3>;
using Error_v =Vec_<float,6>;
using Joint_v=Vec_<float, 5>;
using Joint_v_error=Vec_<float, 5>;
using Pose_v=Vec_<float, 6>;
using Rotation_m = Mat_<float, 3, 3>;
using Homogeneous_m = Mat_<float, 4, 4>;
using Jacobian_m = Mat_<float, 6, 5>;

using DH_row = float[4];
using DH_table = float[5][4];



Homogeneous_m DH_matrix(DH_row& T){
    Homogeneous_m DH;
    float alfa = T[0];
    float a = T[1];
    float d = T[2];
    float theta = T[3];

    DH.at(0,0) = cosf(theta);
    DH.at(0,1) = -cosf(alfa)*sinf(theta);
    DH.at(0,2) = sinf(alfa)*sinf(theta);
    DH.at(0,3) = a*cosf(theta);

    DH.at(1,0) = sinf(theta);
    DH.at(1,1) = cosf(alfa)*cosf(theta);
    DH.at(1,2) = -sinf(alfa)*cosf(theta);
    DH.at(1,3) = a*sinf(theta);

    DH.at(2,0) = 0;
    DH.at(2,1) = sinf(alfa);
    DH.at(2,2) = cosf(alfa);
    DH.at(2,3) = d;

    DH.at(3,0) = 0;
    DH.at(3,1) = 0;
    DH.at(3,2) = 0;
    DH.at(3,3) = 1;
    return DH;
}

Homogeneous_m get_DH(Joint_v &q){

    Homogeneous_m DH;
    
    Homogeneous_m DH_i;
    int n = 5;
    float q_1 = q.at(0);
    float q_2 = q.at(1);
    float q_3 = q.at(2);
    float q_4 = q.at(3);
    float q_5 = q.at(4);
    float a_2 = 10.5;
    float a_3= 10;  
    float a_4 = 2.7;  
    float d_5 = 3.3;

    float M[4][4] = {
    {cos(q_2 + q_3 + q_4)*cos(q_1)*cos(q_5) - sin(q_1)*sin(q_5), - cos(q_5)*sin(q_1) - cos(q_2 + q_3 + q_4)*cos(q_1)*sin(q_5), -sin(q_2 + q_3 + q_4)*cos(q_1), cos(q_1)*(a_3*cos(q_2 + q_3) + a_2*cos(q_2) + a_4*cos(q_2 + q_3 + q_4)) - d_5*sin(q_2 + q_3 + q_4)*cos(q_1)},
    {cos(q_1)*sin(q_5) + cos(q_2 + q_3 + q_4)*cos(q_5)*sin(q_1),   cos(q_1)*cos(q_5) - cos(q_2 + q_3 + q_4)*sin(q_1)*sin(q_5), -sin(q_2 + q_3 + q_4)*sin(q_1), sin(q_1)*(a_3*cos(q_2 + q_3) + a_2*cos(q_2) + a_4*cos(q_2 + q_3 + q_4)) - d_5*sin(q_2 + q_3 + q_4)*sin(q_1)},
    {                             sin(q_2 + q_3 + q_4)*cos(q_5),                               -sin(q_2 + q_3 + q_4)*sin(q_5),           cos(q_2 + q_3 + q_4),                     a_3*sin(q_2 + q_3) + a_2*sin(q_2) + d_5*cos(q_2 + q_3 + q_4) + a_4*sin(q_2 + q_3 + q_4)},
    {                                                         0,                                                            0,                              0,                                                                                                           1}
    };

    /*
    DH_table DHTABLE ={
    { M_PI/2,     0,     0, q_1},
    {    0,  21/2,     0, q_2},
    {    0,    10,     0, q_3},
    {-M_PI/2, 27/10,     0, q_4},
    {    0,     0, 33/10, q_5}
    };
    for(int i = 1; i<n; i++){
       DH_i = DH_matrix(DHTABLE[i]);
       DH = DH*DH_i;
    };
    */
    for(int i = 0; i<4; i++){
        for(int j = 0; j<4; j++){
            if (abs(M[i][j]) < 0.0001){
                DH.at(i,j)=0;
            }
            else{ 
            DH.at(i,j) = M[i][j] ;
            }
        }
    }


    return DH;
}

/*
q_1 = M_PI/2;
q_2 = M_PI/3;
q_3 = 0;
q_4 = M_PI;
q_5 = 0;

ans =

[          0, -1,         0,               0]

[       -1/2,  0, 3^(1/2)/2,           89/10]

[ -3^(1/2)/2,  0,      -1/2, (89*3^(1/2))/10]

[          0,  0,         0,               1]

*/

Rotation_m get_orientation(Joint_v &q)
{
    Rotation_m R;
    Homogeneous_m DH = get_DH(q);
    for(int i = 0; i<3; i++){
        for(int j = 0; j<3; j++){
            
            R.at(i,j) = DH.at(i,j);
        }
    };
    return R;
}


Jacobian_m get_jacobian(Joint_v &q)
{
    Jacobian_m J;
    float q_1 = q.at(0);
    float q_2 = q.at(1);
    float q_3 = q.at(2);
    float q_4 = q.at(3);
    float q_5 = q.at(4);
    float a_2 = 10.5;
    float a_3= 10;  
    float a_4 = 2.7;  
    float d_5 = 3.3;

    // closed form solution from MATLAB script for geometric jacobian
    float M[6][5] = {{-sin(q_1)*(a_3*cos(q_2 + q_3) + a_2*cos(q_2)), -cos(q_1)*(a_3*sin(q_2 + q_3) + a_2*sin(q_2)), -a_3*sin(q_2 + q_3)*cos(q_1), 0, 0},  
{ cos(q_1)*(a_3*cos(q_2 + q_3) + a_2*cos(q_2)), -sin(q_1)*(a_3*sin(q_2 + q_3) + a_2*sin(q_2)), -a_3*sin(q_2 + q_3)*sin(q_1), 0, 0},  
{                                            0,             a_3*cos(q_2 + q_3) + a_2*cos(q_2),           a_3*cos(q_2 + q_3), 0, 0},  
{                                            0,                                             0,                            0, 0, 1},  
{                                            0,                                             1,                            1, 1, 0},  
{                                            1,                                             0,                            0, 0, 0}};


    
    for(int i = 0; i<6; i++){
        for(int j = 0; j<5; j++){
            if (abs(M[i][j]) < 0.0001){
                J.at(i,j)=0;
            }
            else{ 
            J.at(i,j) = M[i][j] ;
            }
        }
    }

    return J;
}

//this works
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

    if (R31 != 1.0 && R31 != -1.0){ 
        
        float pitch_1 = -1*asin(R31);
        float pitch_2 = M_PI - pitch_1;
       
        float roll_1 = atan2( R32 / cos(pitch_1) , R33 /cos(pitch_1));
        float roll_2 = atan2( R32 / cos(pitch_2) , R33 /cos(pitch_2));
        
        float yaw_1 = atan2( R21 / cos(pitch_1) , R11 / cos(pitch_1));
        float yaw_2 = atan2( R21 / cos(pitch_2) , R11 / cos(pitch_2));
        //MORE THEN ONE SOLUTION, CHOOSE THE MORE SUITABLE ONE
       //pitch = pitch_1;
       //roll = roll_1;
       // yaw = yaw_1 ;
        pitch = pitch_2;
        roll = roll_2;
        yaw = yaw_2 ;
       
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
   // roll = roll*180/M_PI;
   // pitch = pitch*180/M_PI;
   // yaw = yaw*180/M_PI ;
    RPY.at(0)=roll;
    RPY.at(1)=pitch;
    RPY.at(2)=yaw;

    return RPY;
}

Pose_v get_pose(Joint_v &q)
{
    Pose_v p;
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

    for(int i=0; i<6; i++){
     
        for(int j = 0; j<5; j++){
            if (abs(p.at(i)) < 0.0001){
                p.at(i)=0;
            }
        }
    
    }

    return p;
}


//NB r_d is the point where to go, q_k is set with the last position (normally the mid_position)
Joint_v inverse_kinematics(Pose_v& r_d, Joint_v& q_k){ 
    cout<< "Entered inside the function" << endl;
    //instantiate initial error 6 x 1
    Error_v error;
    //general q_k+1, instantiated with zeros
    //Joint_v q_k1;
    int i;
    for(i=0;i<6;i++){
        error.at(i)=r_d.at(i)-get_pose(q_k).at(i);
    }
    

    float alpha=0.000001;
    i = 0;
    
    while (
    //abs(error.at(0))>=0.01 &&
    //abs(error.at(1))>=0.01 && 
    //abs(error.at(2))>=0.01 && 
    //abs(error.at(3))>=0.3*(M_PI/180) && 
    //abs(error.at(4))>=0.3*(M_PI/180) && 
    //abs(error.at(5))>=0.3*(M_PI/180) && 
    i < 100000){
       cout<< "Entered inside while \n" << endl;
        //compute new error 
        for(int p=0;p<6;p++){ 
            //f_r(q_k) returns a 6 x 1 vector
            
            error.at(p)=r_d.at(p)-get_pose(q_k).at(p);
       
        }
        cout<< "iteration " << i << " values for previous error :"<< endl;
        cout << error << endl;
        //cout<< "and the norm of e is " << sqrt(error.squaredNorm()) << endl;
        cout<< "and the value of q_k is:" << endl;
       
        
        q_k= q_k + (get_jacobian(q_k).transpose())*error*alpha;

        cout << q_k <<"\n"<< endl;
        //just printing some stuff

       // cout<< "iteration " << i << " values for previous : "<< endl;
       // cout << q_k << endl;
       // 
       // cout << "the error is " << sqrt(q_k.squaredNorm()) << endl;
//
       // cout<< "iteration " << i << " values for actual : "<< endl;
       // cout << q_k1 << endl;

        //q_k=q_k1;

        i++;
         
    }
    return q_k;
}

