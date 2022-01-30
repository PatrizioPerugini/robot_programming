#include <iostream>
#include "static_vec.h"
#include "static_mat.h"
#include <Eigen/Core>
//#include <Eigen/StdVector>
//#include <Eigen/Eigenvalues>
#include <cmath>
using namespace rp;
using namespace std;
using namespace Eigen;


#define _USE_MATH_DEFINES

using Angle_v = Eigen::Matrix<float, 3,1>;
using Error_v =Eigen::Matrix<float,6,1>;
using Joint_v=Eigen::Matrix<float, 5,1>;
using Joint_v_error=Eigen::Matrix<float, 5,1>;
using Pose_v=Eigen::Matrix<float, 6,1>;
using Rotation_m = Eigen::Matrix<float, 3, 3>;
using Homogeneous_m = Eigen::Matrix<float, 4, 4>;
using Jacobian_m = Eigen::Matrix<float, 6, 5>;

using DH_row = float[4];
using DH_table = float[5][4];



Homogeneous_m DH_matrix(DH_row& T){
    Homogeneous_m DH;
    float alfa = T[0];
    float a = T[1];
    float d = T[2];
    float theta = T[3];

    DH << cosf(theta), -cosf(alfa)*sinf(theta), sinf(alfa)*sinf(theta), a*cosf(theta), 
        sinf(theta),   cosf(alfa)*cosf(theta), -sinf(alfa)*cosf(theta), a*sinf(theta),
                  0,               sinf(alfa),               cosf(alfa),           d,
                  0,                        0,                         0,           1;

    return DH;
}

Homogeneous_m get_DH(Joint_v &q){

    Homogeneous_m DH;
    
    Homogeneous_m DH_i;
    
    
    int n = 5;
    
    float q_1 = q(0);
    float q_2 = q(1);
    float q_3 = q(2);
    float q_4 = q(3);
    float q_5 = q(4);
    float a_2 = 10.5;
    float a_3= 10;  
    float a_4 = 2.7;  
    float d_5 = 3.3;
    /*
    DH << cos(q_2 + q_3 + q_4)*cos(q_1)*cos(q_5) - sin(q_1)*sin(q_5), - cos(q_5)*sin(q_1) - cos(q_2 + q_3 + q_4)*cos(q_1)*sin(q_5), -sin(q_2 + q_3 + q_4)*cos(q_1), cos(q_1)*(a_3*cos(q_2 + q_3) + a_2*cos(q_2) + a_4*cos(q_2 + q_3 + q_4)) - d_5*sin(q_2 + q_3 + q_4)*cos(q_1),
    cos(q_1)*sin(q_5) + cos(q_2 + q_3 + q_4)*cos(q_5)*sin(q_1),   cos(q_1)*cos(q_5) - cos(q_2 + q_3 + q_4)*sin(q_1)*sin(q_5), -sin(q_2 + q_3 + q_4)*sin(q_1), sin(q_1)*(a_3*cos(q_2 + q_3) + a_2*cos(q_2) + a_4*cos(q_2 + q_3 + q_4)) - d_5*sin(q_2 + q_3 + q_4)*sin(q_1),
                                 sin(q_2 + q_3 + q_4)*cos(q_5),                               -sin(q_2 + q_3 + q_4)*sin(q_5),           cos(q_2 + q_3 + q_4),                     a_3*sin(q_2 + q_3) + a_2*sin(q_2) + d_5*cos(q_2 + q_3 + q_4) + a_4*sin(q_2 + q_3 + q_4),
                                                             0,                                                            0,                              0,                                                                                                           1;
   
    */
    //uncommenting here
    
    DH_table DHTABLE ={
    { -M_PI/2,     0,     0, q_1},
    {    0,  10.5,     0, q_2},
    {    0,    10.0,     0, q_3},
    { -M_PI/2, 2.7,     0, q_4},
    {    0,     0, 3.3, q_5}
    };
    
    DH = DH_matrix(DHTABLE[0]);

    for(int i = 1; i<n; i++){
       DH_i = DH_matrix(DHTABLE[i]);
       DH = DH*DH_i;
    };
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

//input q: in radiants 
//outpu R: rotation matrix
Rotation_m get_orientation(Joint_v &q)
{
    Rotation_m R;
    Homogeneous_m DH = get_DH(q);

     for(int i = 0; i<3; i++){
        for(int j = 0; j<3; j++){
            R(i,j) = DH(i,j);
        }
    };


    /*
    for(int i = 0; i<3; i++){
        for(int j = 0; j<3; j++){
            
            R.at(i,j) = DH.at(i,j);
        }
    };
    */
    return R;
}

//input q: in radiants 
//outpu J: anal Jacobian matrix
Jacobian_m get_jacobian(Joint_v &q)
{
    Jacobian_m J;
    float q_1 = q(0);
    float q_2 = q(1);
    float q_3 = q(2);
    float q_4 = q(3);
    float q_5 = q(4);
    float a_2 = 10.5;
    float a_3= 10;  
    float a_4 = 2.7;  
    float d_5 = 3.3;

    // closed form solution from MATLAB script for analitic jacobian*(M_PI/180)

    J << d_5*sin(q_2 + q_3 + q_4)*sin(q_1) - sin(q_1)*(a_3*cos(q_2 + q_3) + a_2*cos(q_2) + a_4*cos(q_2 + q_3 + q_4)), - cos(q_1)*(a_3*sin(q_2 + q_3) + a_2*sin(q_2) + a_4*sin(q_2 + q_3 + q_4)) - d_5*cos(q_2 + q_3 + q_4)*cos(q_1), - cos(q_1)*(a_3*sin(q_2 + q_3) + a_4*sin(q_2 + q_3 + q_4)) - d_5*cos(q_2 + q_3 + q_4)*cos(q_1), - d_5*cos(q_2 + q_3 + q_4)*cos(q_1) - a_4*sin(q_2 + q_3 + q_4)*cos(q_1), 0,
    cos(q_1)*(a_3*cos(q_2 + q_3) + a_2*cos(q_2) + a_4*cos(q_2 + q_3 + q_4)) - d_5*sin(q_2 + q_3 + q_4)*cos(q_1), - sin(q_1)*(a_3*sin(q_2 + q_3) + a_2*sin(q_2) + a_4*sin(q_2 + q_3 + q_4)) - d_5*cos(q_2 + q_3 + q_4)*sin(q_1), - sin(q_1)*(a_3*sin(q_2 + q_3) + a_4*sin(q_2 + q_3 + q_4)) - d_5*cos(q_2 + q_3 + q_4)*sin(q_1), - d_5*cos(q_2 + q_3 + q_4)*sin(q_1) - a_4*sin(q_2 + q_3 + q_4)*sin(q_1), 0,
                                                                                                          0,                       d_5*sin(q_2 + q_3 + q_4) - a_2*cos(q_2) - a_4*cos(q_2 + q_3 + q_4) - a_3*cos(q_2 + q_3),                       d_5*sin(q_2 + q_3 + q_4) - a_4*cos(q_2 + q_3 + q_4) - a_3*cos(q_2 + q_3),                     d_5*sin(q_2 + q_3 + q_4) - a_4*cos(q_2 + q_3 + q_4), 0,
                                                                                                              0,                                                                                                             0,                                                                                              0,                                                               0, 1, 
                                                                                                              0,                                                                                                             1,                                                                                              1,                                                               1, 0, 
                                                                                                              1,                                                                                                             0,                                                                                              0,                                                               0, 0;


    /*
    float M[6][5] = {{d_5*sin(q_2 + q_3 + q_4)*sin(q_1) - sin(q_1)*(a_3*cos(q_2 + q_3) + a_2*cos(q_2) + a_4*cos(q_2 + q_3 + q_4)), - cos(q_1)*(a_3*sin(q_2 + q_3) + a_2*sin(q_2) + a_4*sin(q_2 + q_3 + q_4)) - d_5*cos(q_2 + q_3 + q_4)*cos(q_1), - cos(q_1)*(a_3*sin(q_2 + q_3) + a_4*sin(q_2 + q_3 + q_4)) - d_5*cos(q_2 + q_3 + q_4)*cos(q_1), -cos(q_1)*(d_5*cos(q_2 + q_3 + q_4) + a_4*sin(q_2 + q_3 + q_4)), 0}, 
{cos(q_1)*(a_3*cos(q_2 + q_3) + a_2*cos(q_2) + a_4*cos(q_2 + q_3 + q_4)) - d_5*sin(q_2 + q_3 + q_4)*cos(q_1), - sin(q_1)*(a_3*sin(q_2 + q_3) + a_2*sin(q_2) + a_4*sin(q_2 + q_3 + q_4)) - d_5*cos(q_2 + q_3 + q_4)*sin(q_1), - sin(q_1)*(a_3*sin(q_2 + q_3) + a_4*sin(q_2 + q_3 + q_4)) - d_5*cos(q_2 + q_3 + q_4)*sin(q_1), -sin(q_1)*(d_5*cos(q_2 + q_3 + q_4) + a_4*sin(q_2 + q_3 + q_4)), 0}, 
{                                                                                                          0,                       a_3*cos(q_2 + q_3) + a_2*cos(q_2) + a_4*cos(q_2 + q_3 + q_4) - d_5*sin(q_2 + q_3 + q_4),                       a_3*cos(q_2 + q_3) + a_4*cos(q_2 + q_3 + q_4) - d_5*sin(q_2 + q_3 + q_4),             a_4*cos(q_2 + q_3 + q_4) - d_5*sin(q_2 + q_3 + q_4), 0}, 
{                                                                                                          0,                                                                                                             0,                                                                                              0,                                                               0, 1}, 
{                                                                                                          0,                                                                                                             1,                                                                                              1,                                                               1, 0}, 
{                                                                                                          1,                                                                                                             0,                                                                                              0,                                                               0, 0}};

    
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
    */
    return J;
}

//input        R : rotation matrix 
//output [a,b,g] : radiants
Angle_v get_RPY(Rotation_m& R){
    Angle_v RPY;
    float R11=R(0,0);
    float R12=R(0,1);
    float R13=R(0,2);
    float R21=R(1,0);
    float R22=R(1,1);
    float R23=R(1,2);
    float R31=R(2,0);
    float R32=R(2,1);
    float R33=R(2,2);
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
    
    RPY(0)=roll;
    RPY(1)=pitch;
    RPY(2)=yaw;

    return RPY;
}

//input              q : rad
//output [x,y,z,a,b,g] : [cm,cm,cm,rad,rad,rad]
Pose_v get_pose(Joint_v &q)
{
    Pose_v p;
    Homogeneous_m DH = get_DH(q);

    // p (x,y,z) = f(q) got from the DHmatrix
    p(0) = DH(0, 3);
    p(1) = DH(1, 3);
    p(2) = DH(2, 3);

    // fi(a,g,c) = inverse rpy_rotation 'xyz'
    Rotation_m R = get_orientation(q);
    Angle_v RPY = get_RPY(R);
    p(3) = RPY(0);
    p(4) = RPY(1);
    p(5) = RPY(2);

    for(int i=0; i<6; i++){
     
        for(int j = 0; j<5; j++){
            if (abs(p(i)) < 0.0001){
                p(i)=0;
            }
        }
    
    }

    return p;
}


//NB r_d is the point where to go, q_k is set with the last position (normally the mid_position)

//input        r_d :  [cm,cm,cm,rad,rad,rad]
//input        q_k : rad
//output q_desired : rad

Joint_v inverse_kinematics(Pose_v& r_d, Joint_v& q_k){ 
    cout<< "Entered inside the function" << endl;
    //instantiate initial error 6 x 1
    Error_v error;
    //general q_k+1, instantiated with zeros
    //Joint_v q_k1;
    int i;
    for(i=0;i<6;i++){
        error(i)=r_d(i)-get_pose(q_k)[i];
    }
    
    cout << error <<endl;
    float alpha=0.001;
    i = 0;
    float norm_cm =sqrt((error(0)*error(0)) + (error(1)*error(1))+(error(2)*error(2)));
    float norm_rad=sqrt((error(3)*error(3)) + (error(4)*error(4))+(error(5)*error(5)));
    cout << "norm cm " << norm_cm <<endl;
    cout << "norm rad " << norm_rad <<endl;
    while (
   /* (abs(error.at(0))>=0.3 ||
    abs(error.at(1))>=0.3  ||
    abs(error.at(2))>=0.3  ||
    abs(error.at(3))>=3*(M_PI/180) ||
    abs(error.at(4))>=3*(M_PI/180) ||
    abs(error.at(5))>=3*(M_PI/180))*/
    (norm_cm > 0.01 || norm_rad > 0.7*(M_PI/180)) &&
    i < 300){
       //cout<< "Entered inside while because value of error is: "<< error.squaredNorm() << "\n" << endl;
        //compute new error 
        for(int p=0;p<6;p++){ 
            //f_r(q_k) returns a 6 x 1 vector
            
            error(p)=r_d(p)-get_pose(q_k)[p];
       
        }
       // cout<< "iteration " << i << " values for previous error :"<< endl;
       // cout << error << endl;
       // //cout<< "and the norm of e is " << sqrt(error.squaredNorm()) << endl;
        
        cout<< "and the value of q_k is:\n"<<q_k.transpose() << endl;
        //cout << q_k <<""<< endl;
        cout << "error is "<<error.transpose()<<endl;
        cout<< "Entered inside while because value of error is: \n"<< error.squaredNorm() << "\n" << endl;

        Joint_v delta= (get_jacobian(q_k).transpose())*error;//*alpha;

        cout<<"delta is: \n"<<delta.transpose()<<endl;
        
        q_k= q_k + delta*alpha;

       
        //just printing some stuff

        cout<< "iteration " << i << " values for previous : "<< endl;
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



int main(){
   
    
    Pose_v r_d;
    r_d(0)=5.5651;
    r_d(1)=6.205;
    r_d(2)=4.765;
    r_d(3)=-0.1748;
    r_d(4)=M_PI;
    r_d(5)=-0.1748;

    //r_d(0)=0.5651;
    //r_d(1)=3.205;
    //r_d(2)=6.765;
    //r_d(3)=-0.1748;
    //r_d(4)=M_PI;
    //r_d(5)=-0.1748;

    Joint_v q_k;
    q_k(0)=88*M_PI/180;
    q_k(1)=23*M_PI/180;
    q_k(2)=87*M_PI/180;
    q_k(3)=52*M_PI/180;
    q_k(4)=67*M_PI/180;
  //  q_k(5)=41*M_PI/180; 

    Joint_v q_act;
    q_act(0) = 90.0 * (M_PI/180);
    q_act(1) = 75.0 * (M_PI/180);
    q_act(2) = 172.0 * (M_PI/180);
    q_act(3) = 50.0 * (M_PI/180);
    q_act(4) = 90.0 * (M_PI/180); 
    

   // cout<<"q_act pose: " << q_k<< endl;
   // cout<<get_pose(q_act)<<endl;

    
    Joint_v q_desired; 
    q_desired(0)= 80*M_PI/180;
    q_desired(1)= 10*M_PI/180;
    q_desired(2)=110*M_PI/180;
    q_desired(3)= 70*M_PI/180;
    q_desired(4)= 90*M_PI/180;
   // q_desired(5)= 40*M_PI/180;
   Pose_v where =get_pose(q_desired);

    //cout<<"jacobiano per q_desired is " << q_k<< endl;
    //cout<<get_jacobian(q_desired)<<endl;
     
   Joint_v q_ris=inverse_kinematics(where,q_k);
   cout<<"the result is: "<<endl;
   cout << q_ris << endl;
   cout<<"the result should have been: \n"<< q_desired<< endl;

   
   
   
  // cout << q_desired <<endl;
    
    return 0;
      
}