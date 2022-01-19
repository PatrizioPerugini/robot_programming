#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/JointState.h>
#include <sstream>
#include <iostream>
#include "static_vec.h"
#include "static_mat.h"
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



int main(int argc, char **argv)
{
  int i = 0;
  //Current position (p = [px, py, px, alfa, beta, gamma]) initilaized to the setup values of the robot
  
  //initial configuaration (setup values)
  Joint_v q;
  q.at(0) = 90.0;
  q.at(1) = 0.0;
  q.at(2) = 0.0;
  q.at(3) = 150.0;
  q.at(4) = 180.0; 
  
  Position_v p = get_position(q_0);  //initial position
  Position_v p_d; //desired position
  Joint_v q_d; //desired joint conf
  Joint_v_error q_e;  //error: qd - q
  bool position_achieved = 0;

  ros::init(argc, argv, "joint_pub");

  ros::NodeHandle n;

  ros::Publisher pub=n.advertise<sensor_msgs::JointState>("joint_states",1000);

  ros::Rate loop_rate(10);

  sensor_msgs::JointState msg;

  //int cnt =1;
 
  
  while (ros::ok()){
    
    //receive the desired new position p_d
  //if message received
      p_d = ;
      q_d = inverse_kinematics(p_d);    //calculate the dsired new jont values q_d
      q_e = qd - q;
      position_achieved = 0;
      
      //feed gradually the change
      while(position_achieved==0){
        
        if(abs(error.at(0)<=0.09&&
          abs(error.at(1))<=0.09 && 
          abs(error.at(2))<=0.09 && 
          abs(error.at(3))<=0.09 && 
          abs(error.at(4))<=0.09){ 
            position_achieved == 1;
        }

        if(abs(error.at(0)<=0.09){
          
        }

      for(int i = 0; i<4; i++){
      
      msg.position={(double)i, (double)(i/2), double(i-50), 0, 0};
      



    /*
    int second=-40;
    for(int i=-40; i<=40; i++){
    
      msg.position={(double)i, (double)(i/2), double(i-50), 0, 0};
      msg.velocity={0.0};
      msg.effort={0.0};

      ros::Duration(0.01).sleep();
      pub.publish(msg);
    }

    for(int i=40; i>=-40; i--){
      msg.position={(double)i, (double)(i/2), double(i-50), 0, 0};
      msg.velocity={0.0};
      msg.effort={0.0};

      ros::Duration(0.01).sleep();
      pub.publish(msg);
    }
    
    ros::Duration(1.0).sleep();
    */ 

    ros::spinOnce();

    //loop_rate.sleep();

  }

  return 0;
}