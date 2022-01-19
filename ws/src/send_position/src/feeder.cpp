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

void desired_positionCallback(const send_position::Position& msg){
  
}

int main(int argc, char **argv){

  int i = 0;
  
  //initial configuaration (setup values)
  Joint_v q;
  q.at(0) = 90.0;
  q.at(1) = 0.0;
  q.at(2) = 0.0;
  q.at(3) = 150.0;
  q.at(4) = 180.0; 
  
  Position_v p = get_position(q_0);   //Current position (p = [px, py, px, a, b, g]) initilaized to the setup values
  Position_v p_d; //desired position
  Joint_v q_d; //desired joint conf
  Joint_v_error q_e;  //error: qd - q
  bool position_achieved = 0;

  ros::init(argc, argv, "joint_pub");

  ros::NodeHandle n;

  ros::Publisher pub=n.advertise<sensor_msgs::JointState>("joint_states",1000);

  ros::Subscriber sub= n.subscribe<send_position::Position>("desired_position",1000,desired_positionCallback);

  ros::Rate loop_rate(10);

  sensor_msgs::JointState msg;

  
  while (ros::ok()){
    
    //receive the desired new position p_d
    if //message received
      p_d = ;
      q_d = inverse_kinematics(p_d);    //calculate the dsired new jont values q_d
      q_e = qd - q;
      position_achieved = 0;
      
      //feed gradually the change
      while(position_achieved==0){
        
        //increase the joint values (when out of the accuracy range)
        for(int i = 0; i<5; i++){
          if(abs(error.at(0)<1.0){
            q.at(i)++;
          }
        }
        
        //send new increased positions of the joints and add a delay
        msg.position={(double)q.at(0), (double)q.at(1), (double)q.at(2), (double)q.at(3), (double)q.at(4)};
        ros::Duration(0.01).sleep();
        pub.publish(msg);

        //check if all the values of the joints are now inside the accuracy of servos (0.3°) 
        if(abs(error.at(0)<=0.30 &&
          abs(error.at(1))<=0.30 && 
          abs(error.at(2))<=0.30 && 
          abs(error.at(3))<=0.30 && 
          abs(error.at(4))<=0.30){ 
            position_achieved == 1;
        }
      }

    else{
      msg.position={(double)q.at(0), (double)q.at(1), (double)q.at(2), (double)q.at(3), (double)q.at(4)};
      ros::Duration(0.01).sleep();
      pub.publish(msg);
    }

    ros::spinOnce();
    
  }



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

  

  return 0;
}