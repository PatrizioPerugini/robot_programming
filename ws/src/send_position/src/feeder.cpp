#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/JointState.h>
#include <sstream>
#include <iostream>
#include <cmath>
#include "feeder.h"
#include "inverse_kinematics.h"
#include "static_mat.h"
#include "static_vec.h"
#include <geometry_msgs/Pose.h>


sensor_msgs::JointState msg;
Pose_v r_d; //desired position
Joint_v q_d; //desired joint conf
Joint_v_error dq;  //error: qd - q
float EE_d;
float EE;
float dEE;
Pose_v p_0;
Joint_v q_0;
Joint_v q;
const float max_velocity = 0.15;
float delay = 0.01;    // frequence of the messages to arduino 
float duration;
ros::Publisher pub;
ros::NodeHandle n;

void setup(){
  
  int i = 0;
  
  //initial configuaration (setup values)
  
  q_0.at(0) = 90.0;
  q_0.at(1) = 0.0;
  q_0.at(2) = 0.0;
  q_0.at(3) = 150.0;
  q_0.at(4) = 180.0; 
  
  p_0= get_pose(q_0);   //Current pose (p = [px, py, px, a, b, g]) initilaized to the setup values

  // From the spreadshit of the motor I know that the motors can do 0.17 s/60°
  // Consequently I will work considering a speed of: 0.15°/s as max speed
 

  bool pose_achieved = 0;

}


void move_joints(){
  //this should not work 
    for(int i = 0; i<5; i++){
        q.at(i) = move2(q.at(i),dq.at(i), duration);
    }
    EE = pinch(EE, EE_d);
    //send new increased positions of the joints and add a delay
    msg.position={(double)q.at(0), (double)q.at(1), (double)q.at(2), (double)q.at(3), (double)q.at(4), (double)EE};
    ros::Duration(delay).sleep();
    pub.publish(msg);
    ros::spinOnce();
}


float maxx(Joint_v_error& e){
  float ris = -10000;
  for(int i=0;i<5;i++){
    if (e.at(i)> ris){
      ris=e.at(i);
    }
  }
  return ris;
}

void plan_motion(){
  q_d = inverse_kinematics(r_d,q_0);
  dq = q_d - q_0;
  dEE = EE_d - EE;

  float q_max = maxx(dq);

  duration = q_max/max_velocity; //the duration for each complete movement is give by the highest angle at the max speed
  move_joints();
}

void desired_positionCallback(const geometry_msgs::Pose& msg){
  r_d.at(0) = msg.position.x;
  r_d.at(1) = msg.position.y;
  r_d.at(2) = msg.position.z;
  r_d.at(3) = msg.orientation.x;
  r_d.at(4) = msg.orientation.y;
  r_d.at(5) = msg.orientation.z;
  //EE_d = msg.orientation[3];    //I know that it should be the forth coordinate of the quaternion, but doesn't matter.
  plan_motion();
}


int main(int argc, char **argv){

  ros::init(argc, argv, "joint_pub");
  
  pub=n.advertise<sensor_msgs::JointState>("joint_states",1000);
  
  ros::Subscriber sub= n.subscribe("desired_pose",1000,desired_positionCallback);
  
  ros::Rate loop_rate(10);
  setup();
  while (ros::ok()){
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


    //loop_rate.sleep();

  

  return 0;
}