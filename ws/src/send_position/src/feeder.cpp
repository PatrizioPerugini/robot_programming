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
Pose_v p;
Joint_v q;
Joint_v q_init;

float EE_init = 120.0;
Joint_v q_act;

float EE_act = 90;
// From the spreadshit of the motor I know that the motors can do 0.17 s/60°
// Consequently I will work considering a speed of: 0.15°/s as max speed
const float max_velocity = 0.15;
float delay = 0.01;    // frequence of the messages to arduino 
float duration;
int pose_achieved;
ros::Publisher pub;

int i = 0;



int check_for_pose(Joint_v& dq, float dEE){
    if(
        abs(dq.at(0)) <= 0.3 &&
        abs(dq.at(1)) <= 0.3 &&
        abs(dq.at(2)) <= 0.3 &&
        abs(dq.at(3)) <= 0.3 &&
        abs(dq.at(4)) <= 0.3 &&
        abs(dEE) <= 0.3
    ){
        return 1;
    }
    else{
        return 0;
    } 
}

void move_joints(){
  while(pose_achieved == 0){
    for(int i = 0; i<5; i++){
      q.at(i) = move2(q.at(i),dq.at(i), duration);
      EE = pinch(EE, EE_d);
    }

    msg.position={(double)q.at(0), (double)q.at(1), (double)q.at(2),
     (double)q.at(3), (double)q.at(4), (double)EE};
    ros::Duration(delay).sleep();
    pub.publish(msg);
    ros::spinOnce();

    pose_achieved = check_for_pose(dq,dEE);
  }
    
    
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
  q_d = inverse_kinematics(r_d,q);
  dq = q_d - q;
  dEE = EE_d - EE;

  float q_max = maxx(dq);

  duration = q_max/max_velocity; //the duration for each complete movement is give by the highest angle at the max speed
  move_joints();
}

void activation(){
  q_d = q_act;
  dq = q_d - q;
  EE_d = EE_act;
  dEE = EE_d - EE;

  float q_max = maxx(dq);

  duration = q_max/max_velocity; //the duration for each complete movement is give by the highest angle at the max speed
  move_joints();
}


void deactivation(){
  // command from keyboard to define
  activation();
  q_d = q_init;
  dq = q_d - q;
  EE_d = EE_init;
  dEE = EE_d - EE;

  float q_max = maxx(dq);

  duration = q_max/max_velocity; //the duration for each complete movement is give by the highest angle at the max speed
  move_joints();

}

void desired_positionCallback(const geometry_msgs::Pose& msg){
  if(pose_achieved == 1){
    r_d.at(0) = msg.position.x    + p.at(0);
    r_d.at(1) = msg.position.y    + p.at(1);
    r_d.at(2) = msg.position.z    + p.at(2);
    r_d.at(3) = msg.orientation.x + p.at(3);
    r_d.at(4) = msg.orientation.y + p.at(4);
    r_d.at(5) = msg.orientation.z + p.at(5);
    EE_d = msg.orientation.w + EE;    //I know that it should be the forth coordinate of the quaternion, but doesn't matter.
    pose_achieved == 0;
    plan_motion();
  }
}


int main(int argc, char **argv){
  
  //init nodes and topics
  ros::init(argc, argv, "joint_pub");
  ros::NodeHandle n;
  pub=n.advertise<sensor_msgs::JointState>("joint_states",1000);
  ros::Subscriber sub= n.subscribe("/keyboard/desired_pose",1000,desired_positionCallback);
  ros::Rate loop_rate(10);

  //Setup
  q = q_init;
  EE = EE_init;
  q_init.at(0) = 90.0;
  q_init.at(1) = 0.0;
  q_init.at(2) = 0.0;
  q_init.at(3) = 150.0;
  q_init.at(4) = 180.0; 
  

  q_act.at(0) = 90.0;
  q_act.at(1) = 50.0;
  q_act.at(2) = 115.0;
  q_act.at(3) = 50.0;
  q_act.at(4) = 90.0; 
  //activation procedure
  activation();
  
  
  bool pose_achieved = 1;

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