#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/JointState.h>
#include <sstream>
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
using Pose_v=Vec_<float, 6>;
using DH_row = float[4];
using DH_table = float[5][4];
using Error_v =Vec_<float,6>;

ros::init(argc, argv, "joint_pub");

ros::NodeHandle n;

ros::Publisher pub=n.advertise<sensor_msgs::JointState>("joint_states",1000);

ros::Subscriber sub= n.subscribe<geometry_msgs::Pose>("desired_pose",1000,desired_positionCallback);

ros::Rate loop_rate(10);

sensor_msgs::JointState msg;

void setup(){
  
  int i = 0;
  
  //initial configuaration (setup values)
  Joint_v q;
  q.at(0) = 90.0;
  q.at(1) = 0.0;
  q.at(2) = 0.0;
  q.at(3) = 150.0;
  q.at(4) = 180.0; 
  
  Pose_v p = get_pose(q_0);   //Current pose (p = [px, py, px, a, b, g]) initilaized to the setup values
  Pose_v r_d; //desired position
  Joint_v q_d; //desired joint conf
  Joint_v_error dq;  //error: qd - q
  float EE_d;
  float EE;
  float dEE;
   
  // From the spreadshit of the motor I know that the motors can do 0.17 s/60°
  // Consequently I will work considering a speed of: 0.15°/s as max speed
  const float max_velocity = 0.15;
  float delay = 0.01;    // frequence of the messages to arduino

  bool pose_achieved = 0;

}

void desired_positionCallback(const geometry_msgs::Pose& msg){
  r_d[0] = msg.position[0];
  r_d[1] = msg.position[1];
  r_d[2] = msg.position[2];
  r_d[3] = msg.orientation[0];
  r_d[4] = msg.orientation[1];
  r_d[5] = msg.orientation[2];
  EE_d = msg.orientation[3];    //I know that it should be the forth coordinate of the quaternion, but doesn't matter.
  plan_motion();
}

void plan_motion(){
  q_d = inverse_kinematics(r_d);
  dq = q_d - q;
  dEE = EE_d - EE;
  q_max = max(dq); 
  time = q_max/max_velocity; //the time for each complete movement is give by the highest angle at the max speed
  move_joints();
}


void move_joints(){
    for(int i = 0; i<5; i++){
        q.at(i) = move2(q.at(i),dq.at(i), time);
    }
    EE = pinch(EE, EE_d);
    //send new increased positions of the joints and add a delay
    msg.position={(double)q.at(0), (double)q.at(1), (double)q.at(2), (double)q.at(3), (double)q.at(4), (double)EE};
    ros::Duration(delay).sleep();
    pub.publish(msg);
    n.spinOnce();
}


int main(int argc, char **argv){
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