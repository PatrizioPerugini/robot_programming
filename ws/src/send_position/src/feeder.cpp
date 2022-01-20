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

Joint_v q_init;
///////////////
//pose of ee, initially with initial conf 
Pose_v p;
//joint conf, initially with initial conf 
Joint_v q;



float EE_init = 120.0;
Joint_v q_act;

float EE_act = 90;
// From the spreadshit of the motor I know that the motors can do 0.17 s/60°
// Consequently I will work considering a speed of: 357°/s as max speed

//const float max_velocity = 0.15;
float max_velocity = 357.7;
const float velocity = 100.7;
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
  cout << "pose achieved in move_joints: "<< pose_achieved  << endl;
  while(pose_achieved == 0){
    for(int i = 0; i<5; i++){
      if(abs(dq.at(i))>0.3){
        float ddq_i =(dq.at(i)*delay)/duration;
        q.at(i) += ddq_i;
        dq.at(i)-=ddq_i;
       // cout << "variazione joint " << i << "is " << dq.at(i) << endl;
      }
    }
    cout << "I'm going to go to these values: \n" << endl;
    cout << q << endl;
    EE = pinch(EE, EE_d);
    msg.position={(double)q.at(0), (double)q.at(1), (double)q.at(2),
     (double)q.at(3), (double)q.at(4), (double)EE};
    ros::Duration(delay).sleep();
    pub.publish(msg);
 //   ros::spinOnce();

    pose_achieved = check_for_pose(dq,dEE);
  }
    
    
}


float maxx(Joint_v_error& e){
  float ris = -10000;
  for(int i=0;i<5;i++){
    if (ris < abs(e.at(i))){
      ris=e.at(i);
    }
  }
  return ris;
}

void plan_motion(){


  //q_d = inverse_kinematics(r_d,q);

  q_d.at(0) = 90.0;
  q_d.at(1) = 50.0;
  q_d.at(2) = 115.0;
  q_d.at(3) = 50.0;
  q_d.at(4) = 90.0; 
  dEE = 90;

  cout << "I found some inverse solution, values: \n" << endl;
  cout << q_d << endl;
  //vector of single joints difference
  dq = q_d - q;

  dEE = EE_d - EE;


  float dq_max = abs(maxx(dq));
  
  duration = dq_max/velocity; //the duration for each complete movement is give by the highest angle at the max speed
  cout << "the duration of the motion is "<<duration <<endl;
  
  move_joints();
}
/*
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
*/

void desired_positionCallback(const geometry_msgs::Pose& msg){

 
  cout << pose_achieved << endl;
  if(pose_achieved == 1){
    cout << "hey I heard something" << endl;    
    r_d.at(0) = msg.position.x    + p.at(0);
    r_d.at(1) = msg.position.y    + p.at(1);
    r_d.at(2) = msg.position.z    + p.at(2);
    r_d.at(3) = msg.orientation.x + p.at(3);
    r_d.at(4) = msg.orientation.y + p.at(4);
    r_d.at(5) = msg.orientation.z + p.at(5);
    EE_d = msg.orientation.w + EE;    //I know that it should be the forth coordinate of the quaternion, but doesn't matter.
    pose_achieved = 0;
    plan_motion();

    cout << "I finished planning" << endl;
  }
}


int main(int argc, char **argv){

  pose_achieved = 1;
  EE = EE_init;
  q_init.at(0) = 90.0;
  q_init.at(1) = 0.0;
  q_init.at(2) = 0.0;
  q_init.at(3) = 160.0;
  q_init.at(4) = 180.0;
  
  q = q_init;

  p=get_pose(q_init);

  
  //init nodes and topics
  ros::init(argc, argv, "joint_pub");
  ros::NodeHandle n;
  pub=n.advertise<sensor_msgs::JointState>("joint_states",1000);
  ros::Subscriber sub= n.subscribe("/keyboard/desired_pose",1000,desired_positionCallback);
  ros::Rate loop_rate(10);

  //Setup


 

/*
  q_act.at(0) = 90.0;
  q_act.at(1) = 50.0;
  q_act.at(2) = 115.0;
  q_act.at(3) = 50.0;
  q_act.at(4) = 90.0; 
  //activation procedure
  activation();
  */
  
 

  while (ros::ok()){
      ros::spinOnce();
      //don't know
      loop_rate.sleep();
  }

  return 0;
}