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
#include "std_msgs/String.h"
#include <geometry_msgs/Pose.h>


sensor_msgs::JointState msg;
Pose_v r_d; //desired position
Joint_v q_d; //desired joint conf
Joint_v_error dq;  //error: qd - q
float EE_d;
float EE;
float dEE;
float EE_act;
float EE_init;
float EE_constaints[2] ={80,130};

Pose_v p; //pose of ee, initially with initial conf 
Pose_v p_init; //initial pose, calculated the first time with matlab 
Joint_v q; //joint conf, initially with initial conf 
Joint_v q_init;
Joint_v q_act;
float q_constraints[5][2] = {{10*(M_PI/180),170*(M_PI/180)},
{0,130*(M_PI/180)},
{0,270*(M_PI/180)},
{0,180*(M_PI/180)},
{0,180*(M_PI/180)}} ;

// From the spreadshit of the motor I know that the motors can do 0.17 s/60°
// Consequently I will work considering a speed of: 357°/s as max speed
const float max_velocity = 200.7;
const float velocity = 200.7;
const float delay = 0.05;    // frequence of the messages to arduino 
float duration;
int pose_achieved;
ros::Publisher pub;

int i = 0;



int check_for_pose(Joint_v& dq, float dEE){
    if(
        abs(dq(0)) <= 0.3 &&
        abs(dq(1)) <= 0.3 &&
        abs(dq(2)) <= 0.3 &&
        abs(dq(3)) <= 0.3 &&
        abs(dq(4)) <= 0.3 &&
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
  int cnt=0;
  while(pose_achieved == 0){
    for(int i = 0; i<5; i++){
      if(abs(dq(i))>0.3){
        float ddq_i =(dq(i)*delay)/duration;
       
        q(i) += ddq_i;
        dq(i)-=ddq_i; 

       // cout << "variazione joint " << i << "is " << dq.at(i) << endl;
      }
      
    }
    if(abs(dEE)>=0.3){
      if(dEE>0){
        EE += 0.3;
        dEE -= 0.3;
      }
      else{
        EE -=0.3;
        dEE += 0.3;
      }
    }
    
    cout << "I'm going to go to these values: q is " << endl;
    cout << q << endl;
    cout << "I'm going to go to these values: EE is " << endl;
    cout << EE << endl;
    cout<<"\n"<<endl;

    
    msg.position={(double)q(0) * (180/M_PI), (double)q(1)* (180/M_PI), (double)q(2)* (180/M_PI),
     (double)q(3) * (180/M_PI), (double)q(4) * (180/M_PI), (double)EE};
    ros::Duration(delay).sleep();
    pub.publish(msg);

    p = get_pose(q); //here we pass p in order for the RPY to choose the best sol
    cnt++;

    pose_achieved = check_for_pose(dq,dEE);
  }
  
  cout << "finished movement , pose_achieved : "<< pose_achieved << endl;
   
}


float maxx(Joint_v_error& e){

 // cout << "dq in MAXXXX IS " << e <<endl;
  float ris = -10000;
  for(int i=0;i<5;i++){
    if (ris < abs(e(i))){
      ris=e(i);
    }
  }
  
  return ris;
}

void plan_motion(){
  q_d = inverse_kinematics(r_d,q);
  //q_d=q_act;
  cout << "I found some inverse solution, values: \n" << endl;
  cout << q_d << endl;

  
  for(int i=0; i<5;i++){
      if(q(i)<q_constraints[i][0]){      //CHECK low constraint
          q(i)=q_constraints[i][0];
      }
      if(q(i)>q_constraints[i][1]){  //CHECK high constarint
          q(i)=q_constraints[i][1];
      }
  }
  
  //vector of single joints difference
  dq = q_d - q;

  dEE = EE_d - EE;

 

  float dq_max = abs(maxx(dq));
  
  duration = (dq_max*180/M_PI)/velocity; //the duration for each complete movement is give by the highest angle at the max speed
  
  
  
  move_joints();
}


void activation(){
  q_d = q_act;
  dq = q_d - q;
  EE_d = EE_act;
  dEE = EE_d - EE;
  pose_achieved=0;
 
  float q_max = abs(maxx(dq));
  

  duration = (q_max*180/M_PI)/max_velocity; //the duration for each complete movement is give by the highest angle at the max speed
  
  move_joints();
}


void deactivation(){
  cout<<"starting deactivation " <<endl;
  
  activation();
  

  cout<<"in position for deactivation " <<endl;

  q_d = q_init;
  dq = q_d - q;
  EE_d = EE_init;
  dEE = EE_d - EE;
  pose_achieved=0;

  float q_max = abs(maxx(dq));

  duration = (q_max*180/M_PI)/max_velocity; //the duration for each complete movement is give by the highest angle at the max speed
  move_joints();

   cout<<"end OF DISCUSSION " <<endl;


}

void trajectory_Callback(const std_msgs::String& msg ){
    cout << "deactivation sequence initiated" << endl;
    if(msg.data=="h"){
        deactivation();
    }
    else{
        activation();
    }
    
}

void desired_positionCallback(const geometry_msgs::Pose& msg){

  cout << pose_achieved << endl;
  if(pose_achieved == 1){
     
    r_d(0) = msg.position.x    + p(0);
    r_d(1) = msg.position.y    + p(1);
    r_d(2) = msg.position.z    + p(2);
    r_d(3) = msg.orientation.x * (M_PI/180) + p(3);
    r_d(4) = msg.orientation.y * (M_PI/180)+ p(4);
    r_d(5) = msg.orientation.z * (M_PI/180)+ p(5);
    
    EE_d = msg.orientation.w + EE;    //I know that it should be the forth coordinate of the quaternion, but doesn't matter.
    if(EE_d < EE_constaints[0]){      //CHECK low constraint of EE
        EE_d=EE_constaints[0];
    }
    if(EE_d>EE_constaints[1]){  //CHECK high constarint of EE
        EE_d=EE_constaints[1];
    }
    pose_achieved = 0;
    plan_motion();

    cout << "I finished planning" << endl;
  }
}


int main(int argc, char **argv){
  
  //Initial Setup
  pose_achieved = 0;
  
  q_init(0) = 90.0 * (M_PI/180);
  q_init(1) = 0.0* (M_PI/180);
  q_init(2) = 0.0 * (M_PI/180);
  q_init(3) = 160.0 * (M_PI/180);
  q_init(4) = 180.0 * (M_PI/180);
  EE_init = 120.0 ;



  q = q_init;
  EE = EE_init;
  cout<<"la qu dal mainnn \n" << q << "\n"<< endl;
  p=get_pose(q_init);
  
  //init nodes and topics
  ros::init(argc, argv, "joint_pub");
  ros::NodeHandle n;
  pub=n.advertise<sensor_msgs::JointState>("joint_states",1000);
  ros::Subscriber sub= n.subscribe("/keyboard/desired_pose",1000,desired_positionCallback);
  ros::Subscriber subb=n.subscribe("/keyboard/trajectory",1000,trajectory_Callback);
  ros::Rate loop_rate(10);


  
  //get to the best position to start the motion ==> activation
  q_act(0) = 90.0 * (M_PI/180);
  q_act(1) = 90.0 * (M_PI/180);
  q_act(2) = 200 * (M_PI/180);
  q_act(3) = 60.0 * (M_PI/180);
  q_act(4) = 90.0 * (M_PI/180); 
  EE_act = 100 ;
  
  //activation procedure
  activation();
  
  

  while (ros::ok()){
      ros::spinOnce();
      //don't know
      loop_rate.sleep();
  }

  return 0;
}