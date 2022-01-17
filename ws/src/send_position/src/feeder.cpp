#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/JointState.h>
#include <sstream>

int main(int argc, char **argv)
{

  ros::init(argc, argv, "joint_pub");

  ros::NodeHandle n;

  //n.reset(new ros::NodeHandle(config->body()));
  ros::Publisher pub=n.advertise<sensor_msgs::JointState>("joint_states",1000);


  //ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  ros::Rate loop_rate(10);

   sensor_msgs::JointState msg;
  // msg.position[5];
   //msg.velocity[5];
   //msg.effort[5];


  int count = 0;
  while (ros::ok())
  {

   msg.position={0.2, 0, 0, 0, 0};
  
    msg.velocity={0.2, 0.0, 0.0, 0.0, 0.0};
   // msg.velocity[1]=0.0;
   // msg.velocity[2]=0.0;
   // msg.velocity[3]=0.0;
   // msg.velocity[4]=0.0;
//
   msg.effort={1.0, 0.0, 0.0, 0.0, 0.0};
   // msg.effort[1]=0.0;
   // msg.effort[2]=0.0;
   // msg.effort[3]=0.0;
   // msg.effort[4]=0.0;

//  position: [2.0, 0.0, 0.0, 0.0, 0.0]
//velocity: [10.0, 0.0, 0.0, 0.0, 0.0]



 

//    std::stringstream ss;
  //  ss << "is it publishing? " << count;
   

   // ROS_INFO("%f", msg.position[0]);
    //ROS_INFO("%f", msg.velocity[1]);

   
    pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  return 0;
}