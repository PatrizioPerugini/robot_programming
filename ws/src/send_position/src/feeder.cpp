#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/JointState.h>
#include <sstream>


int main(int argc, char **argv)
{

  ros::init(argc, argv, "joint_pub");

  ros::NodeHandle n;


  ros::Publisher pub=n.advertise<sensor_msgs::JointState>("joint_states",1000);

  ros::Rate loop_rate(10);

  sensor_msgs::JointState msg;

  
  while (ros::ok()){
    for(int i=-90; i<90; i+=5){
      msg.position={(double)i, 0, 0, 0, 0};
      msg.velocity={0.2, 0.0, 0.0, 0.0, 0.0};
      msg.effort={1.0, 0.0, 0.0, 0.0, 0.0};

      ros::Duration(0.8).sleep();
      pub.publish(msg);
    }
  
    pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  
  }

  return 0;
}