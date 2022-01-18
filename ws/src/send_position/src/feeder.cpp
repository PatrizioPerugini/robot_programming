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

    int cnt =1;
  /*for(int i=-90; i<=90; i+=10){
      msg.position={(double)i, 0, 0, 0, 0};
      msg.velocity={0.0};
      msg.effort={0.0};

      ros::Duration(2.0).sleep();
      pub.publish(msg);
    }
  */
  
  while (ros::ok()){
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

  
 
   // msg.position={90.0, 0, 0, 0, 0};
   // msg.velocity={0.0};
   // msg.effort={0.0};
   // pub.publish(msg);
  //
   // 
   // ros::Duration(0.5).sleep();
   // 
   // msg.position={-90.0, 0, 0, 0, 0};
   // msg.velocity={0.0};
   // msg.effort={0.0};
   // pub.publish(msg);
  
    ros::Duration(1.0).sleep();

    ros::spinOnce();

    //loop_rate.sleep();
  
  }

  return 0;
}