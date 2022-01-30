#include <ros/ros.h>
#include <iostream>
#include "keyboard.h"
#include <geometry_msgs/Pose.h>
using namespace std;




bool significant(int k){
  return (k==273 || k==274 || k==275 || k==276 || k==280 || k==281 || k==112 || 
          k==111 ||k==119 || k== 97 || k==101 || k==115 || k==114 || k==100);
          
}


int main(int argc, char** argv)
{  
  ros::init(argc, argv, "keyboard");
  ros::NodeHandle n("~");

  
  ros::Publisher pub_down = n.advertise<controller::Key>("keydown", 10);
  ros::Publisher pub_up = n.advertise<controller::Key>("keyup", 10);
  ros::Publisher pub_new_pose= n.advertise<geometry_msgs::Pose>("desired_pose",1000);

  bool allow_repeat=false;
  int repeat_delay, repeat_interval;
  
  n.param<bool>( "allow_repeat", allow_repeat, false ); // disable by default
  n.param<int>( "repeat_delay", repeat_delay, SDL_DEFAULT_REPEAT_DELAY );
  n.param<int>( "repeat_interval", repeat_interval, SDL_DEFAULT_REPEAT_INTERVAL );
  
  if ( !allow_repeat ) repeat_delay=0; // disable 
  keyboard::Keyboard kbd( repeat_delay, repeat_interval );
  
  ros::Rate r(50);
  /*
  TASTO   CODICE  TRAD
  up      273     +z
  down    274     -z
  right   275     +x
  left    276     -x
  pagup   280     +y
  pagdow  281     -y
  p       112     +w
  o       111     -w
  w       119     +a
  a       97      -a
  e       101     +b
  s       115     -b
  r       114     +g
  d       100     -g

*/


  controller::Key k;
  bool pressed, new_event;
  while (ros::ok() && kbd.get_key(new_event, pressed, k.code, k.modifiers)) {
    if (new_event && significant(k.code)) {
      //cout<<k.code<<endl;
      //switch case for the message that needs to be send
      if (pressed){
        int pressed_code = k.code;
        geometry_msgs::Pose msg;
        msg.position.x   =0; 
        msg.position.y   =0; 
        msg.position.z   =0; 
        msg.orientation.x=0; 
        msg.orientation.y=0; 
        msg.orientation.z=0;
        msg.orientation.w=0; 

        switch (pressed_code){
          case(273):
              msg.position.z=0.3;
          break;
          case(274):
              msg.position.z=-1;
          break;
          case(275):
              msg.position.x=1;
          break;
          case(276):
              msg.position.x=-1;
          break;
          case(280):
              msg.position.y=1;
          break;
          case(281):
              msg.position.y=-1;
          break;
          case(112):
              msg.orientation.w=0.3;
          break;
          case(111):
              msg.orientation.w=-0.3;
          break;
          case(119):
              msg.orientation.x=0.3;
          break;
          case(97):
              msg.orientation.x=-0.3;
          break;
          case(101):
              msg.orientation.y=0.3;
          break;
          case(115):
              msg.orientation.y=-0.3;
          break;
          case(114):
              msg.orientation.z=0.3;
          break;
          case(100):
              msg.orientation.z=-0.3;
          break;
        }
        pub_new_pose.publish(msg);



        pub_down.publish(k);
      }
      else pub_up.publish(k);
    }
    ros::spinOnce();
    r.sleep();
  }
  
  ros::waitForShutdown();
}