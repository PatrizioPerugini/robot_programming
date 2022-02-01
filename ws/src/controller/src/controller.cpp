#include <ros/ros.h>
#include <iostream>
#include "keyboard.h"
#include <geometry_msgs/Pose.h>
#include "std_msgs/String.h"
using namespace std;




bool significant(int k){
  return (k==273 || k==274 || k==275 || k==276 || k==280 || k==281 || k==112 || 
          k==111 ||k==119 || k== 97 || k==101 || k==115 || k==114 || k==100 || k==104 || k==106);
          
}


int main(int argc, char** argv)
{  
  ros::init(argc, argv, "keyboard");
  ros::NodeHandle n("~");

  
  ros::Publisher pub_down = n.advertise<controller::Key>("keydown", 10);
  ros::Publisher pub_up = n.advertise<controller::Key>("keyup", 10);
  ros::Publisher pub_new_pose= n.advertise<geometry_msgs::Pose>("desired_pose",1000);
  ros::Publisher trajectory = n.advertise<std_msgs::String>("trajectory", 1000);
  bool allow_repeat=false;
  int repeat_delay, repeat_interval;
  
  n.param<bool>( "allow_repeat", allow_repeat, false ); // disable by default
  n.param<int>( "repeat_delay", repeat_delay, SDL_DEFAULT_REPEAT_DELAY );
  n.param<int>( "repeat_interval", repeat_interval, SDL_DEFAULT_REPEAT_INTERVAL );

  float delta = 1;
  
  if ( !allow_repeat ) repeat_delay=0; // disable 
  keyboard::Keyboard kbd( repeat_delay, repeat_interval );
  
  ros::Rate r(50);
  /*
  TASTO   CODICE  TRAD
  up      273     +z
  down    274     -z
  right   275     +x
  left    276     -x
  pagup   280     +y (-)
  pagdow  281     -y (+)
  p       112     +w
  o       111     -w
  w       119     +a
  a       97      -a
  e       101     +b
  s       115     -b
  r       114     +g
  d       100     -g
  h       104      deactivation
  j       106      activation  
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
        std_msgs::String msg_trajectory;
        
        bool is_deactivated=false;

        

        switch (pressed_code){
          case(273):
              msg.position.z=0.5*delta;
              break;
          case(274):
              msg.position.z=-0.5*delta;
              break;
          case(275):
              msg.position.x=0.25*delta;
              break;
          case(276):
              msg.position.x=-0.25*delta;
              break;
          case(281):
              msg.position.y=0.5*delta;
              break;
          case(280):
              msg.position.y=-0.5*delta;
              break;
          case(112):
              msg.orientation.w=3*delta;
             
              break;
          case(111):
              msg.orientation.w=-3*delta;
              
              break;
          case(119):
              msg.orientation.x=3*delta;
              break;
          case(97):
              msg.orientation.x=-3*delta;
              break;
          case(101):
              msg.orientation.y=3*delta;
              break;
          case(115):
              msg.orientation.y=-3*delta;
              break;
          case(114):
              msg.orientation.z=3*delta;
              break;
          case(100):
              msg.orientation.z=-3*delta;
              break;
          case(104):
              is_deactivated=true;
              msg_trajectory.data="h";
              break;
          case(106):
              is_deactivated=true;
              msg_trajectory.data="j";
              break;
        
        }
        if(is_deactivated){
            cout<<"planning trajectory"<<endl;
            trajectory.publish(msg_trajectory);
        }
        else{
            pub_new_pose.publish(msg);
        }
            pub_down.publish(k);
        
      
      }
      else pub_up.publish(k);
    }

    

    ros::spinOnce();
    r.sleep();
  }
  
  ros::waitForShutdown();
}

/*
x: 0.0
  y: 2.97638
  z: 0.0296235
orientation:
  x: -2.04203
  y: 3.14159
  z: 3.14159
  w: 0.0"  
  */