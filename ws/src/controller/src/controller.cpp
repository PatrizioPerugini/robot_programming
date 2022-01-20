#include <ros/ros.h>
#include <iostream>
#include "keyboard.h"
using namespace std;

int main(int argc, char** argv)
{  
  ros::init(argc, argv, "keyboard");
  ros::NodeHandle n("~");

  ros::Publisher pub_down = n.advertise<controller::Key>("keydown", 10);
  ros::Publisher pub_up = n.advertise<controller::Key>("keyup", 10);

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
    if (new_event) {
      k.header.stamp = ros::Time::now();
      if (pressed) pub_down.publish(k);
      else pub_up.publish(k);
    }
    ros::spinOnce();
    r.sleep();
  }
  
  ros::waitForShutdown();
}