
#include <ros.h>
#include <ArduinoHardware.h>
//#include <ArduinoTcpHardware.h>

#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif



#include <Servo.h>
#include <ros.h>
#include <sensor_msgs/JointState.h>

ros::NodeHandle node_handle;

Servo robot_servos[6];
int servo_pins[6] = {2, 3, 5, 6, 7,9}; // PWM Pins 

int mid_positions[6] = {90, 0, 0, 160, 180, 130};
int servo_CURRENT_positions[6];

float servo_TARGET_position[6] = {0,0,0,0,0,0};

// Convert the joint state values to degrees, adjust for the center and write to the servo
void writeServos() {
  for (int j = 0; j < 6; j++) {
    int target_angle;
    target_angle = servo_TARGET_position[j]; 
    //+ mid_positions[j];
    robot_servos[j].write(target_angle);
    servo_CURRENT_positions[j] = target_angle;
  }
  
}


// Subscriber Callback to store the jointstate position values in the global variables
void servoControlSubscriberCallbackJointState(const sensor_msgs::JointState& msg) {
  servo_TARGET_position[0] = msg.position[0];
  servo_TARGET_position[1] = msg.position[1] * 180/270;
  servo_TARGET_position[2] = msg.position[2] * 180/270;
  servo_TARGET_position[3] = msg.position[3];
  servo_TARGET_position[4] = msg.position[4];
  servo_TARGET_position[5] = msg.position[5];
  digitalWrite(13, HIGH-digitalRead(13));

  //servo_TARGET_position
  // Call the method to write the joint positions to the servo motors
  writeServos();

}

ros::Subscriber<sensor_msgs::JointState> servo_control_subscriber_joint_state("joint_states", &servoControlSubscriberCallbackJointState);

void setup() {
  // Initial the servo motor connections and initialize them at home position
  pinMode(13, OUTPUT);
  for (unsigned int i = 0; i < 6; i++) {
    robot_servos[i].attach(servo_pins[i]);
    robot_servos[i].write(mid_positions[i]);
    servo_CURRENT_positions[i] = mid_positions[i];
  }

  // Set the communication BaudRate and start the node
  node_handle.getHardware()->setBaud(57600);//not sure maybe lower
  node_handle.initNode();
  node_handle.subscribe(servo_control_subscriber_joint_state);
}

void loop() {
  // Keep calling the spinOnce() method in this infinite loop to stay tightly coupled with the ROS Serial
  node_handle.spinOnce();
  delay(1);
}

//rosrun rosserial_python serial_node.py /dev/ttyACM0
