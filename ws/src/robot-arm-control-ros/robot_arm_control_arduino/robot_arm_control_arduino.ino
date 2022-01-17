#include <Servo.h>
#include <ros.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/JointState.h>

ros::NodeHandle node_handle;

Servo robot_servos[5];
int servo_pins[6] = {2, 3, 5, 6, 7, 9}; // PWM Pins 

int mid_positions[5] = {90, 0, 0, 160, 180, 150};
int servo_CURRENT_positions[6];

float servo_TARGET_position[6] = {0,0,0,0,0,0};

// Convert the joint state values to degrees, adjust for the center and write to the servo
void writeServos() {
  for (int j = 0; j < 5; j++) {
    int target_angle;
    target_angle = servo_TARGET_position[j]*(180/3.14) + mid_positions[j];
    robot_servos[j].write(target_angle);
    servo_CURRENT_positions[j] = target_angle;
  }
  node_handle.spinOnce();
}

// Subscriber Callback to store the jointstate position values in the global variables
void servoControlSubscriberCallbackJointState(const sensor_msgs::JointState& msg) {
  servo_TARGET_position[0] = msg.position[0];
  servo_TARGET_position[1] = msg.position[1];
  servo_TARGET_position[2] = msg.position[2];
  servo_TARGET_position[3] = msg.position[3];
  servo_TARGET_position[4] = msg.position[4];
  servo_TARGET_position
  // Call the method to write the joint positions to the servo motors
  writeServos();

}

ros::Subscriber<sensor_msgs::JointState> servo_control_subscriber_joint_state("joint_states", &servoControlSubscriberCallbackJointState);


void setup() {
  // Initial the servo motor connections and initialize them at home position
  for (unsigned int i = 0; i < 5; i++) {
    robot_servos[i].attach(servo_pins[i]);
    robot_servos[i].write(mid_positions[i]);
    servo_CURRENT_positions[i] = mid_positions[i];
  }

  // Set the communication BaudRate and start the node
  node_handle.getHardware()->setBaud(19000);
  node_handle.initNode();
  node_handle.subscribe(servo_control_subscriber_joint_state);
}

void loop() {
  // Keep calling the spinOnce() method in this infinite loop to stay tightly coupled with the ROS Serial
  node_handle.spinOnce();
  delay(1);
}
