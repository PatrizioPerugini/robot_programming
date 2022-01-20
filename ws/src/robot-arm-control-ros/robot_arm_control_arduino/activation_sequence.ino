#include <Servo.h>


Servo robot_servos[6];
int servo_pins[6] = {2, 3, 5, 6, 7,9};
int initial_positions[6] = {90, 0, 0, 160, 180, 130};
int target_positions[6] = {90, 50, 115, 50, 90, 90};
int servo_CURRENT_positions[6];
bool off_set = 1;

void setup() {
  pinMode(13, OUTPUT);
  for (unsigned int i = 0; i < 6; i++) {
    robot_servos[i].attach(servo_pins[i]);
    robot_servos[i].write(initial_positions[i]);
    servo_CURRENT_positions[i] = initial_positions[i];
  }
}

void loop() {
  if(off_set){

    for (int pos = initial_positions[1]; pos <= target_positions[1]; pos++) {
      robot_servos[1].write(pos);
      servo_CURRENT_positions[1] = pos;
      digitalWrite(13, HIGH-digitalRead(13));
      delay(30);
    }
    delay(50);

    for (int pos = initial_positions[2]; pos <= target_positions[2]; pos++) {
      robot_servos[2].write(pos);
      servo_CURRENT_positions[2] = pos;
      digitalWrite(13, HIGH-digitalRead(13));
      delay(30);
    }
    delay(50);

    for (int pos = initial_positions[3]; pos >= target_positions[3]; pos--) {
      robot_servos[3].write(pos);
      servo_CURRENT_positions[3] = pos;
      digitalWrite(13, HIGH-digitalRead(13));
      delay(20);
    }
    delay(50);
    for (int pos = initial_positions[4]; pos >= target_positions[4]; pos--) {
      robot_servos[4].write(pos);
      servo_CURRENT_positions[4] = pos;
      digitalWrite(13, HIGH-digitalRead(13));
      delay(20);
    }
    delay(50);

    for (int pos = initial_positions[5]; pos >= target_positions[5]; pos--) {
      robot_servos[5].write(pos);
      servo_CURRENT_positions[5] = pos;
      digitalWrite(13, HIGH-digitalRead(13));
      delay(10);
    }
    delay(50);

    }
    for (int pos = initial_positions[0]; pos <= target_positions[0]; pos++) {
        robot_servos[0].write(pos);
        servo_CURRENT_positions[0] = pos;
        digitalWrite(13, HIGH-digitalRead(13));
        delay(10);
    }
    delay(50);
  off_set = 0;
}
  /*
  for (int joint3 = initial_positions[2]; joint3 <= target_positions[2]; i++) {
    robot_servos[2].write(joint3);
    servo_CURRENT_positions[2] = joint3;
    delay(10);
  }
  for (int joint4 = initial_positions[3]; joint4 <= target_positions[3]; i++) {
    robot_servos[2].write(joint4);
    servo_CURRENT_positions[2] = joint4;
    delay(10);
  }
  
*/
