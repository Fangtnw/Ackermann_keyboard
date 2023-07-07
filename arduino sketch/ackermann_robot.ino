#include <Servo.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>

const int motor1_pin1 = 5;
const int motor1_pin2 = 4;
const int pwm1 = 3;
const int motor2_pin1 = 8;
const int motor2_pin2 = 7;
const int pwm2 = 6;
const int servo_pin = 9;
bool key_pressed = false;
const int servo_max = 180;
const int servo_min = 0;
int motor_spd = 30;
int motor_state=0;
int servo_pos;

const unsigned long print_interval = 1000; // 1 second
unsigned long previous_print_time = 0;

Servo servo;
ros::NodeHandle nh;
std_msgs::String combined_msg;
ros::Publisher combined_pub("combined", &combined_msg);

void cmd_vel_callback(const geometry_msgs::Twist& twist_msg) {
  int motor_state = twist_msg.linear.x;
  int motor_spd = twist_msg.linear.y;
  int servo_pos = twist_msg.angular.z;
  unsigned long current_time = millis();

  if (current_time - previous_print_time >= print_interval) {
    previous_print_time = current_time;

    // Create the combined message
    char combined_buffer[50];
    snprintf(combined_buffer, sizeof(combined_buffer), "motor spd: %d servo_pos: %d", motor_spd, servo_pos);
    combined_msg.data = combined_buffer;

    // Publish the combined message
    combined_pub.publish(&combined_msg);
  }
  // Set the motor directions and speeds
  if (motor_state == 1) {
    analogWrite(pwm1, motor_spd);
    digitalWrite(motor1_pin1, HIGH);
    digitalWrite(motor1_pin2, LOW);
    analogWrite(pwm2, motor_spd+6);
    digitalWrite(motor2_pin1, HIGH);
    digitalWrite(motor2_pin2, LOW);
    key_pressed = true;
  } else if (motor_state == -1) {
    analogWrite(pwm1, motor_spd);
    digitalWrite(motor1_pin1, LOW);
    digitalWrite(motor1_pin2, HIGH);
    analogWrite(pwm2, motor_spd+6);
    digitalWrite(motor2_pin1, LOW);
    digitalWrite(motor2_pin2, HIGH);
    key_pressed = true;
  } else if (motor_state == 0) {
    analogWrite(pwm1, 0);
    digitalWrite(motor1_pin1, HIGH);
    digitalWrite(motor1_pin2, HIGH);
    analogWrite(pwm2, 0);
    digitalWrite(motor2_pin1, HIGH);
    digitalWrite(motor2_pin2, HIGH);
    key_pressed = false;
  }

  servo.write(servo_pos);
}

ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", &cmd_vel_callback);


void setup() {
  pinMode(motor1_pin1, OUTPUT);
  pinMode(motor1_pin2, OUTPUT);
  pinMode(motor2_pin1, OUTPUT);
  pinMode(motor2_pin2, OUTPUT);
  pinMode(pwm1, OUTPUT);
  pinMode(pwm2, OUTPUT);
  servo.attach(servo_pin);

  nh.initNode();
  nh.subscribe(cmd_vel_sub);
  nh.advertise(combined_pub);
}

void loop() {
  nh.spinOnce();
  delay(5);
  
}
