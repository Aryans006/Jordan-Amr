#include <micro_ros_arduino.h>
#include <ESP32Servo.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <CytronMotorDriver.h>
#include <geometry_msgs/msg/twist.h>

rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
/*******************************
 * ESP D3  - Motor Driver PWM 1 Input
 * ESP D4  - Motor Driver DIR 1 Input
 * ESP D9  - Motor Driver PWM 2 Input
 * ESP D10 - Motor Driver DIR 2 Input
 * ESP GND - Motor Driver GND
******************************/

#define motor1 13  // PWM 1 = Pin 13, DIR 1 = Pin 14.
#define motor2 12 // PWM 2 = Pin 12, DIR 2 = Pin 27.

#define DIR1 14
#define DIR2 27
#define LED_PIN 13

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

//twist message cb
void subscription_callback(const void *msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  // if velocity in x direction is 0 turn off LED, if 1 turn on LED
  //digitalWrite(LED_PIN, (msg->linear.x == 0) ? LOW : HIGH);
}

void setup() {
  set_microros_transports();
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  

  pinMode(motor1, OUTPUT);
  pinMode(motor2, OUTPUT);
  pinMode(DIR1, OUTPUT);
  pinMode(DIR2, OUTPUT);
  
  delay(2000);

  allocator = rcl_get_default_allocator();

   //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/cmd_vel_esp32"));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));

}

void loop() {
  delay(1000);
  Serial.print("Right-Velocity: ");
  analogWrite(motor1, (floor(msg.linear.x))/2);
  digitalWrite(DIR1, HIGH);
  Serial.println(floor(msg.linear.x)); 

  Serial.print("Left-Velocity: ");
  analogWrite(motor2, (floor(msg.linear.y))/2);
  digitalWrite(DIR2, HIGH);
  Serial.println(floor(msg.linear.y)); 

  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}