/*
This is a subscriber microros that allow you to control a motor by the h_bridge.h 
library using micro_ros_platformio with raspberry PI and Arduino framework.

Steps:

  Edit build_src_filter in platformio.ini to build this example
    build_src_filter = +<examples/sub_motor/*> +<h_bridge/*> -<.git/> -<.svn/> 
  
  Compile and upload to raspberry PI, use the firmware.uf2 that is inside the .pio/build/pico/ folder
  
  Terminal1:
    # Find your serial [device name]:
    ls /dev/serial/by-id/*
    
    # Start micro_ros_agent:
    ros2 run micro_ros_agent micro_ros_agent serial --dev [device name]
  
  Terminal2:
    # visualice msgs
    rqt 

  Terminal3:
    # set the pwm [value] of the motor, have to be between -255 and 255
    ros2 topic pub  --once /motor_subcriber std_msgs/msg/Int16 data:\ [value]\
  
    # example
    ros2 topic pub  --once /micro_ros_subcriber std_msgs/msg/Int16 data:\ 255\

reference:
  load raspberry PI program: https://tutoduino.fr/en/pico-platformio/#google_vignette
  micro_ros_platformio: https://github.com/micro-ROS/micro_ros_platformio
  micro_ros_agent: https://micro.ros.org/docs/tutorials/core/first_application_rtos/freertos/
*/

#include "variables.hpp"

unsigned int num_handles = 1;   // 1 subscriber

void setup() {
  // turn the LED on (HIGH is the voltage level)
  pinMode(LED_BUILTIN, OUTPUT); 
  digitalWrite(LED_BUILTIN, HIGH); 
  delay(1000);

  h_bridge_setup();
  microros_setup();
  microros_add_subs();
  microros_add_executor();
}

void loop() {
  delay(10);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}

// ---- MICROROS SETUP -----
void microros_setup() {
  // Configure serial transport
  int bound_rate = 115200;
  const char *node_name = "micro_ros_platformio_node";
  const char *node_ns = ""; //namespace
  
  Serial.begin(bound_rate);
  set_microros_serial_transports(Serial);
  delay(2000);
  
  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator)); //create init_options
  RCCHECK(rclc_node_init_default(&node, node_name, node_ns, &support)); // create node
}

// ---- MICROROS SUB -----
void microros_add_subs(){
  RCCHECK(rclc_subscription_init_default( // create subscriber
    &subscriber, 
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16), 
    "motor_subcriber"));
}

void sub_callback(const void * msgin){
  const std_msgs__msg__Int16 *msg = (const std_msgs__msg__Int16 *) msgin;
  h_bridge_pwm(msg->data, 0);
  
}

// ---- MICROROS EXECUTOR -----
void microros_add_executor(){
  RCCHECK(rclc_executor_init(&executor, &support.context, num_handles, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &sub_msg, &sub_callback, ON_NEW_DATA));
}

// Error handle loop
void error_loop() {
  while(1) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(500);
  }
}