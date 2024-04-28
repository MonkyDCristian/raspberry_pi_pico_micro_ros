/*
This example is a combination of publisher, subscriber and timer running in the same 
microros node using micro_ros_platformio with raspberry PI and Arduino framework

Steps:

  Edit build_src_filter in platformio.ini to build this example
    build_src_filter = +<examples/pub_sub_timer/*> -<.git/> -<.svn/> 
  
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
    # Turn down the LED low
    ros2 topic pub /micro_ros_subcriber std_msgs/msg/Int16 data:\ 0\
  
    # Turn down the LED high
    ros2 topic pub /micro_ros_subcriber std_msgs/msg/Int16 data:\ 1\
reference:
  load raspberry PI program: https://tutoduino.fr/en/pico-platformio/#google_vignette
  micro_ros_platformio: https://github.com/micro-ROS/micro_ros_platformio
  micro_ros_agent: https://micro.ros.org/docs/tutorials/core/first_application_rtos/freertos/
*/

#include "variables.hpp"

unsigned int num_handles = 2;   // 1 subscriber, 1 timer

void setup() {
  // turn the LED on (HIGH is the voltage level)
  pinMode(LED_BUILTIN, OUTPUT); 
  digitalWrite(LED_BUILTIN, HIGH); 
  delay(1000);

  microros_setup();
  microros_add_pubs();
  microros_add_subs();
  microros_add_timers();
  microros_add_executor();
}

void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)));
  digitalWrite(LED_BUILTIN, (sub_msg.data == 0) ? LOW : HIGH); 
}

// ---- MICROROS SETUP -----
void microros_setup() {
  // Configure serial transport, you can use Serial/1/2
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

// ---- MICROROS PUB -----
// define as many piublishers as you need
void microros_add_pubs(){
  RCCHECK(rclc_publisher_init_default( // create publisher
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
    "micro_ros_publisher"));
}

// ---- MICROROS SUB -----
// define as many subscribers as you need
void microros_add_subs(){
  RCCHECK(rclc_subscription_init_default( // create subscriber
    &subscriber, 
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16), 
    "micro_ros_subscriber"));
}

void sub_callback(const void * msgin){
  // Cast message pointer to expected type
  const std_msgs__msg__Int16 *msg = (const std_msgs__msg__Int16 *) msgin;
  sub_msg.data = msg->data;
}

// ---- M//RCSOFTCHECK(rclc_executor_spin(&executor));ICROROS TIMERS -----
void microros_add_timers(){
  const unsigned int timer_timeout = 1000; // create timer
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&publisher, &pub_msg, NULL));
    pub_msg.data++;
  }
}

// ---- MICROROS EXECUTOR -----
void microros_add_executor(){
  RCCHECK(rclc_executor_init(&executor, &support.context, num_handles, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &sub_msg, &sub_callback, ON_NEW_DATA));
  pub_msg.data = 0;
}

// Error handle loop
void error_loop() {
  while(1) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(500);
  }
}
