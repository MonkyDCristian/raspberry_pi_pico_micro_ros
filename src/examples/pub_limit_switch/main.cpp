/*
This example it is a ecoder velocity publisher using 
micro_ros_platformio with raspberry PI and Arduino framework

Steps:

  Edit build_src_filter in platformio.ini to build this example
    build_src_filter = +<examples/pub_limit_switch/*> -<.git/> -<.svn/> 
  
  Compile and upload to raspberry PI, use the firmware.uf2 that is inside the .pio/build/pico/ folder
  
  Terminal1:
    # Find your serial [device name]:
    ls /dev/serial/by-id/*
    
    # Start micro_ros_agent:
    ros2 run micro_ros_agent micro_ros_agent serial --dev [device name]
  
  Terminal2:
    # visualice msgs
    ros2 topic echo /encoder_publisher

reference:
  load raspberry PI program: https://tutoduino.fr/en/pico-platformio/#google_vignette
  micro_ros_platformio: https://github.com/micro-ROS/micro_ros_platformio
  micro_ros_agent: https://micro.ros.org/docs/tutorials/core/first_application_rtos/freertos/
*/

#include "variables.hpp"

unsigned int num_handles = 2;   // 2 publisher

void setup() {
  // turn the LED on (HIGH is the voltage level)
  pinMode(LED_BUILTIN, OUTPUT); 
  digitalWrite(LED_BUILTIN, HIGH); 
 
  limit_switch_setup();

  delay(1000);
 
  microros_setup();
  microros_add_pubs();
  microros_add_timers();
  microros_add_executor();
}

void loop() {
  delay(10);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}

void limit_switch_setup(){
    pinMode(LSPinRIGHT, INPUT_PULLUP);
    pinMode(LSPinLEFT, INPUT_PULLUP);
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

// ---- MICROROS PUB -----
void microros_add_pubs(){
  RCCHECK(rclc_publisher_init_default( // create publisher
    &ls_right_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Empty),
    "rpi/ls_right_pub"));
  
  RCCHECK(rclc_publisher_init_default( // create publisher
    &ls_left_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Empty),
    "rpi/ls_left_pub"));
} 

// ---- MICROROS TIMERS -----
void microros_add_timers(){
  const unsigned int timer_timeout = 100; // 10 hz
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  
  if (timer != NULL) {
    ls_right_state = digitalRead(LSPinRIGHT);
    ls_left_state = digitalRead(LSPinLEFT);

    if (ls_right_state == LOW && last_ls_right_state == HIGH){
      RCSOFTCHECK(rcl_publish(&ls_right_publisher, &ls_right_msg, NULL));
    }

    if (ls_left_state == LOW && last_ls_left_state == HIGH){
      RCSOFTCHECK(rcl_publish(&ls_left_publisher, &ls_left_msg, NULL));
    }

    last_ls_right_state = ls_right_state;
    last_ls_left_state = ls_left_state;
  }
}

// ---- MICROROS EXECUTOR -----
void microros_add_executor(){
  RCCHECK(rclc_executor_init(&executor, &support.context, num_handles, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
}

// Error handle loop
void error_loop() {
  while(1) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(500);
  }
}
