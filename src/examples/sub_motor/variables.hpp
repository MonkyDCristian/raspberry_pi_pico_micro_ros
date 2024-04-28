#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int16.h>

#include "../../h_bridge/h_bridge.hpp"

rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

rcl_subscription_t subscriber;
std_msgs__msg__Int16 sub_msg;

rclc_executor_t executor;


#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void microros_setup();
void microros_add_subs();
void sub_callback(const void * msgin);
void microros_add_executor();
void error_loop();