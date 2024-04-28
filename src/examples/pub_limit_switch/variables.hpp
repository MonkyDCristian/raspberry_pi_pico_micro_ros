#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/empty.h>

rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

rcl_publisher_t ls_right_publisher;
rcl_publisher_t ls_left_publisher;

std_msgs__msg__Empty ls_right_msg;
std_msgs__msg__Empty ls_left_msg;

rcl_timer_t timer;

rclc_executor_t executor;

// confing limit switch Pins

#define LSPinRIGHT 21
#define LSPinLEFT 22

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

char ls_right_state = HIGH;
char ls_left_state = HIGH;
char last_ls_right_state = HIGH;
char last_ls_left_state = HIGH;

void microros_setup();
void microros_add_pubs();
void microros_add_timers();
void timer_callback(rcl_timer_t * timer, int64_t last_call_time);
void microros_add_executor();
void error_loop();

void limit_switch_setup();
void limit_switch_right_cb();
void limit_switch_left_cb();
