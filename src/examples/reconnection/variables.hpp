#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/int16.h>

#define LED_PIN 13
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){return false;}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)\


rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

rcl_publisher_t publisher;
std_msgs__msg__Int16 pub_msg;

rcl_subscription_t subscriber;
std_msgs__msg__Int16 sub_msg;

rcl_timer_t timer;
rclc_executor_t executor;

bool micro_ros_init_successful;

enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;


void sub_callback(const void * msgin);
void timer_callback(rcl_timer_t * timer, int64_t last_call_time);
bool microros_create_entities();
void microros_destroy_entities();
void microros_loop();
void error_loop();

