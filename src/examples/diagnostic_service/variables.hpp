#include <Arduino.h>
#include <CPU.h>

#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <micro_ros_utilities/string_utilities.h>

#include <std_srvs/srv/trigger.h>


rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

rcl_service_t service;
std_srvs__srv__Trigger_Request request_msg;
std_srvs__srv__Trigger_Response response_msg;

rclc_executor_t executor;


#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void microros_setup();
void microros_add_services();
void service_callback(const void * req_msg, void * res_msg);
void microros_add_executor();
void error_loop();

