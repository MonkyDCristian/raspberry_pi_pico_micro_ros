/*
This example is a combination of publisher, subscriber and timer running in the same 
microros node using micro_ros_platformio with raspberry PI and Arduino framework

Steps:

  Add to your /.pio/libdeps/esp32doit-devkit-v1/micro_ros_platformio/metas/colcon.meta file:
  "microxrcedds_client": {
      "cmake-args": [
          "-DUCLIENT_HARD_LIVELINESS_CHECK=ON",
          "-DUCLIENT_HARD_LIVELINESS_CHECK_TIMEOUT=1000"
      ]
  }
 
  Edit build_src_filter in platformio.ini to build this example
    build_src_filter = +<examples/reconnection/*> -<.git/> -<.svn/> 
  
  Terminal1:
    # Find your serial [device name]:
    ls /dev/serial/by-id/*
    
    # Start micro_ros_agent:
    ros2 run micro_ros_agent micro_ros_agent serial --dev [device name]
  
  Terminal2:
    # visualice msgs
    rqt 

reference:
  micro_ros_platformio: https://github.com/micro-ROS/micro_ros_platformio
  micro_ros_agent: https://micro.ros.org/docs/tutorials/core/first_application_rtos/freertos/
  handle_reconnections: https://docs.vulcanexus.org/en/latest/rst/tutorials/micro/handle_reconnections/handle_reconnections.html
*/

#include "variables.hpp"

unsigned int num_handles = 2;   // 1 subscriber, 1 timer

void setup() {
  pinMode(LED_BUILTIN, OUTPUT); 
  digitalWrite(LED_BUILTIN, HIGH); 

  // Configure serial transport, you can use Serial/1/2
  int bound_rate = 115200;
  Serial.begin(bound_rate); 
  set_microros_serial_transports(Serial);
  delay(2000);

  state = WAITING_AGENT;

  pub_msg.data = 0;
}

void loop() {
  microros_loop();
  digitalWrite(LED_BUILTIN, (sub_msg.data == 0) ? LOW : HIGH); 
}

void sub_callback(const void * msgin){
  // Cast message pointer to expected type
  const std_msgs__msg__Int16 *msg = (const std_msgs__msg__Int16 *) msgin;
  sub_msg.data = msg->data;
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    rcl_publish(&publisher, &pub_msg, NULL);
    pub_msg.data++;
  }
}

// 
// Functions create_entities and destroy_entities can take several seconds.
// In order to reduce this rebuild the library with
// - RMW_UXRCE_ENTITY_CREATION_DESTROY_TIMEOUT=0
// - UCLIENT_MAX_SESSION_CONNECTION_ATTEMPTS=3

bool microros_create_entities()
{
  // ---- MICROROS SETUP ----- 
  const char *node_name = "micro_ros_platformio_node";
  const char *node_ns = ""; //namespace

  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator)); //create init_options
  RCCHECK(rclc_node_init_default(&node, node_name, node_ns, &support)); // create node

  // ---- MICROROS PUB ----- define as many piublishers as you need

  RCCHECK(rclc_publisher_init_default( // create publisher
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
    "micro_ros_publisher"));

// ---- MICROROS SUB ----- define as many subscribers as you need
  RCCHECK(rclc_subscription_init_default( // create subscriber
    &subscriber, 
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16), 
    "micro_ros_subscriber"));


  // ---- MICROROS TIMERS -----

  const unsigned int timer_timeout = 500; // create timer/ 2 Hz
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));


  // ---- MICROROS EXECUTOR ----- add all yours timers and subscribers
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, num_handles, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &sub_msg, &sub_callback, ON_NEW_DATA));


  // microros_setup();
  // microros_add_pubs();
  // microros_add_subs();
  // microros_add_timers();
  // microros_add_executor();
  return true;
}

void microros_destroy_entities()
{
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_publisher_fini(&publisher, &node);
  rcl_subscription_fini(&subscriber, &node);
  rcl_timer_fini(&timer);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

void microros_loop()
{
  switch (state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
      break;

    case AGENT_AVAILABLE:
      state = (true == microros_create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) {
        microros_destroy_entities();
      };
      break;

    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
      if (state == AGENT_CONNECTED) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
      }
      break;
      
    case AGENT_DISCONNECTED:
      microros_destroy_entities();
      state = WAITING_AGENT;
      break;

    default:
      break;
  }
}

void error_loop() {
  while(1) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(500);
  }
}
