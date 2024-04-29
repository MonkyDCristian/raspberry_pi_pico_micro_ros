/*
This example is a microros service node that return the temperature of the ESP32.  
This node use micro_ros_platformio with RPI pico and Arduino framework.

Steps:
  Edit build_src_filter in platformio.ini to build this example
    build_src_filter = +<examples/chatter/*> -<.git/> -<.svn/> 
    
  Compile and upload to raspberry PI pico, use the firmware.uf2 that is inside the .pio/build/pico/ folder
  
  Terminal1:
    # Find your serial [device name]:
    ls /dev/serial/by-id/*
    
    # Start micro_ros_agent:
    ros2 run micro_ros_agent micro_ros_agent serial --dev [device name]
  
  Terminal2:
    # visualice msgs
    rqt 

  Terminal3:
    # Call the service
     ros2 service call /rpip_status std_srvs/srv/Trigger {}\
    
reference:
  micro_ros_platformio: https://github.com/micro-ROS/micro_ros_platformio
  micro_ros_agent: https://micro.ros.org/docs/tutorials/core/first_application_rtos/freertos/
  RP2040 CPU Temperature: https://github.com/DeimosHall/RP2040_CPU_Temperature
  micro-ROS service: https://micro.ros.org/docs/tutorials/programming_rcl_rclc/service/
  micro-ROS String Utilities: https://docs.vulcanexus.org/en/iron/rst/microros_documentation/user_api/user_api_utilities.html
  float to char *: https://community.platformio.org/t/convert-float-to-char-array/21484
*/

#include "variables.hpp"

unsigned int num_handles = 1;   // 1 service

int LED_UP = 0;
int LED_DOWN = 1;
int led_state = LED_UP;

CPU cpu;

void setup() {
  // turn the LED on (HIGH is the voltage level)
  pinMode(LED_BUILTIN, OUTPUT); 
  digitalWrite(LED_BUILTIN, HIGH); 
  delay(1000);

  cpu.begin();  // Initialize the CPU temperature sensor

  microros_setup();
  microros_add_services();
  microros_add_executor();
}

void loop() {
  if (Serial.available() > 0) {
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)));
   
    if  (led_state == LED_DOWN) {
      led_state = LED_UP;
    }
    else {
      led_state = LED_DOWN;
    }
  }
  digitalWrite(LED_BUILTIN, (led_state) ? LOW : HIGH); 
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

// ---- MICROROS SERVICES -----
void microros_add_services(){
  const char * service_name = "/rpip_status";
  const rosidl_service_type_support_t * type_support = ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger);
  RCCHECK(rclc_service_init_default(&service, &node, type_support, service_name));
}

void service_callback(const void * req_msg, void * res_msg){
  // Cast messages to expected types
  std_srvs__srv__Trigger_Request * req_in =(std_srvs__srv__Trigger_Request *) req_msg;
  std_srvs__srv__Trigger_Response * res_out = (std_srvs__srv__Trigger_Response *) res_msg;
  
  // microros string message response
  const char * str = "Temperature: ";
  rosidl_runtime_c__String ros_str = micro_ros_string_utilities_init(str);

  String fabc(cpu.getTemperature(), 2);
  const char* t_str = fabc.c_str();
 
  ros_str = micro_ros_string_utilities_append(ros_str, t_str);
  ros_str = micro_ros_string_utilities_append(ros_str, "°C");
  
  response_msg.message.data = ros_str.data;

  // Handle request message and set the response message values
  res_out->success = true;
  res_out->message.data = response_msg.message.data;
}

// ---- MICROROS EXECUTOR -----
void microros_add_executor(){
  RCCHECK(rclc_executor_init(&executor, &support.context, num_handles, &allocator));
  RCCHECK(rclc_executor_add_service(&executor, &service, &request_msg, &response_msg, service_callback));
}

// Error handle loop
void error_loop() {
  while(1) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(500);
  }
}
