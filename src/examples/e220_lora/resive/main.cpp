/*

*/

#include <Arduino.h>

char LED_UP = 1;
int LED_DOWN = 0;
int led_state = LED_UP;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT); 
  digitalWrite(LED_BUILTIN, HIGH); 
  delay(1000);
  
  int bound_rate = 9600; //baud rate e220-400-T22

  // Do not use Serial (TX0, RX0)!! use SoftwareSerial if you need to use another UART port
  Serial1.begin(bound_rate); // RX2, TX2
}

void loop() {
  if (Serial1.available() > 0) {
    char led_on = Serial1.read();

    if  (led_on == 1) {
      led_state = LED_UP;
    }
    else {
      led_state = LED_DOWN;
    }
  }
  
  digitalWrite(LED_BUILTIN, (led_state) ? LOW : HIGH); 
}

