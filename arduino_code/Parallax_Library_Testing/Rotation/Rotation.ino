#include <Keyboard.h>

#include "FeedBackServo.h"

// define feedback signal pin and servo control pin
#define FEEDBACK_PIN 2
#define SERVO_PIN 7
char inChar = 'b';

// set feedback signal pin number
FeedBackServo servo = FeedBackServo(FEEDBACK_PIN);

void setup() {
    // set servo control pin number
    servo.setServoControl(SERVO_PIN);
    servo.setKp(1.0);
    // Initialize Serial Communication   
    Serial.begin(115200);                                        // Set baud rate for serial communication
    while (!Serial)                                              // Loop until serial data is recieved
    ;
    Serial.println("Program loaded.");                           // Print that program is loaded
    //Keyboard.begin();

}

void loop() {
  servo.rotate(180);
  delay(1000);
  servo.rotate(-180);
  delay(1000);
//  if (Serial.available() > 0) {
//
//    // read incoming serial data:
//
//    inChar = Serial.read();
//
//  }
//  if (inChar == 'a'){
//    servo.rotate(400, 4);
//    Serial.print("New angle: ");
//    Serial.println(servo.Angle());
//  }
//  if (inChar == 'b'){
//    servo.rotate(0, 4);
//    Serial.print("New angle: ");
//    Serial.println(servo.Angle());
//  }
}
