#include "FeedBackServo.h"
// define feedback signal pin and servo control pin
#define FEEDBACK_PIN 2
#define SERVO_PIN 7

// set feedback signal pin number
FeedBackServo servo = FeedBackServo(FEEDBACK_PIN);

void setup() {
    Serial.begin(115200);
    Serial.print("Now angle: ");
    Serial.println(servo.Angle());
    // set servo control pin number
    servo.setServoControl(SERVO_PIN);
    // set Kp to proportional controller
    servo.setKp(1.0);
    Serial.print("Now angle: ");
    Serial.println(servo.Angle());
}

void loop() {
    Serial.print("Now angle: ");
    Serial.println(servo.Angle());
    // rotate servo to 270 and -180 degrees(with contains +-4 degrees error) each 1 second.
    servo.rotate(270, 4);
    Serial.print("Now angle: ");
    Serial.println(servo.Angle());
    delay(1000);
    servo.rotate(-180, 4);
    Serial.print("Now angle: ");
    Serial.println(servo.Angle());
    delay(1000);
}
