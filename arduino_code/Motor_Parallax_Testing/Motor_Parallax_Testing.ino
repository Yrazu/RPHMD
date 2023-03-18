#include <FeedBackServo.h>
// Parallax Motor Testing
// define feedback signal pin and servo control pin
#define FEEDBACK_PIN 2
#define SERVO_PIN 7

// set feedback signal pin number
FeedBackServo servo = FeedBackServo(FEEDBACK_PIN);

void setup() {
    // set servo control pin number
    pinMode(SERVO_PIN, INPUT);
    digitalWrite(SERVO_PIN, LOW);
    servo.setServoControl(SERVO_PIN);
    // set Kp to proportional controller
    servo.setKp(1.0);
    Serial.begin(115200);
}

void loop() {
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
