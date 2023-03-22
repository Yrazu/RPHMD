#include <FeedBackServo.h>                                        // Servo motor library

#include <SparkFun_Displacement_Sensor_Arduino_Library.h>         // Bend sensor library
#include <Wire.h>                                                 // Needed for bend sensor

// Parallax Motor
#define FEEDBACK_PIN 2
#define SERVO_PIN    7

// Bend Sensor
ADS myFlexSensor;                                                 // Create ADS object
#define BEND_DATA_READY 3                                         // 'nDRDY' pin indicates data ready to be received
#define I2C_FREQ        400000                                    // I2C frequency for bend sensor operation

// Parallax Motor
FeedBackServo servo = FeedBackServo(FEEDBACK_PIN);                // Set feedback signal pin number

int angle = 40;
int lastAngle = angle;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while (!Serial) { }
  Serial.println("\nProgram loaded.");

  // Initialize status LED
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // Configure motor
  servo.setServoControl(SERVO_PIN);                               // Set servo control pin number
  servo.setKp(1.0);                                               // Set Kp to proportional controller

  // Configure bend sensor
  pinMode(BEND_DATA_READY, INPUT);                                // Set bend sensor data pin as digital input              
  Wire.begin();                                                   // Begins I2C communication
  Wire.setClock(I2C_FREQ);                                        // Set clock for I2C communication
  if (myFlexSensor.begin() == false){                             // Check if bend sensor is connected
    Serial.println(F("No sensor detected. Check wiring. Freezing..."));
    while(1){                                                     // Freeze program and blink status LED (labeled 'L' on Nano)
      digitalWrite(LED_BUILTIN, HIGH);
      delay(1000);
      digitalWrite(LED_BUILTIN, LOW);
      delay(1000);
    }
  }
}

void loop() {
  if (myFlexSensor.available()) {
    angle = myFlexSensor.getX();
    Serial.print("Bend: ");
    Serial.println(angle);
    angle += 90;
  }
  
  if (angle < 40)
    angle = 40;
  if (angle > 350)
    angle = 350;

  Serial.print("           Angle: ");
  Serial.println(angle);
    
  if (abs(lastAngle - angle) > 1) {
    lastAngle = angle;
    servo.rotate(angle, 0);
    Serial.print("                        Servo angle: ");
    Serial.println(servo.Angle());
  }
}
