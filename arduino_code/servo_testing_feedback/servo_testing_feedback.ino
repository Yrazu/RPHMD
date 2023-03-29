#include <Servo.h>

#include <FeedBackServo.h>                                        // Servo motor library

#include "SparkFun_Displacement_Sensor_Arduino_Library.h"         // Bend sensor library
#include <Wire.h>                                                 // Needed for bend sensor

// Parallax Motor
#define FEEDBACK_PIN 2
#define SERVO_PIN    7

// Bend Sensor
ADS myFlexSensor;                                                 // Create ADS object
#define BEND_DATA_READY 3                                         // 'nDRDY' pin indicates data ready to be received
#define I2C_FREQ        400000                                    // I2C frequency for bend sensor operation
int angle;
Servo myservo;  // create servo object to control a servo

int feedbackPin = A2;  // analog pin used to connect the potentiometer
int val;    // variable to read the value from the analog pin

void setup() {
  Serial.begin(115200);
  while(!Serial){}
  Serial.println("Program loaded");

  // Initialize status LED
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // Configure bend sensor
  pinMode(D3, INPUT);
  Wire.begin();                                                   // Begins I2C communication
  Wire.setClock(I2C_FREQ);                                        // Set clock for I2C communication
  if (myFlexSensor.begin(18) == false){                             // Check if bend sensor is connected
    Serial.println(F("No sensor detected. Check wiring. Freezing..."));
    while(1){                                                     // Freeze program and blink status LED (labeled 'L' on Nano)
      digitalWrite(LED_BUILTIN, HIGH);
      delay(1000);
      digitalWrite(LED_BUILTIN, LOW);
      delay(1000);
    }
  }
  myFlexSensor.run();
  
//  myservo.attach(7);  // attaches the servo on pin 9 to the servo object

  attachInterrupt(digitalPinToInterrupt(D3), readBend, LOW);
}

void readBend(){
  if (myFlexSensor.available()){
    angle = myFlexSensor.getX();
    Serial.print("Bend: ");
    Serial.println(angle);
  }
  else {
    Serial.println("Unavailable");
  }
}

void loop() {
  if (myFlexSensor.available()){
    angle = myFlexSensor.getX();
    Serial.print("bend: ");
    Serial.println(angle);
  }
  else {
    Serial.println("Unavailable");
  }
//  val = analogRead(feedbackPin);       // reads the value of the potentiometer (value between 0 and 1023)
//  Serial.print("value: ");
//  Serial.println(val);
//  //val = map(val, 0, 1023, 0, 180);     // scale it to use it with the servo (value between 0 and 180)
//  myservo.write(180);                  // sets the servo position according to the scaled value
//  delay(1000);                           // waits for the servo to get there
//  val = analogRead(feedbackPin);       // reads the value of the potentiometer (value between 0 and 1023)
//  Serial.print("value: ");
//  Serial.println(val);
//  myservo.write(-180);
//  delay(1000);
}
