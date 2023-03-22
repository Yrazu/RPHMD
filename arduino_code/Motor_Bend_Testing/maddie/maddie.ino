#include <FeedBackServo.h>
#include <Wire.h>                                              // Include wire library for bend sensor use
#include <SparkFun_Displacement_Sensor_Arduino_Library.h>      // Include bend sensor library

#include <Thread.h>
#include <ThreadController.h>

// Parallax Motor Testing
// define feedback signal pin and servo control pin
#define FEEDBACK_PIN 2
#define SERVO_PIN 7

// Bend Sensor Variables - one axis sensor
ADS myFlexSensor;                                              // Create object of the ADS class
#define BEND_DATA_READY  4                                     // 'nDRDY' pin according to pinout that indicates data is ready to be recieved 
#define I2C_FREQ         400000                                // I2C frequency for bend sensor operation
int pos;
// set feedback signal pin number
FeedBackServo servo = FeedBackServo(FEEDBACK_PIN);

ThreadController controll = ThreadController();
Thread motorThread = Thread();
Thread angleThread = Thread();

int testNum;

void angleCallback() {
  static bool ledStatus = false;
  ledStatus = !ledStatus;

  digitalWrite(LED_BUILTIN, ledStatus);

//  Serial.print("COOL! I'm running on: ");
//  Serial.println(millis());
  Serial.print("COOL! Continuous Reading: ");
  Serial.println(servo.Angle());
}

void motorCallback() {
  servo.rotate(180, 0);
  Serial.print("Now at angle: ");
  Serial.println(servo.Angle());
  servo.rotate(-180, 0);
  Serial.print("Now at angle: ");
  Serial.println(servo.Angle());
}

void setup() {
  Serial.begin(115200);
  Serial.println("program loaded");
    // Initialize Status LED 
  pinMode(LED_BUILTIN, OUTPUT);                                // Set LED pin as digital output
  //digitalWrite(LED_BUILTIN, LOW);                              // Turn off LED

    // Initialize Bend Sensor
//  pinMode(BEND_DATA_READY, INPUT);                             // Set bend sensor data pin as digital input              
//  Wire.begin();                                                // Begins I2C communication
//  Wire.setClock(I2C_FREQ);                                     // Set clock for I2C communication
//  if (myFlexSensor.begin() == false){                          // Check if bend sensor is connected
//    Serial.println(F("No sensor detected. Check wiring. Freezing..."));
//    while(1){                                                  // Freeze program and blink status LED (labeled 'L' on Nano)
//      digitalWrite(LED_BUILTIN, HIGH);                         // Turn on LED
//      delay(1000);                                             // Wait 1 second
//      digitalWrite(LED_BUILTIN, LOW);                          // Turn off LED
//      delay(1000);                                             // Wait 1 second
//    }
//  }

  // Configure threads
  motorThread.onRun(motorCallback);
  motorThread.setInterval(1000);

  angleThread.onRun(angleCallback);
  angleThread.setInterval(100);

  controll.add(&motorThread);
  controll.add(&angleThread);

  angleThread.enabled = false;

  // set servo control pin number
  servo.setServoControl(SERVO_PIN);
  // set Kp to proportional controller
  servo.setKp(1.0);
}
  

void loop() {
    controll.run();

    Serial.print("COOL! Continuous Reading: ");
    Serial.println(servo.Angle());

//    Serial.print("********** TEST NUM IS");
//    Serial.print(testNum);
//    Serial.println("**********");
//
//    testNum++;
    
//    int init_pos = servo.Angle();
//    pos = init_pos;
//    Serial.print("Now angle: ");
//    Serial.println(init_pos);
//    servo.rotate(pos+10, 4);
//    servo.rotate(pos+20, 4);
//    servo.rotate(pos+30, 4);
//    servo.rotate(pos+40, 4);
//    servo.rotate(pos+50, 4);
//    servo.rotate(pos, 4);
//    while (pos < init_pos+360){
//      servo.rotate(pos+10, 4);
//      pos = servo.Angle();
//      Serial.print("Now angle: ");
//      Serial.println(pos);
//    }
    
//    servo.rotate(180, 0);
//    Serial.print("Now angle: ");
//    Serial.println(servo.Angle());
//    //delay(1000);
//    servo.rotate(-180, 0);
//    Serial.print("Now angle: ");
//    Serial.println(servo.Angle());
//    //delay(1000);
}
