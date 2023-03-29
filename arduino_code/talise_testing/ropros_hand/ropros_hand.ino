// ROPROS Robotic Hand Code!
/* Last Updated: 2/22/2023
 * Created by: Maddie Kogelis 
 * Supported by: the rest of the Ropros Gang
 */
/////////////////////////////////////////////////////////////////
#include <FeedBackServo.h>                                     // Include library for parallax motor. Download here:https://github.com/HyodaKazuaki/Parallax-FeedBack-360-Servo-Control-Library-4-Arduino
#include <Wire.h>                                              // Include wire library for bend sensor use
#include <SparkFun_Displacement_Sensor_Arduino_Library.h>      // Include bend sensor library
#include <SPI.h>                                               // 
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// State Machine Variables
enum State {GRASP, PINCH, BIRDIE, POINT, REST};                // Enum for state machine cases
State state = REST;                                            // Keeps track of current state

// Solenoid Variables
#define SOL_POINTER     5                                      // Pointer finger digital output pin
#define SOL_MIDDLE      6                                      // Middle finger digital output pin
#define SOL_OTHER       4                                      // Pinkie and ring finger digital output pin

// Force Sensitive Resistor Variables
#define FSR_ANALOG_PIN  2                                      // Analog pin reading FSR data
#define FSR_THRESHOLD   10                                     // Threshold FSR value to indicate user tap
int fsr_reading = 0;                                           // Analog reading from FSR
bool fsr_status = 0;                                           // 1 when threshold is reached, 0 when below threshold
bool fsr_prev = 0;                                             // Previous FSR status in last reading
bool change_state = 0;                                         // Based on FSR status and previous status, state changes when current status = 1 and previous = 0

// Bend Sensor Variables - one axis sensor
ADS myFlexSensor;                                              // Create object of the ADS class
#define BEND_DATA_READY  4                                     // 'nDRDY' pin according to pinout that indicates data is ready to be recieved 
#define I2C_FREQ         400000                                // I2C frequency for bend sensor operation

// Motor Variables
#define MOT_FEEDBACK_PIN    2                                  // Motor angular position feedback data pin (needs to be pin 2 or 3)
#define MOT_PIN             7                                  // Motor PWM pin

#define MOT_GRASP_ANGLE     300                                // Motor angle for a full grasp
#define MOT_REST_ANGLE      50                                 // Motor angle for upright fingers
#define MOT_INTERVAL        50                                 // Motor movement interval in milliseconds
#define MOT_THRESHOLD_LOW   40                                 // Motor low angle threshold
#define MOT_THRESHOLD_HIGH  350                                // Motor high angle threshold
#define MOT_TOLERANCE       0                                  // Motor error tolerance
FeedBackServo servo = FeedBackServo(MOT_FEEDBACK_PIN);         // Set feedback signal pin number
int motor_angle = MOT_THRESHOLD_LOW;                           // Set initial motor angle
int motor_time;

// OLED Variables
#define SCREEN_WIDTH 128                                      // OLED display width, in pixels
#define SCREEN_HEIGHT 64                                      // OLED display height, in pixels
#define OLED_RESET     -1                                     // Reset pin # (-1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
extern const unsigned char bitmap_birdie [];
extern const unsigned char bitmap_grasp [];
extern const unsigned char bitmap_pinch []; 
extern const unsigned char bitmap_point [];
extern const unsigned char bitmap_rest [];

/////////////////////////////////////////////////////////////////
// INITIALIZE PROGRAM
void setup() {       
  // Initialize Serial Communication   
  Serial.begin(115200);                                        // Set baud rate for serial communication
  while (!Serial)                                              // Loop until serial data is recieved
  ;
  Serial.println("Program loaded.");                           // Print that program is loaded

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  // Show initial display buffer Adafruit splash screen
  display.display();
  delay(2000);
  // Clear the buffer
  display.clearDisplay();
  delay(200);

  // Initialize Solenoids
  pinMode(SOL_POINTER, OUTPUT);                                // Set solenoid pin as digital output
  pinMode(SOL_MIDDLE, OUTPUT);                                 // Set solenoid pin as digital output
  pinMode(SOL_OTHER, OUTPUT);                                  // Set solenoid pin as digital output

  // Initialize Status LED 
  pinMode(LED_BUILTIN, OUTPUT);                                // Set LED pin as digital output
  digitalWrite(LED_BUILTIN, LOW);                              // Turn off LED

  // Initialize Bend Sensor
  pinMode(BEND_DATA_READY, INPUT);                             // Set bend sensor data pin as digital input              
  Wire.begin();                                                // Begins I2C communication
  Wire.setClock(I2C_FREQ);                                     // Set clock for I2C communication
  if (myFlexSensor.begin() == false) {                          // Check if bend sensor is connected
    Serial.println(F("No sensor detected. Check wiring. Freezing..."));
    while(1){                                                  // Freeze program and blink status LED (labeled 'L' on Nano)
      digitalWrite(LED_BUILTIN, HIGH);                         // Turn on LED
      delay(1000);                                             // Wait 1 second
      digitalWrite(LED_BUILTIN, LOW);                          // Turn off LED
      delay(1000);                                             // Wait 1 second
    }
  }

  //Initialize Motor
  servo.setServoControl(MOT_PIN);                              // Set servo pwm control pin
  servo.setKp(1.0);                                            // Set Kp to proportional controller
  motor_time = millis();
  motor_rest();                                                // Set motor angle to rest position
}

/////////////////////////////////////////////////////////////////
// MAIN LOOP
void loop() {
  mode_state_machine();                                        // Call the state machine to activate the desired state
}

/////////////////////////////////////////////////////////////////
// STATE MACHINE MODE FUNCTION
void mode_state_machine() {
  change_state = read_fsr();
  switch(state) {                                               // Switch cases according to the state variable
    case GRASP:                                                // Grasp: actuate all fingers to grasp an object
      grasp();                                                 // Call the state handler function for grasping
      if (change_state == 1) {
        Serial.println("Change to pinch");
        //rest();
        state = PINCH;
      }
      break; 
    case PINCH:                                                // Pinch: lock ring finger and pinkie in the upright position then actuate middle and pointer fingers
      pinch();                                                 // Call the state handler function for pinching
      if (change_state == 1) {
        Serial.println("Change to birdie");
        //rest();
        state = BIRDIE;
      }
      break;
    case BIRDIE:                                               // Birdie: lock middle finger and actuate others
      birdie();                                                // Call the state handler function for birdie
      if (change_state == 1) {
        Serial.println("Change to point");
        //rest();
        state = POINT;
      }
      break;
    case POINT:                                                // Point: lock pointer finger and actuate others
      point();                                                 // Call the state handler function for pointing
      if (change_state == 1) {
        Serial.println("Change to rest");
        state = REST;
      }
      break;
    case REST:                                                 // Rest: release all breaks and allow reverse motor back to neutral position
      rest();                                                  // Call the state handler function for resting
      if (change_state == 1) {
        Serial.println("Change to grasp");
        state = GRASP;
      }
      break;
  }
}

/////////////////////////////////////////////////////////////////
// STATE HANDLERS

// Grasp: actuate all fingers 100%
void grasp() {
   displayImage(bitmap_grasp);                                 // Show grasp image on OLED display
   motor_mirror_thumb();                                        // Actuate fingers based on bend sensor
}

// Pinch: lock ring finger and pinkie upright then actuate other fingers 100%
void pinch() {
  displayImage(bitmap_pinch);                                  // Show pinch image on OLED display
  activate_solenoid(SOL_OTHER);                                // Activate braking for ring finger and pinkie
  motor_mirror_thumb();                                        // Actuate fingers based on bend sensor
}

// Birdie: lock middle finger upright then actuate other fingers 100%
void birdie() { 
  displayImage(bitmap_birdie);                                 // Show birdie image on OLED display
  activate_solenoid(SOL_MIDDLE);                               // Activate braking for middle finger
  motor_full_grasp();                                          // Actuate fingers 100%
}

// Point: lock pointer finger upright then actuate other fingers 100%
void point() {
  displayImage(bitmap_point);                                  // Show point image on OLED display
  activate_solenoid(SOL_POINTER);                              // Activate braking for pointer finger
  motor_full_grasp();                                          // Actuate fingers 100%
}

// Rest: reset motor so fingers go back to neutral, upright position then deactivate all braking
void rest() {      
  displayImage(bitmap_rest);                                   // Show rest image on OLED display
  motor_rest();                                                // Reset fingers to upright position
  deactivate_all();                                            // Deactivate all braking
}

/////////////////////////////////////////////////////////////////
// OLED FUNCTIONS
void displayImage(const uint8_t* bitmap) {
  //Show bitmap on OLED display
  display.clearDisplay();
  delay(200);
  display.drawBitmap(0, 0, bitmap, SCREEN_WIDTH, SCREEN_HEIGHT, WHITE);
  display.display();
}

/////////////////////////////////////////////////////////////////
// MOTOR FUNCTIONS
int motor_full_grasp() {
  ////////////// Code Blocking Method //////////////
  // Rotate motor to full grasp angle
  if (abs(MOT_GRASP_ANGLE - servo.Angle()) > MOT_TOLERANCE) {
    servo.rotate(MOT_GRASP_ANGLE, MOT_TOLERANCE);
    Serial.print("Angle: ");
    Serial.print(MOT_GRASP_ANGLE);
    Serial.print("    Servo: ");
    Serial.println(servo.Angle());
  }

//  ////////////// Looping Method //////////////
//  // Rotate the motor to full grasp angle in increments
//  if (MOT_GRASP_ANGLE - servo.Angle() > MOT_TOLERANCE) {
//    int angle = servo.Angle() + 10;
//    servo.rotate(angle, MOT_TOLERANCE);
//    Serial.print("Angle: ");
//    Serial.print(angle);
//    Serial.print("    Servo: ");
//    Serial.println(servo.Angle());
//  }

//  ////////////// Old Method //////////////
//  motor_angle = servo.Angle();
//  Serial.println("New angle: ");
//  Serial.println(motor_angle);
//  servo.rotate(desired_angle, 0);
//  change_state = read_fsr();
//  
//  while (change_state == false){
//    change_state = read_fsr();
//  }
//  
//  return motor_angle;
}

int motor_mirror_thumb() {
  // Read bend sensor angle
  if (myFlexSensor.available()) {
    if (millis() - motor_time > MOT_INTERVAL) {
      motor_angle = myFlexSensor.getX();
      motor_time = millis();
      Serial.print("Bend: ");
      Serial.print(angle);
      Serial.print("    ");
    }
  } else {
    Serial.println("Bend sensor not available");
  }

  // Ensure angle is within motor thresholds
  if (motor_angle < MOT_THRESHOLD_LOW)
    motor_angle = MOT_THRESHOLD_LOW;
  if (motor_angle > MOT_THRESHOLD_HIGH
    motor_angle = MOT_THRESHOLD_HIGH;
  Serial.print("Angle: ");
  Serial.print(motor_angle);
  Serial.print("    ");

  // Rotate motor to new angle
  servo.rotate(motor_angle, MOT_TOLERANCE);
  Serial.print("Servo: ");
  Serial.println(servo.Angle());

  return
}

int motor_rest() {
  // Copy motor_full_grasp() code once method is decided on
}

/////////////////////////////////////////////////////////////////
// SOLENOID FUNCTIONS

// Activate a solenoid
void activate_solenoid(int solenoidPin) {
  digitalWrite(solenoidPin, HIGH);                             // Switch Solenoid ON
}

// Deactivate a solenoid
void deactivate_solenoid(int solenoidPin) {
  digitalWrite(solenoidPin, LOW);                              // Switch Solenoid OFF
}

// Deactivate all solenoids
void deactivate_all() {
  digitalWrite(SOL_POINTER, LOW);                              // Switch Solenoid OFF
  digitalWrite(SOL_MIDDLE, LOW);                               // Switch Solenoid OFF
  digitalWrite(SOL_OTHER, LOW);                                // Switch Solenoid OFF
  
}

/////////////////////////////////////////////////////////////////
// SENSOR FUNCTIONS

// Read FSR Sensor
bool read_fsr() {
  int switch_state = 0;
  fsr_reading = analogRead(FSR_ANALOG_PIN);
  if (fsr_reading >= FSR_THRESHOLD) {
    fsr_status = 1;
  }
  else if (fsr_reading <= FSR_THRESHOLD) {
    fsr_status = 0;
  }
  if (fsr_status == 1 & fsr_prev == 0) {
    switch_state = 1;
  }
  Serial.println(switch_state);
  delay(100);
  fsr_prev = fsr_status;
  return switch_state;
}
