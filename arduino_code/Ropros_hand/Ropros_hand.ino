// ROPROS Robotic Hand Code!
/* Last Updated: 2/22/2023
 * Created by: Maddie Kogelis 
 * Supported by: the rest of the Ropros Gang
 */
/////////////////////////////////////////////////////////////////
#include <FeedBackServo.h>                                     // Include library for parallax motor. Download here:https://github.com/HyodaKazuaki/Parallax-FeedBack-360-Servo-Control-Library-4-Arduino
#include <Wire.h>                                              // Include wire library for bend sensor use
#include <SparkFun_Displacement_Sensor_Arduino_Library.h>      // Include bend sensor library

// State Machine Variables
enum State {GRASP, PINCH, BIRDIE, POINT, REST};                // Enum for state machine cases
State state = REST;                                            // Keeps track of current state

// Solenoid Variables
#define SOL_POINTER     5                                      // Pointer finger digital output pin
#define SOL_MIDDLE      6                                      // Middle finger digital output pin
#define SOL_OTHER       4                                      // Pinkie and ring finger digital output pin

// Force Sensitive Resistor Variables
#define FSR_ANALOG_PIN  2                                      // Analog pin reading FSR data
#define FSR_THRESHOLD   50                                     // Threshold FSR value to indicate user tap
int fsr_reading = 0;                                           // Analog reading from FSR
bool fsr_status = 0;                                           // 1 when threshold is reached, 0 when below threshold
bool fsr_prev = 0;                                             // Previous FSR status in last reading
bool change_state = 0;                                         // Based on FSR status and previous status, state changes when current status = 1 and previous = 0

// Bend Sensor Variables - one axis sensor
ADS myFlexSensor;                                              // Create object of the ADS class
#define BEND_DATA_READY  4                                    // 'nDRDY' pin according to pinout that indicates data is ready to be recieved 
#define I2C_FREQ         400000                               // I2C frequency for bend sensor operation

// Motor Variables
#define MOT_FEEDBACK_PIN    2                                  // Motor angular position feedback data pin (needs to be pin 2 or 3)
#define MOT_PIN             7                                  // Motor PWM pin
FeedBackServo servo = FeedBackServo(MOT_FEEDBACK_PIN);         // Set feedback signal pin number

/////////////////////////////////////////////////////////////////
// INITIALIZE PROGRAM
void setup() {       
  // Initialize Serial Communication   
  Serial.begin(115200);                                        // Set baud rate for serial communication
  while (!Serial)                                              // Loop until serial data is recieved
  ;
  Serial.println("Program loaded.");                           // Print that program is loaded

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
  if (myFlexSensor.begin() == false){                          // Check if bend sensor is connected
    Serial.println(F("No sensor detected. Check wiring. Freezing..."));
    while(1){                                                  // Freeze program and blink status LED (labeled 'L' on Nano)
      digitalWrite(LED_BUILTIN, HIGH);                         // Turn on LED
      delay(1000);                                             // Wait 1 second
      digitalWrite(LED_BUILTIN, LOW);                          // Turn off LED
      delay(1000);                                             // Wait 1 second
    }
  }
  
  servo.setServoControl(MOT_PIN);                              // Set servo pwm control pin
  servo.setKp(1.0);                                            // Set Kp to proportional controller
}

/////////////////////////////////////////////////////////////////
// MAIN LOOP
void loop() {
  mode_state_machine();                                        // Call the state machine to activate the desired state
}

/////////////////////////////////////////////////////////////////
// STATE MACHINE MODE FUNCTION
void mode_state_machine(){
  change_state = read_fsr();
  switch(state){                                               // Switch cases according to the state variable
    case GRASP:                                                // Grasp: actuate all fingers to grasp an object
      //grasp();                                                 // Call the state handler function for grasping
      if (change_state == 1){
        Serial.println("Change to pinch");
        //rest();
        state = PINCH;
      }
      break; 
    case PINCH:                                                // Pinch: lock ring finger and pinkie in the upright position then actuate middle and pointer fingers
      //pinch();                                                 // Call the state handler function for pinching
      if (change_state == 1){
        Serial.println("Change to birdie");
        //rest();
        state = BIRDIE;
      }
      break;
    case BIRDIE:                                               // Birdie: lock middle finger and actuate others
      //birdie();                                                // Call the state handler function for birdie
      if (change_state == 1){
        Serial.println("Change to point");
        //rest();
        state = POINT;
      }
      break;
    case POINT:                                                // Point: lock pointer finger and actuate others
      //point();                                                 // Call the state handler function for pointing
      if (change_state == 1){
        Serial.println("Change to rest");
        state = REST;
      }
      break;
    case REST:                                                 // Rest: release all breaks and allow reverse motor back to neutral position
      //rest();                                                  // Call the state handler function for resting
      if (change_state == 1){
        Serial.println("Change to grasp");
        state = GRASP;
      }
      break;
  }
}

/////////////////////////////////////////////////////////////////
// STATE HANDLERS

// Grasp: actuate all fingers 100%
void grasp(){
}

// Pinch: lock ring finger and pinkie upright then actuate other fingers 100%
void pinch(){
  activate_solenoid(SOL_OTHER);                                // Activate braking for ring finger and pinkie
}

// Birdie: lock middle finger upright then actuate other fingers 100%
void birdie(){ 
  activate_solenoid(SOL_MIDDLE);                               // Activate braking for middle finger
}

// Point: lock pointer finger upright then actuate other fingers 100%
void point(){
  activate_solenoid(SOL_POINTER);                              // Activate braking for pointer finger
}

// Rest: deactivate all braking so fingers go back to neutral, upright position
void rest(){                                                   
  deactivate_all();                                            // Deactivate all braking
}

/////////////////////////////////////////////////////////////////
// MOTOR FUNCTIONS

/////////////////////////////////////////////////////////////////
// SOLENOID FUNCTIONS

// Activate a solenoid
void activate_solenoid(int solenoidPin){
  digitalWrite(solenoidPin, HIGH);                             // Switch Solenoid ON
}

// Deactivate a solenoid
void deactivate_solenoid(int solenoidPin){
  digitalWrite(solenoidPin, LOW);                              // Switch Solenoid OFF
}

// Deactivate all solenoids
void deactivate_all(){
  digitalWrite(SOL_POINTER, LOW);                              // Switch Solenoid OFF
  digitalWrite(SOL_MIDDLE, LOW);                               // Switch Solenoid OFF
  digitalWrite(SOL_OTHER, LOW);                                // Switch Solenoid OFF
  
}

/////////////////////////////////////////////////////////////////
// SENSOR FUNCTIONS

// Read FSR Sensor
bool read_fsr(){
  int switch_state = 0;
  fsr_reading = analogRead(FSR_ANALOG_PIN);
  if (fsr_reading >= FSR_THRESHOLD){
    fsr_status = 1;
  }
  else if (fsr_reading <= FSR_THRESHOLD){
    fsr_status = 0;
  }
  if (fsr_status == 1 & fsr_prev == 0){
    switch_state = 1;
  }
  Serial.println(switch_state);
  delay(100);
  fsr_prev = fsr_status;
  return switch_state;
}

///////////////////////////////////////////////////////////////////
// GRAPHICS
