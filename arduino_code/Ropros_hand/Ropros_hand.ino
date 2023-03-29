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
State state = BIRDIE; //REST;                                            // Keeps track of current state

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
#define BEND_ADDRESS     18

// Motor Variables
#define MOT_FEEDBACK_PIN    2                                  // Motor angular position feedback data pin (needs to be pin 2 or 3)
#define MOT_PIN             7                                  // Motor PWM pin
FeedBackServo servo = FeedBackServo(MOT_FEEDBACK_PIN);         // Set feedback signal pin number
int motor_angle;

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

int fullGrasp = 180;
bool i = 1;

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
  if (myFlexSensor.begin(BEND_ADDRESS) == false){                          // Check if bend sensor is connected
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
      grasp();                                                 // Call the state handler function for grasping
      if (change_state == 1){
        i = 1;
        Serial.println("Change to rest");//Serial.println("Change to pinch");
        //rest();
        state = REST;  //PINCH;
      }
      break; 
    case PINCH:                                                // Pinch: lock ring finger and pinkie in the upright position then actuate middle and pointer fingers
      pinch();                                                 // Call the state handler function for pinching
      if (change_state == 1){
        Serial.println("Change to birdie");
        //rest();
        state = BIRDIE;
      }
      break;
    case BIRDIE:                                               // Birdie: lock middle finger and actuate others
      birdie();                                                // Call the state handler function for birdie
      if (change_state == 1){
        Serial.println("Change to point");
        //rest();
        state = POINT;
      }
      break;
    case POINT:                                                // Point: lock pointer finger and actuate others
      point();                                                 // Call the state handler function for pointing
      if (change_state == 1){
        Serial.println("Change to rest");
        state = REST;
      }
      break;
    case REST:                                                 // Rest: release all breaks and allow reverse motor back to neutral position
      rest();                                                  // Call the state handler function for resting
      if (change_state == 1){
        i = 1;
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
   displayImage(bitmap_grasp);                                 // Show grasp image on OLED display
   motor_full_grasp(fullGrasp);
}

// Pinch: lock ring finger and pinkie upright then actuate other fingers 100%
void pinch(){
  displayImage(bitmap_pinch);                                 // Show pinch image on OLED display
  activate_solenoid(SOL_OTHER);                                // Activate braking for ring finger and pinkie
}

// Birdie: lock middle finger upright then actuate other fingers 100%
void birdie(){ 
  displayImage(bitmap_birdie);                                // Show birdie image on OLED display
  activate_solenoid(SOL_MIDDLE);                               // Activate braking for middle finger
  motor_angle = servo.Angle();
  servo.rotate(180, 4);
  Serial.println("New angle: ");
  Serial.println(motor_angle);
  motor_angle = servo.Angle();
  servo.rotate(-180, 4);
  Serial.println("New angle: ");
  Serial.println(motor_angle);
}

// Point: lock pointer finger upright then actuate other fingers 100%
void point(){
  displayImage(bitmap_point);                                 // Show point image on OLED display
  activate_solenoid(SOL_POINTER);                              // Activate braking for pointer finger
}

// Rest: deactivate all braking so fingers go back to neutral, upright position
void rest(){      
  displayImage(bitmap_rest);                                  // Show rest image on OLED display                                               
  deactivate_all();                                            // Deactivate all braking
  motor_reset(fullGrasp);
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
void motor_full_grasp(int desired_angle){
  //if (i == 1) {
    motor_angle = servo.Angle();
    servo.rotate(-300, 4);
    Serial.println("New angle: ");
    Serial.println(motor_angle);
    i = 0;
  //}
  //return motor_angle;
}

void motor_reset(int desired_angle){
  //if (i == 1){
    motor_angle = servo.Angle();
    servo.rotate(0, 4);
    Serial.println("New angle: ");
    Serial.println(motor_angle);
    i = 0;
  //}
  //return motor_angle
}

int motor_mirror_thumb(){
  
}

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
