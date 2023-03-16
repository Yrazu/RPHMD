// ROPROS Robotic Hand Code!
/* Last Updated: 2/22/2023
 * Created by: Maddie Kogelis 
 * Supported by: the rest of the Ropros Gang
 */
/////////////////////////////////////////////////////////////////
#include <Servo.h>                                             // Include servo library

enum State {GRASP, PINCH, BIRDIE, POINT, REST};                // Enum for state machine cases
State state = REST;                                           // Keeps track of current state

int sol_pointer = 5;                                           // Pointer finger breaking digital output pin
int sol_middle = 6;                                            // Middle finger breaking digital output pin
int sol_other = 4;                                             // Pinkie and ring finger breaking digital output pin

int fsr_analog_pin = 2;                                        // Analog pin reading FSR data
int fsr_reading = 0;                                               // Analog reading from FSR
int fsr_threshold = 50;                                      // Max allowed value from the FSR (approximated in testing)
bool fsr_status = 0;
bool fsr_prev = 0;
int change_state = 0;

/////////////////////////////////////////////////////////////////
// INITIALIZE PROGRAM
void setup() {          
  Serial.begin(115200);                                          // Set baud rate for serial communication
  Serial.println("Program loaded.");                           // Print that program is loaded

  pinMode(sol_pointer, OUTPUT);                                // Set solenoid pin as digital output
  pinMode(sol_middle, OUTPUT);                                 // Set solenoid pin as digital output
  pinMode(sol_other, OUTPUT);                                  // Set solenoid pin as digital output
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
  activate_solenoid(sol_other);                                // Activate breaking for ring finger and pinkie
}

// Birdie: lock middle finger upright then actuate other fingers 100%
void birdie(){ 
  activate_solenoid(sol_middle);                               // Activate breaking for middle finger
}

// Point: lock pointer finger upright then actuate other fingers 100%
void point(){
  activate_solenoid(sol_pointer);                              // Activate breaking for pointer finger
}

// Rest: deactivate all breaking so fingers go back to neutral, upright position
void rest(){                                                   
  deactivate_all();                                            // Deactivate all breaking
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
  digitalWrite(sol_pointer, LOW);                              // Switch Solenoid OFF
  digitalWrite(sol_middle, LOW);                               // Switch Solenoid OFF
  digitalWrite(sol_other, LOW);                                // Switch Solenoid OFF
  
}

/////////////////////////////////////////////////////////////////
// SENSOR FUNCTIONS

// Read FSR Sensor
bool read_fsr(){
  int switch_state = 0;
  fsr_reading = analogRead(fsr_analog_pin);
  if (fsr_reading >= fsr_threshold){
    fsr_status = 1;
  }
  else if (fsr_reading <= fsr_threshold){
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
