// ROPROS Robotic Hand Code!
/* Last Updated: 2/22/2023
 * Created by: Maddie Kogelis 
 * Supported by: the rest of the Ropros Gang
 */
/////////////////////////////////////////////////////////////////
#include <Servo.h>                                             // Include servo library

enum State {GRASP, PINCH, BIRDIE, POINT, REST};                // Enum for state machine cases
State state = GRASP;                                           // Keeps track of current state

int sol_pointer = 5;                                           // Pointer finger breaking digital output pin
int sol_middle = 6;                                            // Middle finger breaking digital output pin
int sol_other = 4;                                             // Pinkie and ring finger breaking digital output pin

char character; //= 'r';                                          // The characer that is used to switch modes for demonstration

Servo myservo;                                                 // Create servo object
int servo_pin = 7;                                             // Attach pin D7 to the servo

int i;


/////////////////////////////////////////////////////////////////
// INITIALIZE PROGRAM
void setup() {          
  Serial.begin(9600);                                          // Set baud rate for serial communication
  Serial.println("Program loaded.");                           // Print that program is loaded

  myservo.attach(servo_pin);                                   // Attach servo instance to pin

  pinMode(sol_pointer, OUTPUT);                                // Set solenoid pin as digital output
  pinMode(sol_middle, OUTPUT);                                 // Set solenoid pin as digital output
  pinMode(sol_other, OUTPUT);                                  // Set solenoid pin as digital output

  Serial.println("Switch Modes:");
  Serial.println("   g = GRASP");
  Serial.println("   p = PINCH");
  Serial.println("   b = BIRDIE");
  Serial.println("   t = POINT");
  Serial.println("   r = REST");
}

/////////////////////////////////////////////////////////////////
// MAIN LOOP
void loop() {
  if (Serial.available() > 0) {                                // Switch modes only if a new character is sent
    character = Serial.read();                                 // Read the incoming byte
    deactivate_all();                                          // Disable all solenoids and the motor
    i = 1;
  }
  if(character == 'g'){                                        // If the character sent is 'g'
    state = GRASP;                                             // Switch to grasping state        
  }
  else if(character == 'p'){                                   // If the character sent is 'p'
    state = PINCH;                                             // Switch to pinching state
  }
  else if(character == 'b'){                                   // If the character sent is 'b'
    state = BIRDIE;                                            // Switch to birdie state
  }
  else if(character == 't'){                                   // If the character sent is 't'
    state = POINT;                                             // Switch to pointing state
  }
  else if(character == 'r'){                                   // If the character sent is 'r'
    state = REST;                                              // Switch to resting state
  }
  state_machine();                                             // Call the state machine to activate the desired state
}

/////////////////////////////////////////////////////////////////
// STATE MACHINE MODE FUNCTION
void state_machine(){
  switch(state){                                               // Switch cases according to the state variable
    case GRASP:                                                // Grasp: actuate all fingers to grasp an object
      grasp();                                                 // Call the state handler function for grasping
      break; 
    case PINCH:                                                // Pinch: lock ring finger and pinkie in the upright position then actuate middle and pointer fingers
      pinch();                                                 // Call the state handler function for pinching
      break;
    case BIRDIE:                                               // Birdie: lock middle finger and actuate others
      birdie();                                                // Call the state handler function for birdie
      break;
    case POINT:                                                // Point: lock pointer finger and actuate others
      point();                                                 // Call the state handler function for pointing
      break;
    case REST:                                                 // Rest: release all breaks and allow reverse motor back to neutral position
      rest();                                                  // Call the state handler function for resting
      break;
  }
}

/////////////////////////////////////////////////////////////////
// STATE HANDLERS

// Grasp: actuate all fingers 100%
void grasp(){
  if (i == 1){
    i = 0;
    fingers_actuate();                                           // Move the motor CW /////////////////////////////////// change. don't have hand so don't know direction
  }
  
}

// Pinch: lock ring finger and pinkie upright then actuate other fingers 100%
void pinch(){
  activate_solenoid(sol_other);                                // Activate breaking for ring finger and pinkie
  if (i == 1){
    i = 0;
    fingers_actuate();                                           // Move the motor CW /////////////////////////////////// change. don't have hand so don't know direction
  }
}

// Birdie: lock middle finger upright then actuate other fingers 100%
void birdie(){ 
  activate_solenoid(sol_middle);                               // Activate breaking for middle finger
  if (i == 1){
    i = 0;
    fingers_actuate();                                           // Move the motor CW /////////////////////////////////// change. don't have hand so don't know direction
  }
}

// Point: lock pointer finger upright then actuate other fingers 100%
void point(){
  activate_solenoid(sol_pointer);                              // Activate breaking for pointer finger
  if (i == 1){
    i = 0;
    fingers_actuate();                                           // Move the motor CW /////////////////////////////////// change. don't have hand so don't know direction
  }
}

// Rest: deactivate all breaking so fingers go back to neutral, upright position
void rest(){                                                   
  deactivate_all();                                            // Deactivate all breaking
  if (i == 1){
    i = 0;
    fingers_rest();                                           // Move the motor CW /////////////////////////////////// change. don't have hand so don't know direction
  }
}

/////////////////////////////////////////////////////////////////
// MOTOR FUNCTIONS

// Actuate fingers so they are at full grasp
void fingers_actuate(){
  motor_moveccw();
  delay(740);
  motor_stop();
}

// Bring fingers back to resting position
void fingers_rest(){
  motor_movecw();
  delay(740);
  motor_stop();
}

// Move the servo motor clockwise
void motor_movecw(){
  myservo.write(0);                                         // Move counterclockwise
}

// Move the servo motor counterclockwise
void motor_moveccw(){
  myservo.write(180);                                         // Move counterclockwise
}

// Stop motor
void motor_stop(){
  myservo.write(90);                                          // Stop motor
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
  digitalWrite(sol_pointer, LOW);                              // Switch Solenoid OFF
  digitalWrite(sol_middle, LOW);                               // Switch Solenoid OFF
  digitalWrite(sol_other, LOW);                                // Switch Solenoid OFF
  motor_stop();
}
