// Solenoid Variables
#define SOL_POINTER     5                                      // Pointer finger digital output pin
#define SOL_MIDDLE      6                                      // Middle finger digital output pin
#define SOL_OTHER       4                                      // Pinkie and ring finger digital output pin


void setup() {
  pinMode(SOL_POINTER, OUTPUT);
  pinMode(SOL_MIDDLE, OUTPUT);
  pinMode(SOL_OTHER, OUTPUT);

  // Initialize Serial Communication   
  Serial.begin(115200);                                        // Set baud rate for serial communication
  while (!Serial)                                              // Loop until serial data is recieved
  ;
  Serial.println("Program loaded.");                           // Print that program is loaded
}

void loop() {
  digitalWrite(SOL_OTHER, LOW);
  Serial.println("middle finger");
  digitalWrite(SOL_MIDDLE, HIGH);
  delay(3000);
  
  digitalWrite(SOL_MIDDLE, LOW);
  Serial.println("pointer finger");
  digitalWrite(SOL_POINTER, HIGH);
  delay(3000);

  digitalWrite(SOL_POINTER, LOW);
  Serial.println("other fingers");
  digitalWrite(SOL_OTHER, HIGH);
  delay(3000);
}
