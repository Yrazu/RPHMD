// ROPROS Robotic Hand Code!
/* Last Updated: 3/16/2023 by Talise Lindorf
 * Created by: Maddie Kogelis 
 * Supported by: the rest of the Ropros Gang
 */
/////////////////////////////////////////////////////////////////

#include <Servo.h>                                            // Include servo library
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128                                      // OLED display width, in pixels
#define SCREEN_HEIGHT 64                                      // OLED display height, in pixels

#define OLED_RESET     -1                                     // Reset pin # (-1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

enum State {GRASP, PINCH, BIRDIE, POINT, REST};               // Enum for state machine cases
State state = REST;                                           // Keeps track of current state

int sol_pointer = 5;                                          // Pointer finger breaking digital output pin
int sol_middle = 6;                                           // Middle finger breaking digital output pin
int sol_other = 4;                                            // Pinkie and ring finger breaking digital output pin

int fsr_analog_pin = 2;                                       // Analog pin reading FSR data
int fsr_reading = 0;                                          // Analog reading from FSR
int fsr_threshold = 50;                                       // Max allowed value from the FSR (approximated in testing)
bool fsr_status = 0;
bool fsr_prev = 0;
int change_state = 0;

// Declare image bitmap arrays (128x64)
const unsigned char bitmap_birdie [] PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x01, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x02, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x04, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x08, 0xe2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x09, 0xf2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x09, 0xf2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x09, 0xf2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x09, 0xf2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x09, 0xf2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x09, 0xf2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x09, 0xf2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x09, 0xf2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x09, 0xf2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x09, 0xf2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x09, 0xf2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x09, 0xf2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x09, 0xf2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x01, 0xf1, 0xf9, 0xf3, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x02, 0x0a, 0x09, 0xf2, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x04, 0x04, 0x01, 0xf0, 0x05, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x08, 0xe0, 0xe1, 0xf0, 0xe2, 0x08, 0x3f, 0xf0, 0x1c, 0x00, 0x00, 0x00, 0xf0, 0x70, 0x00, 0x00, 
  0x09, 0xf1, 0xf1, 0xf1, 0xf0, 0x04, 0x3f, 0xf0, 0x1c, 0x00, 0x00, 0x00, 0xf0, 0x70, 0x00, 0x00, 
  0x11, 0xf1, 0xf1, 0xf1, 0xf0, 0xe2, 0x38, 0x3c, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x00, 0x00, 0x00, 
  0x13, 0xfb, 0xfb, 0xfb, 0xf1, 0xf2, 0x38, 0x3c, 0x7c, 0x0e, 0x7c, 0x3f, 0xf1, 0xf0, 0x1f, 0xf0, 
  0x13, 0xfb, 0xfb, 0xfb, 0xf1, 0xf2, 0x38, 0x3c, 0x7c, 0x0e, 0x7c, 0x3f, 0xf1, 0xf0, 0x1f, 0xf0, 
  0x13, 0xff, 0xff, 0xff, 0xf1, 0xf2, 0x3f, 0xf0, 0x1c, 0x0f, 0x80, 0xe0, 0xf0, 0x70, 0x78, 0x38, 
  0x13, 0xff, 0xff, 0xff, 0xf1, 0xf2, 0x3f, 0xf0, 0x1c, 0x0f, 0x80, 0xe0, 0xf0, 0x70, 0x78, 0x38, 
  0x13, 0xff, 0xff, 0xff, 0xf1, 0xf2, 0x38, 0x3c, 0x1c, 0x0f, 0x00, 0xe0, 0xf0, 0x70, 0x7f, 0xf8, 
  0x13, 0xff, 0xff, 0xff, 0xf1, 0xf2, 0x38, 0x3c, 0x1c, 0x0f, 0x00, 0xe0, 0xf0, 0x70, 0x7f, 0xf8, 
  0x13, 0xff, 0xff, 0xff, 0xf1, 0xf2, 0x38, 0x3c, 0x1c, 0x0f, 0x00, 0xe0, 0xf0, 0x70, 0x78, 0x00, 
  0x13, 0xff, 0xff, 0xff, 0xf1, 0xf2, 0x3f, 0xf1, 0xff, 0xcf, 0x00, 0x3f, 0xf3, 0xfe, 0x1f, 0xf0, 
  0x13, 0xff, 0xff, 0xff, 0xf1, 0xf2, 0x3f, 0xf1, 0xff, 0xcf, 0x00, 0x3f, 0xf3, 0xfe, 0x1f, 0xf0, 
  0x13, 0xff, 0xff, 0xff, 0xf1, 0xf2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x13, 0xff, 0xff, 0xff, 0xf1, 0xf2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x13, 0xff, 0xff, 0xff, 0xf1, 0xf2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x13, 0xff, 0xff, 0xff, 0xf1, 0xf2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x13, 0xff, 0xff, 0xff, 0xfb, 0xf2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x13, 0xff, 0xff, 0xff, 0xfb, 0xf2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x13, 0xff, 0xff, 0xff, 0xff, 0xf2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x13, 0xff, 0xff, 0xff, 0xff, 0xf2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x13, 0xff, 0xff, 0xff, 0xff, 0xf2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x13, 0xff, 0xff, 0xff, 0xff, 0xf2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x13, 0xff, 0xff, 0xff, 0xff, 0xf2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x13, 0xff, 0xff, 0xff, 0xff, 0xe2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x13, 0xff, 0xff, 0xff, 0xff, 0xe4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x13, 0xff, 0xff, 0xff, 0xff, 0xc4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x13, 0xff, 0xff, 0xff, 0xff, 0x88, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x11, 0xff, 0xff, 0xff, 0xff, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x09, 0xff, 0xff, 0xff, 0xfe, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x08, 0xff, 0xff, 0xff, 0xfc, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x04, 0x7f, 0xff, 0xff, 0xf8, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x02, 0x3f, 0xff, 0xff, 0xf1, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x01, 0x1f, 0xff, 0xff, 0xe2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x87, 0xff, 0xff, 0x84, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x40, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x30, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x0f, 0xff, 0xff, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
const unsigned char bitmap_grasp [] PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x7c, 0x7c, 0x7c, 0x7c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x82, 0x82, 0x82, 0x82, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x02, 0x38, 0x38, 0x38, 0x38, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x02, 0x7c, 0x7c, 0x7c, 0x7c, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x04, 0x7c, 0x7c, 0x7c, 0x7c, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x04, 0xfe, 0xfe, 0xfe, 0xfc, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x04, 0xfe, 0xfe, 0xfe, 0xfc, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x04, 0xff, 0xff, 0xff, 0xfc, 0x60, 0x03, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x04, 0xff, 0xff, 0xff, 0xfc, 0x10, 0x03, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x04, 0xff, 0xff, 0xff, 0xfc, 0x08, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x04, 0xff, 0xff, 0xff, 0xfd, 0xc4, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x04, 0xff, 0xff, 0xff, 0xfd, 0xe4, 0x3c, 0x00, 0x0f, 0x3f, 0x3f, 0xf0, 0x3f, 0xf0, 0xff, 0xf0, 
  0x04, 0xff, 0xff, 0xff, 0xfd, 0xe4, 0x3c, 0x00, 0x0f, 0x3f, 0x3f, 0xf0, 0x3f, 0xf0, 0xff, 0xf0, 
  0x04, 0xff, 0xff, 0xff, 0xfd, 0xe4, 0x3c, 0x3f, 0x0f, 0xc0, 0x00, 0x3c, 0xf0, 0x00, 0xf0, 0x3c, 
  0x04, 0xff, 0xff, 0xff, 0xfc, 0xe4, 0x3c, 0x3f, 0x0f, 0xc0, 0x00, 0x3c, 0xf0, 0x00, 0xf0, 0x3c, 
  0x04, 0xff, 0xff, 0xff, 0xfc, 0xe4, 0x3c, 0x0f, 0x0f, 0x00, 0x3f, 0xfc, 0x3f, 0xf0, 0xf0, 0x3c, 
  0x04, 0xff, 0xff, 0xff, 0xfc, 0xe4, 0x3c, 0x0f, 0x0f, 0x00, 0x3f, 0xfc, 0x3f, 0xf0, 0xf0, 0x3c, 
  0x04, 0xff, 0xff, 0xff, 0xfc, 0xe4, 0x0f, 0x0f, 0x0f, 0x00, 0xc0, 0x3c, 0x00, 0x3c, 0xff, 0xf0, 
  0x04, 0xff, 0xff, 0xff, 0xfc, 0xe4, 0x0f, 0x0f, 0x0f, 0x00, 0xc0, 0x3c, 0x00, 0x3c, 0xff, 0xf0, 
  0x04, 0xff, 0xff, 0xff, 0xfc, 0xe4, 0x03, 0xff, 0x0f, 0x00, 0x3f, 0xfc, 0xff, 0xf0, 0xf0, 0x00, 
  0x04, 0xff, 0xff, 0xff, 0xfe, 0xe4, 0x03, 0xff, 0x0f, 0x00, 0x3f, 0xfc, 0xff, 0xf0, 0xf0, 0x00, 
  0x04, 0xff, 0xff, 0xff, 0xfe, 0xe4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x00, 
  0x04, 0xff, 0xff, 0xff, 0xff, 0xe4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x00, 
  0x04, 0xff, 0xff, 0xff, 0xff, 0xe4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x00, 
  0x04, 0xff, 0xff, 0xff, 0xff, 0xe4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x00, 
  0x04, 0xff, 0xff, 0xff, 0xff, 0xe4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x04, 0xff, 0xff, 0xff, 0xff, 0xe4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x04, 0xff, 0xff, 0xff, 0xff, 0xe4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x04, 0xff, 0xff, 0xff, 0xff, 0xc4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x04, 0x7f, 0xff, 0xff, 0xff, 0xc8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x02, 0x7f, 0xff, 0xff, 0xff, 0x88, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x02, 0x3f, 0xff, 0xff, 0xff, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x01, 0x1f, 0xff, 0xff, 0xfe, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x8f, 0xff, 0xff, 0xfc, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x47, 0xff, 0xff, 0xf8, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x21, 0xff, 0xff, 0xe1, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x10, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x0c, 0x00, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x03, 0xff, 0xff, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
const unsigned char bitmap_pinch [] PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x01, 0xf1, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x02, 0x0a, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x04, 0x04, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x08, 0xe0, 0xe2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x09, 0xf1, 0xf2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x09, 0xf1, 0xf2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x09, 0xf1, 0xf2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x09, 0xf1, 0xf2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x09, 0xf1, 0xf2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x09, 0xf1, 0xf2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x09, 0xf1, 0xf2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x09, 0xf1, 0xf2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x09, 0xf1, 0xf2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x09, 0xf1, 0xf2, 0x3f, 0xff, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x09, 0xf1, 0xf2, 0x40, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x09, 0xf1, 0xf2, 0x80, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x09, 0xf1, 0xf3, 0x1f, 0xbf, 0x88, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x09, 0xf1, 0xf2, 0x3f, 0x7f, 0xc4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x09, 0xf1, 0xf0, 0x7e, 0xff, 0xe4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x09, 0xf1, 0xf0, 0xfd, 0xff, 0xe2, 0x3f, 0xfc, 0x03, 0xc0, 0x00, 0x00, 0x00, 0x0f, 0x00, 0x00, 
  0x09, 0xf1, 0xf1, 0xfb, 0xf1, 0xf2, 0x3f, 0xfc, 0x03, 0xc0, 0x00, 0x00, 0x00, 0x0f, 0x00, 0x00, 
  0x09, 0xf1, 0xf1, 0xf3, 0xf6, 0xf2, 0x3c, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0x00, 0x00, 
  0x11, 0xf1, 0xf3, 0xf7, 0xee, 0xf2, 0x3c, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0x00, 0x00, 
  0x13, 0xfb, 0xfb, 0xf7, 0xee, 0xf2, 0x3c, 0x0f, 0x0f, 0xc0, 0xff, 0xc0, 0xff, 0xcf, 0xfc, 0x00, 
  0x13, 0xfb, 0xfb, 0xf7, 0xee, 0xf2, 0x3c, 0x0f, 0x0f, 0xc0, 0xff, 0xc0, 0xff, 0xcf, 0xfc, 0x00, 
  0x13, 0xff, 0xff, 0xff, 0xde, 0xe2, 0x3c, 0x0f, 0x03, 0xc0, 0xf0, 0xf3, 0xc0, 0x0f, 0x0f, 0x00, 
  0x13, 0xff, 0xff, 0xff, 0xcc, 0x02, 0x3c, 0x0f, 0x03, 0xc0, 0xf0, 0xf3, 0xc0, 0x0f, 0x0f, 0x00, 
  0x13, 0xff, 0xff, 0xff, 0xc0, 0x71, 0x3f, 0xfc, 0x03, 0xc0, 0xf0, 0xf3, 0xc0, 0x0f, 0x0f, 0x00, 
  0x13, 0xff, 0xff, 0xff, 0xc0, 0xf9, 0x3f, 0xfc, 0x03, 0xc0, 0xf0, 0xf3, 0xc0, 0x0f, 0x0f, 0x00, 
  0x13, 0xff, 0xff, 0xff, 0xc0, 0xf9, 0x3c, 0x00, 0x03, 0xc0, 0xf0, 0xf3, 0xc0, 0x0f, 0x0f, 0x00, 
  0x13, 0xff, 0xff, 0xff, 0xc0, 0xf9, 0x3c, 0x00, 0x03, 0xc0, 0xf0, 0xf3, 0xc0, 0x0f, 0x0f, 0x00, 
  0x13, 0xff, 0xff, 0xff, 0xc0, 0xf9, 0x3c, 0x00, 0x3f, 0xfc, 0xf0, 0xf0, 0xff, 0xcf, 0x0f, 0x00, 
  0x13, 0xff, 0xff, 0xff, 0xc0, 0xf9, 0x3c, 0x00, 0x3f, 0xfc, 0xf0, 0xf0, 0xff, 0xcf, 0x0f, 0x00, 
  0x13, 0xff, 0xff, 0xff, 0xe1, 0xf9, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x13, 0xff, 0xff, 0xff, 0xe1, 0xf9, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x13, 0xff, 0xff, 0xff, 0xe1, 0xf9, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x13, 0xff, 0xff, 0xff, 0xf3, 0xf9, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x13, 0xff, 0xff, 0xff, 0xf3, 0xf9, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x13, 0xff, 0xff, 0xff, 0xff, 0xf1, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x13, 0xff, 0xff, 0xff, 0xff, 0xf1, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x13, 0xff, 0xff, 0xff, 0xff, 0xe2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x13, 0xff, 0xff, 0xff, 0xff, 0xc4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x13, 0xff, 0xff, 0xff, 0xff, 0xc4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x13, 0xff, 0xff, 0xff, 0xff, 0x88, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x13, 0xff, 0xff, 0xff, 0xff, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x13, 0xff, 0xff, 0xff, 0xfe, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x13, 0xff, 0xff, 0xff, 0xfc, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x11, 0xff, 0xff, 0xff, 0xf8, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x09, 0xff, 0xff, 0xff, 0xf1, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x08, 0xff, 0xff, 0xff, 0xe2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x04, 0x7f, 0xff, 0xff, 0xe2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x02, 0x3f, 0xff, 0xff, 0xc4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x01, 0x1f, 0xff, 0xff, 0xc4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x87, 0xff, 0xff, 0x88, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x40, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x30, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x0f, 0xff, 0xff, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
const unsigned char bitmap_point [] PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x01, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x02, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x04, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x08, 0xe2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x09, 0xf2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x09, 0xf2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x09, 0xf2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x09, 0xf2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x09, 0xf2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x09, 0xf2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x09, 0xf2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x09, 0xf2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x09, 0xf2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x09, 0xf2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x09, 0xf2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x09, 0xf2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x09, 0xf2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x01, 0xf1, 0xf1, 0xf9, 0xf2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x02, 0x0a, 0x0a, 0x09, 0xf2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x04, 0x04, 0x04, 0x01, 0xf3, 0xf0, 0x3f, 0xfc, 0x00, 0x00, 0x03, 0xc0, 0x00, 0x00, 0x3c, 0x00, 
  0x08, 0xe0, 0xe0, 0xe1, 0xf2, 0x08, 0x3f, 0xfc, 0x00, 0x00, 0x03, 0xc0, 0x00, 0x00, 0x3c, 0x00, 
  0x09, 0xf1, 0xf1, 0xf1, 0xf0, 0x04, 0x3c, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3c, 0x00, 
  0x11, 0xf1, 0xf1, 0xf1, 0xf0, 0xe2, 0x3c, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3c, 0x00, 
  0x13, 0xfb, 0xfb, 0xfb, 0xf1, 0xf2, 0x3c, 0x0f, 0x0f, 0xfc, 0x0f, 0xc0, 0xff, 0xc3, 0xff, 0xc0, 
  0x13, 0xfb, 0xfb, 0xfb, 0xf1, 0xf2, 0x3c, 0x0f, 0x0f, 0xfc, 0x0f, 0xc0, 0xff, 0xc3, 0xff, 0xc0, 
  0x13, 0xff, 0xff, 0xff, 0xf1, 0xf2, 0x3c, 0x0f, 0x3c, 0x0f, 0x03, 0xc0, 0xf0, 0xf0, 0x3c, 0x00, 
  0x13, 0xff, 0xff, 0xff, 0xf1, 0xf2, 0x3c, 0x0f, 0x3c, 0x0f, 0x03, 0xc0, 0xf0, 0xf0, 0x3c, 0x00, 
  0x13, 0xff, 0xff, 0xff, 0xf1, 0xf2, 0x3f, 0xfc, 0x3c, 0x0f, 0x03, 0xc0, 0xf0, 0xf0, 0x3c, 0x00, 
  0x13, 0xff, 0xff, 0xff, 0xf1, 0xf2, 0x3f, 0xfc, 0x3c, 0x0f, 0x03, 0xc0, 0xf0, 0xf0, 0x3c, 0x00, 
  0x13, 0xff, 0xff, 0xff, 0xf1, 0xf2, 0x3c, 0x00, 0x3c, 0x0f, 0x03, 0xc0, 0xf0, 0xf0, 0x3c, 0x00, 
  0x13, 0xff, 0xff, 0xff, 0xf1, 0xf2, 0x3c, 0x00, 0x3c, 0x0f, 0x03, 0xc0, 0xf0, 0xf0, 0x3c, 0x00, 
  0x13, 0xff, 0xff, 0xff, 0xf1, 0xf2, 0x3c, 0x00, 0x0f, 0xfc, 0x3f, 0xfc, 0xf0, 0xf0, 0x3c, 0x00, 
  0x13, 0xff, 0xff, 0xff, 0xf1, 0xf2, 0x3c, 0x00, 0x0f, 0xfc, 0x3f, 0xfc, 0xf0, 0xf0, 0x3c, 0x00, 
  0x13, 0xff, 0xff, 0xff, 0xf1, 0xf2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x13, 0xff, 0xff, 0xff, 0xf1, 0xf2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x13, 0xff, 0xff, 0xff, 0xf1, 0xf2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x13, 0xff, 0xff, 0xff, 0xfb, 0xf2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x13, 0xff, 0xff, 0xff, 0xfb, 0xf2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x13, 0xff, 0xff, 0xff, 0xff, 0xf2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x13, 0xff, 0xff, 0xff, 0xff, 0xf2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x13, 0xff, 0xff, 0xff, 0xff, 0xf2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x13, 0xff, 0xff, 0xff, 0xff, 0xf2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x13, 0xff, 0xff, 0xff, 0xff, 0xf2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x13, 0xff, 0xff, 0xff, 0xff, 0xe2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x13, 0xff, 0xff, 0xff, 0xff, 0xe4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x13, 0xff, 0xff, 0xff, 0xff, 0xc4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x13, 0xff, 0xff, 0xff, 0xff, 0x88, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x11, 0xff, 0xff, 0xff, 0xff, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x09, 0xff, 0xff, 0xff, 0xfe, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x08, 0xff, 0xff, 0xff, 0xfc, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x04, 0x7f, 0xff, 0xff, 0xf8, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x02, 0x3f, 0xff, 0xff, 0xf1, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x01, 0x1f, 0xff, 0xff, 0xe2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x87, 0xff, 0xff, 0x84, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x40, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x30, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x0f, 0xff, 0xff, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
const unsigned char bitmap_rest [] PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x01, 0xf1, 0xf1, 0xf1, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x02, 0x0a, 0x0a, 0x0a, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x04, 0x04, 0x04, 0x04, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x08, 0xe0, 0xe0, 0xe0, 0xe2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x09, 0xf1, 0xf1, 0xf1, 0xf2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x09, 0xf1, 0xf1, 0xf1, 0xf2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x09, 0xf1, 0xf1, 0xf1, 0xf2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x09, 0xf1, 0xf1, 0xf1, 0xf2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x09, 0xf1, 0xf1, 0xf1, 0xf2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x09, 0xf1, 0xf1, 0xf1, 0xf2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x09, 0xf1, 0xf1, 0xf1, 0xf2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x09, 0xf1, 0xf1, 0xf1, 0xf2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x09, 0xf1, 0xf1, 0xf1, 0xf2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x09, 0xf1, 0xf1, 0xf1, 0xf2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x09, 0xf1, 0xf1, 0xf1, 0xf2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x09, 0xf1, 0xf1, 0xf1, 0xf2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x09, 0xf1, 0xf1, 0xf1, 0xf2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x09, 0xf1, 0xf1, 0xf1, 0xf2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x09, 0xf1, 0xf1, 0xf1, 0xf2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x09, 0xf1, 0xf1, 0xf1, 0xf3, 0xf0, 0x3f, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x07, 0x80, 0x00, 0x00, 
  0x09, 0xf1, 0xf1, 0xf1, 0xf0, 0x08, 0x3f, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x07, 0x80, 0x00, 0x00, 
  0x09, 0xf1, 0xf1, 0xf1, 0xf0, 0x04, 0x3c, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x07, 0x80, 0x00, 0x00, 
  0x11, 0xf1, 0xf1, 0xf1, 0xf0, 0xe2, 0x3c, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x07, 0x80, 0x00, 0x00, 
  0x13, 0xfb, 0xfb, 0xfb, 0xf1, 0xf2, 0x3c, 0x0f, 0x0f, 0xfc, 0x0f, 0xfc, 0x7f, 0xf8, 0x00, 0x00, 
  0x13, 0xfb, 0xfb, 0xfb, 0xf1, 0xf2, 0x3c, 0x0f, 0x0f, 0xfc, 0x0f, 0xfc, 0x7f, 0xf8, 0x00, 0x00, 
  0x13, 0xff, 0xff, 0xff, 0xf1, 0xf2, 0x3c, 0x3f, 0x3c, 0x0f, 0x3c, 0x00, 0x07, 0x80, 0x00, 0x00, 
  0x13, 0xff, 0xff, 0xff, 0xf1, 0xf2, 0x3c, 0x3f, 0x3c, 0x0f, 0x3c, 0x00, 0x07, 0x80, 0x00, 0x00, 
  0x13, 0xff, 0xff, 0xff, 0xf1, 0xf2, 0x3f, 0xf0, 0x3f, 0xff, 0x0f, 0xfc, 0x07, 0x80, 0x00, 0x00, 
  0x13, 0xff, 0xff, 0xff, 0xf1, 0xf2, 0x3f, 0xf0, 0x3f, 0xff, 0x0f, 0xfc, 0x07, 0x80, 0x00, 0x00, 
  0x13, 0xff, 0xff, 0xff, 0xf1, 0xf2, 0x3c, 0xfc, 0x3c, 0x00, 0x00, 0x0f, 0x07, 0x80, 0x00, 0x00, 
  0x13, 0xff, 0xff, 0xff, 0xf1, 0xf2, 0x3c, 0xfc, 0x3c, 0x00, 0x00, 0x0f, 0x07, 0x80, 0x00, 0x00, 
  0x13, 0xff, 0xff, 0xff, 0xf1, 0xf2, 0x3c, 0x3f, 0x0f, 0xfc, 0x3f, 0xfc, 0x07, 0x80, 0x00, 0x00, 
  0x13, 0xff, 0xff, 0xff, 0xf1, 0xf2, 0x3c, 0x3f, 0x0f, 0xfc, 0x3f, 0xfc, 0x07, 0x80, 0x00, 0x00, 
  0x13, 0xff, 0xff, 0xff, 0xf1, 0xf2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x13, 0xff, 0xff, 0xff, 0xf1, 0xf2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x13, 0xff, 0xff, 0xff, 0xf1, 0xf2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x13, 0xff, 0xff, 0xff, 0xfb, 0xf2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x13, 0xff, 0xff, 0xff, 0xfb, 0xf2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x13, 0xff, 0xff, 0xff, 0xff, 0xf2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x13, 0xff, 0xff, 0xff, 0xff, 0xf2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x13, 0xff, 0xff, 0xff, 0xff, 0xf2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x13, 0xff, 0xff, 0xff, 0xff, 0xf2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x13, 0xff, 0xff, 0xff, 0xff, 0xf2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x13, 0xff, 0xff, 0xff, 0xff, 0xe2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x13, 0xff, 0xff, 0xff, 0xff, 0xe4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x13, 0xff, 0xff, 0xff, 0xff, 0xc4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x13, 0xff, 0xff, 0xff, 0xff, 0x88, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x11, 0xff, 0xff, 0xff, 0xff, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x09, 0xff, 0xff, 0xff, 0xfe, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x08, 0xff, 0xff, 0xff, 0xfc, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x04, 0x7f, 0xff, 0xff, 0xf8, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x02, 0x3f, 0xff, 0xff, 0xf1, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x01, 0x1f, 0xff, 0xff, 0xe2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x87, 0xff, 0xff, 0x84, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x40, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x30, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x0f, 0xff, 0xff, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

/////////////////////////////////////////////////////////////////
// INITIALIZE PROGRAM
void setup() {          
  Serial.begin(115200);                                       // Set baud rate for serial communication
  
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  
  Serial.println("Program loaded.");                          // Print that program is loaded

  pinMode(sol_pointer, OUTPUT);                               // Set solenoid pin as digital output
  pinMode(sol_middle, OUTPUT);                                // Set solenoid pin as digital output
  pinMode(sol_other, OUTPUT);                                 // Set solenoid pin as digital output

  // Show initial display buffer Adafruit splash screen
  display.display();
  delay(2000);

  // Clear the buffer
  display.clearDisplay();
  delay(200);
}

/////////////////////////////////////////////////////////////////
// MAIN LOOP
void loop() {
  mode_state_machine();                                       // Call the state machine to activate the desired state
}

/////////////////////////////////////////////////////////////////
// STATE MACHINE MODE FUNCTION
void mode_state_machine(){
  change_state = read_fsr();
  switch(state){                                              // Switch cases according to the state variable
    case GRASP:                                               // Grasp: actuate all fingers to grasp an object
      //grasp();                                              // Call the state handler function for grasping
      if (change_state == 1){
        Serial.println("Change to pinch");
        //rest();
        state = PINCH;
      }
      break; 
    case PINCH:                                               // Pinch: lock ring finger and pinkie in the upright position then actuate middle and pointer fingers
      //pinch();                                              // Call the state handler function for pinching
      if (change_state == 1){
        Serial.println("Change to birdie");
        //rest();
        state = BIRDIE;
      }
      break;
    case BIRDIE:                                              // Birdie: lock middle finger and actuate others
      //birdie();                                             // Call the state handler function for birdie
      if (change_state == 1){
        Serial.println("Change to point");
        //rest();
        state = POINT;
      }
      break;
    case POINT:                                               // Point: lock pointer finger and actuate others
      //point();                                              // Call the state handler function for pointing
      if (change_state == 1){
        Serial.println("Change to rest");
        state = REST;
      }
      break;
    case REST:                                                // Rest: release all breaks and allow reverse motor back to neutral position
      //rest();                                               // Call the state handler function for resting
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
void grasp() {
  displayImage(bitmap_grasp);                                 // Show grasp image on OLED display
}

// Pinch: lock ring finger and pinkie upright then actuate other fingers 100%
void pinch() {
  displayImage(bitmap_pinch);                                 // Show pinch image on OLED display
  activate_solenoid(sol_other);                               // Activate breaking for ring finger and pinkie
}

// Birdie: lock middle finger upright then actuate other fingers 100%
void birdie() { 
  displayImage(bitmap_birdie);                                // Show birdie image on OLED display
  activate_solenoid(sol_middle);                              // Activate breaking for middle finger
}

// Point: lock pointer finger upright then actuate other fingers 100%
void point() {
  displayImage(bitmap_point);                                 // Show point image on OLED display
  activate_solenoid(sol_pointer);                             // Activate breaking for pointer finger
}

// Rest: deactivate all breaking so fingers go back to neutral, upright position
void rest() { 
  displayImage(bitmap_rest);                                  // Show rest image on OLED display                                                  
  deactivate_all();                                           // Deactivate all breaking
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

/////////////////////////////////////////////////////////////////
// SOLENOID FUNCTIONS

// Activate a solenoid
void activate_solenoid(int solenoidPin){
  digitalWrite(solenoidPin, HIGH);                            // Switch Solenoid ON
}

// Deactivate a solenoid
void deactivate_solenoid(int solenoidPin){
  digitalWrite(solenoidPin, LOW);                             // Switch Solenoid OFF
}

// Deactivate all solenoids
void deactivate_all(){
  digitalWrite(sol_pointer, LOW);                             // Switch Solenoid OFF
  digitalWrite(sol_middle, LOW);                              // Switch Solenoid OFF
  digitalWrite(sol_other, LOW);                               // Switch Solenoid OFF
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