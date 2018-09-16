//////////////////////////////////////////////////////////////////////////////////////////////////////////
//  STACKDUINO 1.2 (BETA)                                                                               //
//                                                                                                      //
//  https://github.com/ReallySmall/Stackduino-2                                                         //
//                                                                                                      //
//  An Arduino compatible Focus Stacking Controller                                                     //
//  Richard Iles 2017                                                                                   //
//                                                                                                      //
//  Written for use with the Digole 128 x 64 OLED module using default font (4 rows of 16 chars)        //
//  To support other fonts or screens, some recoding will be required                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////////*/

/* YOUR HARDWARE SETTINGS */
const float HARDWARE_CONSTANT = 0.2; // a number which on your system results in one step of the motor resulting in one micron of movement
const byte MICRO_STEPS = 16; // the microstepping amount set on the stepper driver

/* DEFINES AND DEPENDENCIES (you probably don't need to change these */
#define _Digole_Serial_I2C_ // OLED screen configured with solder jumper to run in I2C mode
#define OLED_ROWS 4 // OLED text rows
#define OLED_COLS 16 // OLED text columns
#define ENC_A A0 // Rotary encoder
#define ENC_B A1 // Rotary encoder
#define ENC_PORT PINC // Rotary encoder
#define settings_elements 4 // Number of properties in each settings

#include "DigoleSerial.h" // https://github.com/chouckz/HVACX10Arduino/blob/master/1.0.1_libraries/DigoleSerial/DigoleSerial.h
#include <Wire.h> //
#include <EEPROM.h>
#include "avr/pgmspace.h"


/* CREATE REQUIRED OBJECTS */
DigoleSerialDisp screen(&Wire, '\x27'); // Digole OLED in i2c mode


typedef struct { // A struct type for storing char arrays
  char title [OLED_COLS + 1];
} stringConstants;

const stringConstants settings_titles[] PROGMEM = { // A struct of char arrays stored in Flash
  {"Stackduino v1.2"},
  {"Slice size"},
  {"Slices"},
  {"Pause for"},
  {"Mirror lockup"},
  {"Bracketing"},
  {"Return to home"},
  {"Units"},
  {"Motor speed"},
  {"Bluetooth"}
};

char uom_chars[3] = {'u', 'm', 'c'};

struct SettingsDesc { // A struct type for storing settings

  byte lower; // The lowest value the setting may have
  int upper; // The highest value the setting may have
  byte multiplier; // Any multiplier to apply when the setting is incremented

} settingsDesc[] = { // A struct of user menu settings
  {}, // The home screen - not actually used by any functions at the moment, but the placeholder array must exist to maintain the correct indexing
  {1, 500, 1}, // "Slice size"
  {5, 500, 5}, // "Number of slices"
  {0, 60, 1}, // "Pause time"
  {0, 1, 1}, // "Mirror lockup"
  {1, 10, 1}, // "Bracketing"
  {0, 1, 1}, // "Return to start"
  {0, 2, 1}, // "Unit of measure"
  {5, 20, 1} // "Stepper speed"
};

int settingsValue[] = {
  0,
  1,
  5,
  2,
  0,
  1,
  0,
  0,
  1
};

byte settings_count = sizeof(settingsDesc) / sizeof(SettingsDesc); // The number of settings
char char_buffer[OLED_COLS + 1]; // make sure this is large enough for the largest string it must hold



/* SET UP REMAINING ATMEGA PINS */
const byte main_button = 3;  //start/ stop stack button
const byte encoder_button = 2; //select/ unselect menu item button
const byte step_dir = 4; // Direction pin on A4988 stepper driver
const byte do_step = 5; // Step pin on A4988 stepper driver
const byte cam_focus = 6; // Camera autofocus signal
const byte cam_shutter = 7; // Camera shutter signal
const byte stepper_enable = 8; // Enable/ disable the stepper driver
const byte btn_bwd = A2; // Manual forward button
const byte btn_fwd = A3; // Manual backward button
const byte limit_switch = 9;


/* SETTINGS */
const int uom_multipliers[] = {1, 1000, 10000}; // Multipliers for active unit of measure (mn, mm or cm)

boolean app_connected = false; // Whether a remote application is actively communicating with the controller
boolean can_disable_stepper = true; // Whether to disable the A4988 stepper driver when possible to save power and heat
boolean update_display = true; // True whenever functions are called which change a setting's value
boolean update_header = true; // True whenever the value of an item in the header changes
boolean home_screen = true;
boolean previous_direction;

int menu_item = 0; // The active menu item
int increments = 0; // Count increments the active menu setting should be changed by on the next poll
byte slice_count = 0; // Count of number of focus slices made so far in the stack

//main button toggle
volatile int main_button_state = LOW; //the current state of the output pin
volatile int main_button_reading; //the current reading from the input pin
volatile int main_button_previous = HIGH; //the previous reading from the input pin
volatile long main_button_time = 0; //the last time the output pin was toggled
volatile long main_button_debounce = 400; //the debounce time, increase if the output flickers

//rotary pushButton toggle
volatile int rotary_button_state = HIGH; //the current state of the output pin
volatile int rbreading; //the current reading from the input pin
volatile int rbprevious = LOW; //the previous reading from the input pin
volatile long rbtime = 0; //the last time the output pin was toggled
volatile long rbdebounce = 400; //the debounce time, increase if the output flickers

unsigned long time_stack_started;

char* app_conn_icon = "";

volatile byte btn_reading; // The current reading from the button
volatile byte btn_previous = LOW; // The previous reading from the button
volatile long btn_time = 0; // The last time the button was toggled


void loadSettings(){
  byte value = EEPROM.read(0);  
  if (value != 2) return;

  EEPROM.get(1, settingsValue);
}

void saveSettings(){
  EEPROM.put(1, settingsValue);
  EEPROM.update(0, 2);
}


/* SETUP() */
void setup() {

  /* SET ATMEGA PINMODES AND PULLUPS */
  pinMode(encoder_button, INPUT); digitalWrite(encoder_button, HIGH);
  pinMode(main_button, INPUT); digitalWrite(main_button, HIGH);
  pinMode(ENC_A, INPUT); digitalWrite(ENC_A, HIGH);
  pinMode(ENC_B, INPUT); digitalWrite(ENC_B, HIGH);
  pinMode(limit_switch, INPUT); digitalWrite(limit_switch, HIGH);
  
  pinMode(step_dir, OUTPUT); digitalWrite(step_dir, HIGH);
  pinMode(do_step, OUTPUT); digitalWrite(do_step, LOW);
  pinMode(cam_focus, OUTPUT); digitalWrite(cam_focus, LOW);
  pinMode(cam_shutter, OUTPUT); digitalWrite(cam_shutter, LOW);
  pinMode(btn_fwd, OUTPUT); digitalWrite(btn_fwd, HIGH);
  pinMode(btn_bwd, OUTPUT); digitalWrite(btn_bwd, HIGH);  

  /* SET UP INTERRUPTS */
  attachInterrupt(1, button_main_change, CHANGE);  //main button on interrupt 1
  attachInterrupt(0, button_rotary_change, CHANGE);  //encoder button on interrupt 0

  pciSetup(ENC_A); // ATMega pin change interrupt
  pciSetup(ENC_B); // ATMega pin change interrupt
  
  Wire.begin(); // join i2c bus
  Serial.begin(9600); // Start serial (always initialise at 9600 for compatibility with OLED)

  loadSettings();
}



/*////////////////////////////////////////////////////////////////////////////////////////////////////////
// FUNCTIONS                                                                                            //
////////////////////////////////////////////////////////////////////////////////////////////////////////*/



/* UPDATE THE SCREEN WHEN DATA CHANGES
*
* Clears the screen then re-outputs it
*
*/
void screenUpdate() { /* WIPE THE SCREEN, PRINT THE HEADER AND SET THE CURSOR POSITION */

  screen.clearScreen(); // Repaints the screen blank
  screen.drawBox(1, 17, 128, 1); // Draw a horizontal line to mark out the header section
  screen.setPrintPos(0, 2); // Set text position to beginning of content area

}



/* PRINT TEXT ON DIGOLE OLED SCREEN
*
* Assumes the default font is in use (allowing 4 rows of 16 characters)
*
* char_length => The length of the string in the char buffer
* text => The string to be centered and printed
* print_pos_y => The text line to print on
*
*/
void screenPrint(byte char_length, char* text, byte print_pos_y = 4) {

  // Calculate the start point on x axis for the number of characters in the string to print it centrally
  byte print_pos_x = (OLED_COLS - char_length) / 2;
  
  screen.setPrintPos(print_pos_x, print_pos_y); // Set the start point for printing
  
  byte offset_x = char_length % 2 != 0 ? 4 : 0; // If the string to print is an odd number in length, apply an offset to centralise it
  byte offset_y = print_pos_y == 4 ? -2 : 0; // If printing on line 4, nudge upwards a bit to prevent lower parts of characters like 'g' being cut off

  screen.setTextPosOffset(offset_x, offset_y); // Set the required positioning offsets defined above
  
  screen.print(text); // Finally, print the centered text to the screen
  
}



/* ENABLE OR DISABLE THE STEPPER DRIVER AND SET DIRECTION
*
* enable => Enables the stepper driver if true, Disables if false and option set
* direction => Set as 0 for forward direction, 1 for backward direction
* toggle_direction => Toggle the direction to be the opposite of its current state
*
*/
void stepperDriverEnable(boolean enable = true, byte direction = 0, boolean toggle_direction = false) {

  if (enable) {
    digitalWrite(stepper_enable, LOW); // Enable the stepper driver
    
    if (toggle_direction) { // Toggle the direction to set (used by limit switches)
      direction = !previous_direction;
    }
    
    digitalWrite(step_dir, !direction); // Set the direction
    previous_direction = direction; // Set the new direction for future toggle_direction calls
    
  }
  else {
    digitalWrite(stepper_enable, HIGH); // Disable the stepper driver
  }
  

}



/* MOVE STAGE FORWARD BY ONE SLICE */
void stepperMoveOneSlice(byte direction = 1){

  int slice_size = settingsValue[1];
  float hardware = HARDWARE_CONSTANT; 
  int unit_of_measure = uom_multipliers[settingsValue[7]];
  byte micro_steps = MICRO_STEPS;

  unsigned int rounded_steps = (slice_size * hardware * micro_steps * unit_of_measure) + 0.5;
  Serial.println(rounded_steps);
  Serial.println(direction);
  
  stepperDriverEnable(true, direction, false); // Enable the stepper driver

  for (unsigned int i = 0; i < rounded_steps; i++) { // Move the required number of steps 

    stepperDriverStep(); // Send a step signal to the stepper driver
    if (!stepperDriverInBounds() || stackCancelled()) break; // Exit early if the stack has been cancelled or a limit switch is hit

  }
  
  stepperDriverEnable(false); // Disable the stepper driver

}


/* MOVE STAGE BACKWARD AND FORWARD */
void stepperDriverManualControl(byte direction = 1, boolean serial_control = false, boolean short_press = false) {

    if ((digitalRead(btn_fwd) == LOW || digitalRead(btn_bwd) == LOW || serial_control == true) && stepperDriverInBounds()) {
      
      unsigned long button_down = millis();
      char* manual_ctl_strings[2] = {"<", ">"};
          
      screenUpdate();
      screenPrint(sprintf_P(char_buffer, PSTR("Moving stage")), char_buffer, 2);
      screen.setPrintPos(4, 4);
      for (byte i = 0; i < 8; i++) {
        screen.print(manual_ctl_strings[direction]);
      }

      stepperDriverEnable(true, direction); // Enable the stepper driver and set the direction
      
      // Move the stage for as long as the control button is pressed
      while (stepperDriverInBounds() && Serial.available() == 0 && (digitalRead(btn_fwd) == LOW || digitalRead(btn_bwd) == LOW)) {
         
          
          stepperDriverStep();
          if (!stepperDriverInBounds()) stepperDriverClearLimitSwitch();
          
       }

    stepperDriverEnable(false); // Disable the stepper driver
    update_display = true; // Go back to displaying the active menu item once manual control button is no longer pressed
    
    }

}



/* APPLICATION CONNECTION STATUS
*
* NOT CURRENTLY USED
*
* Sends a keep-alive token and expects one back the next time it is called
* Otherwise assumes the connection with the remote application has been lost
* Also sets the icon for current connection status
*
*/
void appConnection() {

  static boolean last_app_conn_stat;

  //app_conn_icon = app_connected == true ? : ;

  if (app_connected != last_app_conn_stat) {
    update_display = true;
  }

  last_app_conn_stat = app_connected;

  //Serial.print(F("k")); // Send token

}



/* SIGNAL CAMERA TO TAKE IMAGE(S)
*
* Optionally sends multiple delay seperated signals for exposure bracketing each focus slice
*
*/
void captureImages() {

  screenUpdate();

  for (byte i = 1; i <= settingsValue[5]; i++) { // Number of brackets to take per focus slice

    screenPrintPositionInStack();

    if (settingsValue[5] > 1) { // If more than one image is being taken, display the current position in the bracket
      screenPrint(sprintf_P(char_buffer, PSTR("Bracket %d/%d"), i, settingsValue[5]), char_buffer, 4);
    }
    
    for (byte i = 0; i < settingsValue[3]; i++) { // Count down the pause for camera on the screen

      screenPrintPositionInStack();
      screenPrint(sprintf_P(char_buffer, PSTR("Resume in %ds"), settingsValue[3] - i), char_buffer, 4);
      pause(1000);
      
      if (stackCancelled()) break; // Exit early if the stack has been cancelled

    }

    shutter(); // Take the image
    pause(1000); // Wait for capture

    if (stackCancelled()) break; // Exit early if the stack has been cancelled

  }
  
  update_display = true;
    
}



/* TRIGGER THE CAMERA SHUTTER
*
* Optionally sends two delay seperated signals to support mirror lockup
*
*/
void shutter() {

  for (byte i = 0; i <= settingsValue[4]; i++) {

    screenPrintPositionInStack();
    
    if (settingsValue[4] && i == 0) { // If mirror lockup enabled
      screenPrint(sprintf_P(char_buffer, PSTR("Mirror up")), char_buffer, 4);
    } else {
      screenPrint(sprintf_P(char_buffer, PSTR("Shutter")), char_buffer, 4);
    }    
    
    pause(300);

    digitalWrite(cam_focus, HIGH); // Trigger camera autofocus - camera may not take picture in some modes if this is not triggered first
    digitalWrite(cam_shutter, HIGH); // Trigger camera shutter

    pause(200); //Small delay needed for camera to process above signals

    digitalWrite(cam_shutter, LOW); // Switch off camera trigger signal
    digitalWrite(cam_focus, LOW); // Switch off camera focus signal

    if (settingsValue[4] && i == 0) { // If mirror lockup enabled
      pause(2000); // Pause between mirror up and shutter actuation
    }
  }
}



/*////////////////////////////////////////////////////////////////////////////////////////////////////////
//  BUTTON FUNCTIONS                                                                                    //       
////////////////////////////////////////////////////////////////////////////////////////////////////////*/ 

void button_main_change(){ /* RETURN CURRENT STATE OF MAIN PUSH BUTTON */

  main_button_reading = digitalRead(main_button);

  if (main_button_reading == LOW && main_button_previous == HIGH && millis() - main_button_time > main_button_debounce) {
    if (main_button_state == HIGH)
    main_button_state = LOW;
    else
    main_button_state = HIGH;

    main_button_time = millis();    
  }

  main_button_previous = main_button_reading;
} 

void button_rotary_change(){/* RETURN CURRENT STATE OF ROTARY ENCODER'S PUSH BUTTON */

  rbreading = digitalRead(encoder_button);
  if (rbreading == LOW && rbprevious == HIGH && millis() - rbtime > rbdebounce) {
    if (rotary_button_state == HIGH && menu_item != 0)
      rotary_button_state = LOW;
    else
      rotary_button_state = HIGH;
      rbtime = millis();    
  }

  update_display = true;
  rbprevious = rbreading;

}




void startStack() {

  main_button_state = main_button_state == HIGH ? LOW : HIGH;

}

// This is designed to flag a running stack as cancelled
// It's set with an interrupt with an emphasis on speed of execution
// So there's no debouncing or toggling of state
void cancelStack(){
  
  if(main_button_state){
    main_button_state = LOW; 
  }
  
}

void menuNav() {

  if (menu_item != 0) {
    rotary_button_state = rotary_button_state == true ? false : true;
  }

  update_display = true;

}



/* RETURN ANY CHANGE IN ROTARY ENCODER POSITION
*
* Using work by:
* http://www.circuitsathome.com/mcu/rotary-encoder-interrupt-service-routine-for-avr-micros
*
*/
int8_t encoderRead() {

  static int8_t enc_states[] = {
    //0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0 // Use this line instead to increment encoder in the opposite direction
    0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0
  };

  static uint8_t old_AB = 0;

  old_AB <<= 2; // Remember previous state
  old_AB |= ( ENC_PORT & 0x03 ); // Add current state

  int8_t encoder_data = ( enc_states[( 0x0f & old_AB )]);

  static int8_t encoder_pulse_counter = 0; // Counts pulses from encoder

  if (encoder_data) { // If Gray Code returns a valid input (1 or -1)
    encoder_data == 1 ? encoder_pulse_counter++ : encoder_pulse_counter--;
  }

  if (encoder_pulse_counter > 3) { // Record the encoder was moved by one click (detent)
    increments++;
    encoder_pulse_counter = 0;
  }

  else if (encoder_pulse_counter < -3) { // Record the encoder was moved by one click (detent) in the opposite direction
    increments--;
    encoder_pulse_counter = 0;
  }

}



/* SET UP ATMEGA PIN CHANGE INTERRUPTS */
void pciSetup(byte pin)
{
  *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));  // enable pin
  PCIFR  |= bit (digitalPinToPCICRbit(pin)); // clear any outstanding interrupt
  PCICR  |= bit (digitalPinToPCICRbit(pin)); // enable interrupt for the group
}



/* READ THE ENCODER ON ANALOGUE PORT INTERRUPT */
ISR (PCINT1_vect) {

  encoderRead();

}



/* PAUSE
*
* Delays are needed throughout the code, but these should be non-blocking
*
*/
void pause(unsigned int length) {

  unsigned long initial_millis = millis(); // Get the current millis
  unsigned long end_millis = initial_millis + length; // Define when the pause ends

  while (end_millis > millis()) { // If the end of the pause time hasn't yet been reached

    if (stackCancelled()) break; // Check continuously for any reason to end it early

  }

}



/* PRINT MENU NAVIGATION ARROWS ON DIGOLE OLED SCREEN
*
* Assumes the default font is in use (allowing 4 rows of 16 characters)
*
*/
void screenPrintMenuArrows() {

  if (!main_button_state) {
    if (rotary_button_state) { // Move to the next menu item
      //TODO: CREATE ICON FOR FONT FILE TO REPLACE BELOW
      screen.drawBox(5, 52, 1, 9); // Left outward arrow
      screen.drawBox(4, 53, 1, 7);
      screen.drawBox(3, 54, 1, 5);
      screen.drawBox(2, 55, 1, 3);
      screen.drawBox(1, 56, 1, 1);

      screen.drawBox(122, 52, 1, 9); // Right outward arrow
      screen.drawBox(123, 53, 1, 7);
      screen.drawBox(124, 54, 1, 5);
      screen.drawBox(125, 55, 1, 3);
      screen.drawBox(126, 56, 1, 1);
    } else { // Change the value of the current menu item
      //TODO: CREATE ICON FOR FONT FILE TO REPLACE BELOW
      screen.drawBox(1, 52, 1, 9); // Left inward arrow
      screen.drawBox(2, 53, 1, 7);
      screen.drawBox(3, 54, 1, 5);
      screen.drawBox(4, 55, 1, 3);
      screen.drawBox(5, 56, 1, 1);

      screen.drawBox(122, 56, 1, 1); // Right inward arrow
      screen.drawBox(123, 55, 1, 3);
      screen.drawBox(124, 54, 1, 5);
      screen.drawBox(125, 53, 1, 7);
      screen.drawBox(126, 52, 1, 9);
    }
  }
}



/* PRINT APPLICATION CONNECTION ICON ON DIGOLE OLED SCREEN
*
* Assumes the default font is in use (allowing 4 rows of 16 characters)
*
*/
void screenPrintConnection() {

  screen.print(app_connected == true ? F("AC") : F("AU"));

}



/* PRINT THE CURRENT POSITION IN A RUNNING FOCUS STACK ON DIGOLE OLED SCREEN
*
* Assumes the default font is in use (allowing 4 rows of 16 characters)
*
*/
void screenPrintPositionInStack() {

  screenUpdate();
  if(slice_count > 0){ // Default behaviour in a running stack
    screenPrint(sprintf_P(char_buffer, PSTR("Slice %d/%d"), slice_count, settingsValue[2]), char_buffer, 2);
  } else { // If called by a function when in setup
    screenPrint(sprintf_P(char_buffer, PSTR("Single slice")), char_buffer, 2);
  }

}



/* CONTROL VIA SERIAL
*
* Checks for incoming characters over serial and runs any mtaching functions
* Allows for remote control via USB or Bluetooth
*
*/
void serialCommunications() {

  if (Serial.available() > 0) {

    int serial_in = Serial.read(); // read the incoming byte:
    Serial.write("accepted\n");

    switch (serial_in) {

      case 'a': // Start or stop stack
        startStack();
        break;

      case 'b': // Toggle menu navigation
        menuNav();
        break;

      case 'c': // Increment active setting variable
        increments++;
        break;

      case 'd': // Decrement active setting variable
        increments--;
        break;

      case 'e': // Move stage backward by one slice
        stepperDriverManualControl(0, true, true);
        break;

      case 'f': // Move stage backward until another character is sent
        stepperDriverManualControl(0, true, false);
        break;

      case 'g': // Move stage forward by one slice
        stepperDriverManualControl(1, true, true);
        break;

      case 'h': // Move stage forward until another character is sent
        stepperDriverManualControl(1, true, false);
        break;

      case 'i': // Take a test image
        shutter();
        break;
        
      case 'j': // Take a suite of test images
        captureImages();
        break;

      case 'k': // Switch Bluetooth on or off
        settingsValue[10] = settingsValue[10] == 1 ? 0 : 1;
        update_display = true;
        break;

      case 'l':
        saveSettings();
        break;

      case 'm': // Switch off controller
        //sysOff();
        break;

      case 'n': // Keep connection alive
        app_connected = true;
        break;

      default: // Unmatched character - pretend the function call never happened
        app_connected = true;
    }
  }
}



/* CHECK IF THE FOCUS STACK HAS BEEN CANCELLED */
boolean stackCancelled() {
  
  if(slice_count){
    if (Serial.available() > 0) {
      int serial_in = Serial.read(); // Read the incoming byte:
      if(serial_in == 'a') startStack(); // Stop stack
    }
    
    if (!main_button_state && time_stack_started && (millis() > time_stack_started + 2000)) {
      return true; // Stack is cancelled if has been running for over 2 seconds (debouncing interrupt without using a delay)
    }
  
    main_button_state = HIGH;
  }
  
  return false;

}



/* CLEAN UP AFTER FOCUS STACK COMPLETION
*
* Optionally return the stage to its starting position
* Reset various variables to inital values
*
*/
void stackEnd() {

  screenUpdate();
  screenPrint(sprintf_P(char_buffer, main_button_state == LOW ? PSTR("Stack cancelled") : PSTR("Stack completed")), char_buffer, 2);
  
  // Calculate how many minutes and seconds the stack ran for before it completed or was cancelled
  unsigned int seconds = (millis() - time_stack_started) / 1000; // Number of seconds since stack was started
  
  div_t mins_secs = div (seconds, 60); // Number of minutes and seconds since stack was started
    
  screenPrint(sprintf_P(char_buffer, PSTR("%02dm:%02ds"), mins_secs.quot, mins_secs.rem), char_buffer, 4);

  delay(3000);

  if (settingsValue[6] == 1) { // If return stage to start position option is enabled

    screenPrint(sprintf_P(char_buffer, PSTR("Returning")), char_buffer, 4);
    pause(1000);

    for (int i = 0; i < slice_count - 1; i++) {

      stepperMoveOneSlice(0);

    }

    if (!stepperDriverInBounds()) stepperDriverClearLimitSwitch();
    stepperDriverEnable(false); // Disable the stepper driver

  }

  menu_item = 0; // Set menu to first option screen
  home_screen = true; // Reinstate the home screen
  slice_count = 0; // Reset pic counter
  main_button_state = LOW; // Return to menu options section
  update_display = true; // Write the first menu item to the screen

}



/* PULL STAGE AWAY FROM TRIPPED LIMIT SWITCH */
void stepperDriverClearLimitSwitch() {

  screenUpdate();
  
  screenPrint(sprintf_P(char_buffer, PSTR("Limit hit")), char_buffer, 2);
  screenPrint(sprintf_P(char_buffer, PSTR("Returning")), char_buffer, 4);

  stepperDriverEnable(true, 0, true); // Enable the stepper driver and toggle the direction

  while (digitalRead(limit_switch) == LOW) { //turn stepper motor for as long as  the limit switch remains pressed
    delayMicroseconds(100);
    stepperDriverStep();
  }

  stepperDriverEnable(true, 0, true); // Enable stepper driver and toggle the direction

  update_display = true;
  stepperDriverEnable(false); // Disable the stepper driver

  if (main_button_state) { // If a stack was in progress, cancel it
    main_button_state = LOW;
    stackCancelled();
  }

}



/* CHECK IF STAGE IS IN BOUNDS OF TRAVEL I.E. NEITHER LIMIT SWITCH IS TRIPPED */
boolean stepperDriverInBounds() {

  if (digitalRead(limit_switch) == HIGH) {
    return true;
  }

  return false;

}



/* SEND STEP SIGNAL TO A4988 STEPPER DRIVER */
void stepperDriverStep() {

  digitalWrite(do_step, LOW); // This LOW to HIGH change is what creates the
  digitalWrite(do_step, HIGH); // "Rising Edge" so the driver knows when to step
  delayMicroseconds(settingsValue[8] * 10); // Delay time between steps, too short and motor may stall or miss steps

}



/* CHANGE THE ACTIVE MENU SETTING'S VALUE
*
* May be implemented by either the rotary encoder or a command via serial
*
* setting_value => The variable to change the value of
* setting_lower => The lower bounds for constraining the value of var
* setting_upper => The upper bounds for constraining the value of var
* setting_multipler = Factor to multiply the change in value of var by
*
*/
void settingUpdate(int &setting_value, int setting_lower, int setting_upper, int setting_multiplier = 1) {

  setting_value = constrain(setting_value, setting_lower, setting_upper); // Keep variable value within specified range

  if (increments != 0) { // If the variable's value was changed at least once since the last check
    setting_value += (setting_multiplier * increments); // Add or subtract from variable
    increments = 0; // Reset for next check
    update_display = true;
  }

}



/* OUTPUT MENUS
*
* Publishes menus which refresh whenever the selected variable changes
*
*/
void menuInteractions() {

  char* repeated_strings[] = {"Enabled", "Disabled"};

  int lower_limit = home_screen == true ? 0 : 1; // The home screen is only appears once when the menu is first loaded, it's skipped when looping around the options

  if (rotary_button_state) { // Move through menu items
    settingUpdate(menu_item, lower_limit - 1, settings_count); // Display the currently selected menu item
    if (menu_item == settings_count) menu_item = lower_limit; // Create looping navigation
    if (menu_item == lower_limit - 1) menu_item = settings_count - 1; // Create looping navigation
  } else { // Otherwise change the value of the current menu item
    settingUpdate(settingsValue[menu_item], settingsDesc[menu_item].lower - 1, settingsDesc[menu_item].upper + 1, settingsDesc[menu_item].multiplier);
    if (settingsValue[menu_item] > settingsDesc[menu_item].upper) settingsValue[menu_item] = settingsDesc[menu_item].lower; // Create looping navigation
    if (settingsValue[menu_item] < settingsDesc[menu_item].lower) settingsValue[menu_item] = settingsDesc[menu_item].upper; // Create looping navigation
  }

  if (menu_item != 0) home_screen = false; // Remove homescreen from the menu loop once navigation begun

  if (update_display) { // Refresh menu content if the active variable has changed

    int menu_var = settingsValue[menu_item]; // The value of the active menu item
    byte string_length; // The length of the formatted string describing the current menu item
    screenUpdate(); // Update the screen
    screenPrintMenuArrows(); // Print menu arrows

    switch (menu_item) { // The menu options

      case 0:
        string_length = sprintf_P(char_buffer, PSTR("Setup"));
        break;

      case 1: // Change the number of increments to move each time
        string_length = sprintf(char_buffer, "%d", menu_var);
        break;

      case 2: // Change the number of slices to create in the stack
        string_length = sprintf(char_buffer, "%d", menu_var);
        break;

      case 3: // Change the number of seconds to wait for the camera to capture an image before continuing
        string_length = sprintf_P(char_buffer, PSTR("%ds"), menu_var);
        break;

      case 4: // Toggle mirror lockup for the camera ("Enabled"/ "Disabled")
        string_length = menu_var == 1 ? sprintf(char_buffer, repeated_strings[0]) : sprintf(char_buffer, repeated_strings[1]);
        break;

      case 5: // Change the number of images to take per focus slice (exposure bracketing)
        string_length = menu_var == 1 ? sprintf(char_buffer, "%s", repeated_strings[1]) : sprintf(char_buffer, "%d", menu_var);
        break;

      case 6: // Toggle whether camera/subject is returned the starting position at the end of the stack ("Enabled"/ "Disabled")
        string_length = menu_var == 1 ? sprintf(char_buffer, repeated_strings[0]) : sprintf(char_buffer, repeated_strings[1]);
        break;

      case 7: // Select the unit of measure to use for focus slices: Microns, Millimimeters or Centimeters
        string_length = sprintf(char_buffer, "%c%c", uom_chars[menu_var], uom_chars[1]);
        break;

      case 8: // Adjust the stepper motor speed (delay in microseconds between slice_size)
        // A smaller number gives faster motor speed but reduces torque
        // Setting this too low may cause the motor to miss steps or stall
        string_length = sprintf_P(char_buffer, PSTR("%d0uS"), menu_var);
        break;

    }

    stringConstants flashString; //Retrieve the setting title from progmem
    memcpy_P(&flashString, &settings_titles[menu_item], sizeof flashString);
    
    screenPrint(strlen(flashString.title), flashString.title, 2); // Print the menu setting title

    if (!rotary_button_state) { // Invert the colour of the current menu item to indicate it is editable
      byte boxWidth = (string_length * 8) + 2;
      byte leftPos = ((128 - boxWidth) / 2);
      screen.setMode('~');
      screen.drawBox(leftPos, 48, boxWidth, 22);
      screenPrint(string_length, char_buffer, 4); // Print the menu setting value
    } else {
      screenPrint(string_length, char_buffer, 4); // Print the menu setting value
    }
    update_display = false;

  }

}

void btnPress(byte btn_pin) {

  btn_reading = digitalRead(btn_pin);
  
  if ((btn_reading == LOW) && (btn_previous == HIGH) && (millis() - btn_time > 150)) {
    
    boolean short_press = false;
    unsigned long btn_down_time = millis();

    //if (btn_pin == main_button){
    //  while(millis() < btn_down_time + 500){ // Press and hold to start a stack
    //    if(digitalRead(btn_pin) == HIGH){ // Press and immedeately release to take test images
    //      short_press = true;
    //      break; 
    //    } 
    //  }
    //}
        
    btn_time = millis();  
  
    switch(btn_pin){
      case btn_fwd: 
      case btn_bwd:
        stepperDriverManualControl(btn_pin == btn_fwd ? 1 : 0);
        break;
      case main_button:
        short_press == true ? captureImages() : startStack();
        break;
    }
        
  }

  btn_previous = btn_reading;

}


/*////////////////////////////////////////////////////////////////////////////////////////////////////////
//  MAIN LOOP                                                                                           //
////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void loop() {


  /* MENUS AND STAGE CONTROL
  *
  * Manual control of linear stage is only enabled when in this section of the loop
  * - prevents running stacks being ruined by accidental button presses
  *
  * Menu navigation controlled either by rotary encoder with integral push button, or via serial
  *
  */
  if (main_button_state == LOW) { // User settings menus and manual stage control

    btnPress(main_button);
    btnPress(btn_fwd);
    btnPress(btn_bwd);
    serialCommunications();  // Check for commands via serial
    menuInteractions(); // Change menu options and update the screen when changed
  }



  /* FOCUS STACK
  *
  * Images are captured, the stage is advanced and the process is repeated for as many times as needed
  *
  */
  else {

    byte slice_size = settingsValue[1];
    byte slices = settingsValue[2];

    saveSettings();
    
    time_stack_started = millis();

    slice_count = 1; // Register the first image(s) of the stack is being taken
    screenPrintPositionInStack(); // Print the current position in the stack
    captureImages(); // Take the image(s) for the first slice of the stack

    for (unsigned int i = 1; i < slices; i++) { // For each subsequent focus slice in the stack

      slice_count++; // Record slices made in the stack so far

      if (stackCancelled()) break; // Exit early if the stack has been cancelled

      screenPrintPositionInStack(); // Print the current position in the stack
      // Print that the stage is being advanced by x units
      screenPrint(sprintf_P(char_buffer, PSTR("Advance %d%c%c"), slice_size, uom_chars[settingsValue[7]], uom_chars[1]), char_buffer, 4);

      if (stackCancelled()) break; // Exit early if the stack has been cancelled

      stepperDriverEnable(true, 1); // Enable the stepper driver and set the direction to forwards
      stepperMoveOneSlice(1); // Move forward by one focus slice

      if (!stepperDriverInBounds()) stepperDriverClearLimitSwitch(); // Clear hit limit switch
      if (stackCancelled()) break; // Exit early if the stack has been cancelled

      stepperDriverEnable(false); // Disable the stepper driver
      captureImages(); // Take image(s)

    }

    stackEnd(); // End of focus stack tasks

  }
}
