/*//sketch for test purpose only.
 * It will toogle the led each time you press the correponding button
 * 
 * Created by ArduinoGetStarted.com
 *
 * This example code is in the public domain
 *
 * Tutorial page: https://arduinogetstarted.com/tutorials/arduino-button-toggle-led
 */

// constants won't change
const int HBABUTTON_PIN = A3; // Arduino pin connected to button's pin
const int HBALED_PIN    = 6; // Arduino pin connected to LED's pin
const int SAMBUTTON_PIN = A4; // Arduino pin connected to button's pin
const int SAMLED_PIN    = 5; // Arduino pin connected to LED's pin
const int AASBUTTON_PIN = A5; // Arduino pin connected to button's pin
const int AASLED_PIN    = 3; // Arduino pin connected to LED's pin


// variables will change:
int HBAledState = LOW;     // the current state of LED
int HBAlastButtonState;    // the previous state of button
int HBAcurrentButtonState; // the current state of button
int SAMledState = LOW;     // the current state of LED
int SAMlastButtonState;    // the previous state of button
int SAMcurrentButtonState; // the current state of button
int AASledState = LOW;     // the current state of LED
int AASlastButtonState;    // the previous state of button
int AAScurrentButtonState; // the current state of button

void setup() {
  Serial.begin(115200);                // initialize serial
  pinMode(HBABUTTON_PIN, INPUT_PULLUP); // set arduino pin to input pull-up mode
  pinMode(HBALED_PIN, OUTPUT);          // set arduino pin to output mode
  
  pinMode(SAMBUTTON_PIN, INPUT_PULLUP); // set arduino pin to input pull-up mode
  pinMode(SAMLED_PIN, OUTPUT);          // set arduino pin to output mode
  
  pinMode(AASBUTTON_PIN, INPUT_PULLUP); // set arduino pin to input pull-up mode
  pinMode(AASLED_PIN, OUTPUT);          // set arduino pin to output mode
  
  HBAcurrentButtonState = digitalRead(HBABUTTON_PIN);
  SAMcurrentButtonState = digitalRead(SAMBUTTON_PIN);
  AAScurrentButtonState = digitalRead(AASBUTTON_PIN);

}

void loop() {
  HBAlastButtonState    = HBAcurrentButtonState;      // save the last state
  HBAcurrentButtonState = digitalRead(HBABUTTON_PIN); // read new state

  if(HBAlastButtonState == HIGH && HBAcurrentButtonState == LOW) {
    Serial.println("The HBA is pressed");

    // toggle state of LED
    HBAledState = !HBAledState;

    // control LED arccoding to the toggled state
    digitalWrite(HBALED_PIN, HBAledState); 
      }

      SAMlastButtonState    = SAMcurrentButtonState;      // save the last state
  SAMcurrentButtonState = digitalRead(SAMBUTTON_PIN); // read new state

  if(SAMlastButtonState == HIGH && SAMcurrentButtonState == LOW) {
    Serial.println("The SAM is pressed");

    // toggle state of LED
    SAMledState = !SAMledState;

    // control LED arccoding to the toggled state
    digitalWrite(SAMLED_PIN, SAMledState); 
      }

      AASlastButtonState    = AAScurrentButtonState;      // save the last state
  AAScurrentButtonState = digitalRead(AASBUTTON_PIN); // read new state

  if(AASlastButtonState == HIGH && AAScurrentButtonState == LOW) {
    Serial.println("The AAS is pressed");

    // toggle state of LED
    AASledState = !AASledState;

    // control LED arccoding to the toggled state
    digitalWrite(AASLED_PIN, AASledState); 
  }
}
