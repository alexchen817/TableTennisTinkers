#include <Arduino.h>

const int upButton = 13;
const int downButton = 12;
const int rightButton = 14;
const int leftButton = 27;

void setup() {
  // intiailize 4 buttons 
  Serial.begin(115200);
  pinMode(upButton, INPUT_PULLUP);
  pinMode(downButton, INPUT_PULLUP);
  pinMode(rightButton, INPUT_PULLUP);
  pinMode(leftButton, INPUT_PULLUP);
}

void loop() {
  // read the button state
  // not pressed down = 0
  // pressed = 1
  int upState = !digitalRead(upButton);
  int downState = !digitalRead(downButton);
  int rightState = !digitalRead(rightButton);
  int leftState = !digitalRead(leftButton);
  Serial.print("Up Button: ");
  Serial.println(upState);
  Serial.print("Down Button: ");
  Serial.println(downState);
  Serial.print("right Button: ");
  Serial.println(rightState);
  Serial.print("left Button: ");
  Serial.println(leftState);

  // avoid wasting resources 
  delay(100);
}
