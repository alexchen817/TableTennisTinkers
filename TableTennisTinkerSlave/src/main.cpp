#include <WiFi.h>
#include <Arduino.h>
#include <esp_now.h>
#include <ESP32Servo.h>

const int MAX_DEGREES = 175;
const int WAIT_TIME = 100;

typedef struct payload {
  uint8_t upState;
  uint8_t downState;
  uint8_t rightState;
  uint8_t leftState;
} Payload;

Payload payload;

// -----------
// SERVO STRUCT 
// -----------
struct ServoState {
  Servo servo;
  int pin;
  int currentAngle; // current angle pos. 
  int waitTime; // to avoid moving infinitely inside loop() in miliseconds
  unsigned long lastMoveTime; // timestamp to hold the most recent time the servo moved
};

ServoState PitchServo;
ServoState YawServo;

// CALLBACK FUNCTION
void onDataRecv(const esp_now_recv_info *recv_info, const uint8_t *incomingData, int len) {
  memcpy(&payload, incomingData, sizeof(payload));
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("up state: ");
  Serial.println(payload.upState);
  Serial.print("down state: ");
  Serial.println(payload.downState);
  Serial.print("left state: ");
  Serial.println(payload.leftState);
  Serial.print("right state: ");
  Serial.println(payload.rightState);
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  // set device as wifi station 
  WiFi.mode(WIFI_STA);

  // init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_recv_cb(onDataRecv);

  // servo setup
  PitchServo.pin = 18;
  PitchServo.servo.attach(PitchServo.pin, 500, 2500);
  PitchServo.waitTime = WAIT_TIME;
  PitchServo.lastMoveTime = 0;
  PitchServo.currentAngle = 0;

  YawServo.pin = 19;
  YawServo.servo.attach(YawServo.pin, 500, 2500);
  YawServo.waitTime = WAIT_TIME;
  YawServo.lastMoveTime = 0;
  YawServo.currentAngle = 0;
}

void loop() {
  // first check if two buttons are press at the same time
  if (payload.upState && payload.downState) {
    // since loop() is a function within an implicit while(true)
    return;
  } else if(payload.rightState && payload.leftState) {
    // since loop() is a function within an implicit while(true)
    return;
  }

  // check time elapsed 
  unsigned long currentTime = millis();
  // if the time elapsed has been long enough, move the motor 
  // without delay() this will get checked 50000 times 
  if (currentTime - PitchServo.lastMoveTime >= PitchServo.waitTime) {
    if (payload.upState) {
      if (PitchServo.currentAngle >= MAX_DEGREES) {
        // hit 180 degree limit, stop
        return;
      } else {
        PitchServo.currentAngle += 3; 
      }
      PitchServo.servo.write(PitchServo.currentAngle);
    } else if (payload.downState) {
      if (PitchServo.currentAngle <= 0) {
        return;
      } else {
        PitchServo.currentAngle -= 3;
      }
      PitchServo.servo.write(PitchServo.currentAngle);
    }
    // update the time the motor moved at 
    PitchServo.lastMoveTime = currentTime;
  }
  // if the time elapsed has been long enough, move the motor 
  // without delay() this will get checked 50000 times 
  if (currentTime - YawServo.lastMoveTime >= YawServo.waitTime) {
    if (payload.rightState) {
      if (YawServo.currentAngle >= MAX_DEGREES) {
        // hit 180 degree limit, stop
        return;
      } else {
        YawServo.currentAngle += 3; 
      }
      YawServo.servo.write(YawServo.currentAngle);
    } else if (payload.leftState) {
      if (YawServo.currentAngle <= 0) {
        return;
      } else {
        YawServo.currentAngle -= 3;
      }
      YawServo.servo.write(YawServo.currentAngle);
    }
    // update the time the motor moved at 
    YawServo.lastMoveTime = currentTime;
  }
  // calling delay() will block all background processes to do with
  // ESP-NOW wifi coms, DO NOT WRITE DELAY HERE
}
