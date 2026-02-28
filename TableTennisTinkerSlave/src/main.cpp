#include <WiFi.h>
#include <Arduino.h>
#include <esp_now.h>
#include <ESP32Servo.h>

const int MAX_DEGREES = 175;
const int WAIT_TIME_MSEC = 100;
const int INDEXER_WAIT_TIME_MSEC = 5000;
const int NUM_CHUTES = 4;
const int INDEXER_START_POS_DEG = 32;
typedef struct payload {
  uint8_t upState;
  uint8_t downState;
  uint8_t rightState;
  uint8_t leftState;
  uint8_t indexerState;
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
ServoState IndexerServo;

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
  Serial.print("indexer State: ");
  Serial.println(payload.indexerState);
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
  // wait for connection
  delay(1000);
  // servo setup
  PitchServo.pin = 18;
  // pulse width range (500-2500)
  PitchServo.servo.attach(PitchServo.pin, 500, 2500);
  PitchServo.waitTime = WAIT_TIME_MSEC;
  PitchServo.lastMoveTime = 0;
  PitchServo.currentAngle = 10;
  PitchServo.servo.write(PitchServo.currentAngle);
  YawServo.pin = 19;
  YawServo.servo.attach(YawServo.pin, 700, 2500);
  YawServo.waitTime = WAIT_TIME_MSEC;
  YawServo.lastMoveTime = 0;
  YawServo.currentAngle = 100;
  YawServo.servo.write(YawServo.currentAngle);

  IndexerServo.pin = 21;
  IndexerServo.waitTime = INDEXER_WAIT_TIME_MSEC;
  IndexerServo.lastMoveTime = 0;
  IndexerServo.currentAngle = INDEXER_START_POS_DEG;
  IndexerServo.servo.write(IndexerServo.currentAngle);
  IndexerServo.servo.attach(IndexerServo.pin, 500, 2400);
}

int indexerPos = 0;
bool lastIndexerState = false;
void loop() {
  // first check if two buttons are pressed at the same time
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

  // indexer control
  // only move servo if the last state was false 
  if (payload.indexerState && lastIndexerState == false) {
      indexerPos++;
      if (indexerPos > NUM_CHUTES) {
        // reset to starting pos 
        IndexerServo.currentAngle = INDEXER_START_POS_DEG;
        indexerPos = 0;
      } else {
        // gear ratio is 2:1 (driven gear:driver gear)
        IndexerServo.currentAngle += 36; // 72 / 2
      }
      // input angle to servo 
      IndexerServo.servo.write(IndexerServo.currentAngle);
      delay(50);

  }
  // update the indexer state from the packet 
  lastIndexerState = payload.indexerState;
  // calling delay() will block all background processes to do with
  // ESP-NOW wifi coms, DO NOT WRITE DELAY HERE
}
