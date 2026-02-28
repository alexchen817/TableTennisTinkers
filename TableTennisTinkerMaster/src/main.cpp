#include <Arduino.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <ESP32Servo.h>

const int upButton = 13;
const int downButton = 12;
const int rightButton = 14;
const int leftButton = 27;
const int indexerButton = 26;

// old slave mac address dont erase pls in case it revives!!!
// uint8_t slaveMacAddr[] = {0x94, 0xE6, 0x86, 0x3B, 0x6F, 0xF8};

// ESP WROVER MAC ADDRESS 
uint8_t slaveMacAddr[] = {0x94, 0xE6, 0x86, 0x3B, 0x5D, 0x9C};

typedef struct {
  uint8_t upState;
  uint8_t downState;
  uint8_t rightState;
  uint8_t leftState;
  uint8_t indexerState;
} Payload;

Payload payload;

esp_now_peer_info_t slaveInfo;

// function callback when data is sent
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void setup() {
  // intiailize 4 buttons 
  Serial.begin(115200);
  pinMode(upButton, INPUT_PULLUP);
  pinMode(downButton, INPUT_PULLUP);
  pinMode(rightButton, INPUT_PULLUP);
  pinMode(leftButton, INPUT_PULLUP);
  pinMode(indexerButton, INPUT_PULLUP);
  // init comms for master-slave communication 
  // wifi on standby mode 
  WiFi.mode(WIFI_STA);

  // init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error: Initialization ESP-NOW FAILED");
    return;
  }
  // register callback function 
  // this is so we can track packet success/failure 
  esp_now_register_send_cb(esp_now_send_cb_t(onDataSent));

  // register slave
  memcpy(slaveInfo.peer_addr, slaveMacAddr, 6);
  slaveInfo.channel = 0;
  slaveInfo.encrypt = false;

  // add peer 
  if (esp_now_add_peer(&slaveInfo) != ESP_OK) {
    Serial.println("Failed to add slave");
    return;
  }
}

void loop() {
  // Serial.print("ESP32 BOARD MAC ADDRESS: ");
  // Serial.println(WiFi.macAddress());

  // send input 
  payload.upState = !digitalRead(upButton);
  payload.downState = !digitalRead(downButton);
  payload.leftState = !digitalRead(leftButton);
  payload.rightState = !digitalRead(rightButton);
  payload.indexerState = !digitalRead(indexerButton);
  esp_err_t packet = esp_now_send(slaveMacAddr, (uint8_t * ) &payload, sizeof(payload));

  if (packet == ESP_OK) {
    Serial.println("Sent with success");
  } else {
    Serial.println("Error sending data");
  }
  delay(250);
}
