#include <WiFi.h>
#include <Arduino.h>
#include <esp_now.h>

typedef struct payload {
  uint8_t upState;
  uint8_t downState;
  uint8_t rightState;
  uint8_t leftState;
} Payload;

Payload payload;

void onDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len) {
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
}

void loop() {

}
