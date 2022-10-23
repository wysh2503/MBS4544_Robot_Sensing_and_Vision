#include <esp_now.h>
#include <WiFi.h>

#define CHANNEL 1

uint8_t data;
uint16_t newData;

typedef struct struct_message {
  uint8_t data;
} struct_message;

// Create a struct_message called myData
struct_message myData;

// callback when data is recv from Master
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int data_len) {
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.print("Received  :");
  Serial.println(myData.data);
  newData = myData.data * 10;
}

void setup() {
  Serial.begin(115200);// Initialize Serial Monitor
  WiFi.mode(WIFI_STA); // Set device as a Wi-Fi Station
  if (esp_now_init() != ESP_OK){
    Serial.println("err init....");
    return;
  }
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
 Serial.print("Data manipulated: ");
 Serial.println(newData);
 delay(200);
}

