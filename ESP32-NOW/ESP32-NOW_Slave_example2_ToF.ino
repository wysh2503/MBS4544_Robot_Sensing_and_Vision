#include <esp_now.h>
#include <WiFi.h>

uint16_t newData;

typedef struct struct_message {
  int dist;
} struct_message;

// Create a struct_message called myData
struct_message myData;

// callback when data is recv from Master
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int data_len) {
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.print("Measured distance in (mm)  :");
  Serial.println(myData.dist);
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
}

