#include <Adafruit_VL53L0X.h>
#include <esp_now.h>
#include <WiFi.h>


#define CHANNEL 1
esp_now_peer_info_t slave;

uint8_t broadcastAddress[] = {0x8C,0xCE,0x4E,0xA6,0x41,0xC4}; // hard code broadcast address for receiver

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

int dist;

// Structure example to send data
// Must match the receiver structure
typedef struct struct_message {
  int dist;
} struct_message;

// Create a struct_message called myData
struct_message myData;

// callback when data is sent from Master to Slave
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  Serial.print("Distance measurement sent: ");
  Serial.println(dist);
}

void setup() {
  Serial.begin(115200); // Init Serial Monitor
  WiFi.mode(WIFI_STA); // Set device as a Wi-Fi Station
  // Init ESP-NOW
  if (esp_now_init() != ESP_OK){
    Serial.println("Error init");
    return;
  }
  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
// Register peer
  memcpy(slave.peer_addr, broadcastAddress, 6);
  slave.channel = CHANNEL; // pick a channel
  slave.encrypt = false; // no encryption
  // Add peer
  if (esp_now_add_peer(&slave) !=ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  Serial.println("Adafruit VL53L0X test");
  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while(1);
  }
  // power 
  Serial.println(F("VL53L0X API Simple Ranging example\n\n")); 
}

void loop() {
  myData.dist = dist;
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
  VL53L0X_RangingMeasurementData_t measure;
    
  Serial.print("Reading a measurement... ");
  lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

  if (measure.RangeStatus != 4) {  // phase failures have incorrect data
    dist = measure.RangeMilliMeter;
    Serial.print("Distance (mm): "); Serial.println(measure.RangeMilliMeter);
  } else {
    Serial.println(" out of range ");
  }
    
  delay(100);
}

