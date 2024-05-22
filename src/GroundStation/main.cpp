#include <esp_now.h>
#include <WiFi.h>

/* ESPNOW */
uint8_t broadcastAddress[] = {0xFC, 0xB4, 0x67, 0xF5, 0x4D, 0x8C};
esp_now_peer_info_t peerInfo;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void send(String message){
  // Send message via ESP-NOW
  if(message.length() >= 250){
    Serial.println("Message is too long!");
    return;
  }
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t*)message.c_str(), message.length());
}

void ESPNOW_begin(){
  WiFi.mode(WIFI_MODE_STA);
  Serial.println(WiFi.macAddress());

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
}

void setup(){
  Serial.begin(115200);
  
  ESPNOW_begin();
}

void loop(){
  Serial.println("Input AZ EL:");
  while (!Serial.available());
  String AZ_EL = Serial.readStringUntil('\n');
  Serial.println("Sending instruction: " + AZ_EL);
  send(AZ_EL);
}


