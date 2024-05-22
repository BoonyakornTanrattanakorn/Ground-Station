#include "WifiComm.h"

#define DEBUG 1

#if DEBUG
    #define debug(x) Serial.print(x)
    #define debugln(x) Serial.println(x)
#else
    #define debug(x)
    #define debugln(x)
#endif

WifiComm::WifiComm(){
    if(DEBUG){
        WiFi.mode(WIFI_MODE_STA);
        debug("MAC Address: ");
        debugln(WiFi.macAddress());
    }
}

void WifiComm::begin(){
    // Set device as a Wi-Fi Station
    WiFi.mode(WIFI_STA);

    // Init ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }
    
    // Register peer
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0;  
    peerInfo.encrypt = false;
    
    // Add peer        
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
        debugln("Failed to add peer");
        return;
    }
}

void WifiComm::send(String message){
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t*)message.c_str(), message.length());
}

String WifiComm::get_message(){
    new_data = 0;
    return message;
}

void WifiComm::set_broadcast_address(uint8_t* broadcast_address){
    for(int i = 0; i < 6; ++i){
        broadcastAddress[i] = broadcast_address[i];
    }

}