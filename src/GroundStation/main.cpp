
/* ESPNOW */
// #include <esp_now.h>
// #include <WiFi.h>
// uint8_t broadcastAddress[] = {0xFC, 0xB4, 0x67, 0xF5, 0x4D, 0x8C};
// esp_now_peer_info_t peerInfo;

// // callback when data is sent
// void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
//   Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
// }

// void send(String message){
//   // Send message via ESP-NOW
//   if(message.length() >= 250){
//     Serial.println("Message is too long!");
//     return;
//   }
//   esp_err_t result = esp_now_send(broadcastAddress, (uint8_t*)message.c_str(), message.length());
// }

// void ESPNOW_begin(){
//   WiFi.mode(WIFI_MODE_STA);
//   Serial.println(WiFi.macAddress());

//   // Set device as a Wi-Fi Station
//   WiFi.mode(WIFI_STA);

//   // Init ESP-NOW
//   if (esp_now_init() != ESP_OK) {
//     Serial.println("Error initializing ESP-NOW");
//     return;
//   }

//   // Once ESPNow is successfully Init, we will register for Send CB to
//   // get the status of Trasnmitted packet
//   esp_now_register_send_cb(OnDataSent);
  
//   // Register peer
//   memcpy(peerInfo.peer_addr, broadcastAddress, 6);
//   peerInfo.channel = 0;  
//   peerInfo.encrypt = false;
  
//   // Add peer        
//   if (esp_now_add_peer(&peerInfo) != ESP_OK){
//     Serial.println("Failed to add peer");
//     return;
//   }
// }


/* LCD */
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 20, 4);

/* Lora */
#include <SX126x.h>


#include <Arduino.h>

void LCD_begin(){
  lcd.init();
  lcd.backlight();
  lcd.print("LCD begin.");
}

void LoRa_thread(void* pvParameters);

void setup(){
  Serial.begin(115200);
  
  //ESPNOW_begin();
  LCD_begin();

  xTaskCreatePinnedToCore(
  LoRa_thread,          /* Task function. */
  "LoRa thread",        /* name of task. */
  10000,                /* Stack size of task */
  NULL,                 /* parameter of the task */
  tskIDLE_PRIORITY,                    /* priority of the task */
  NULL,                 /* Task handle to keep track of created task */
  1);                   /* pin task to core 1 */
  delay(500);
}

void loop(){
  delay(10);
    
  // Serial.println("Input AZ EL:");
  // while (!Serial.available());
  // String AZ_EL = Serial.readStringUntil('\n');
  // Serial.println("Sending instruction: " + AZ_EL);
  // send(AZ_EL);
}

struct SensorData
{
  float temperature;
  float pressure;
  float altitude;
  int32_t latitude;
  int32_t longitude;
  int32_t altitudeGPS;
  float accelerationX;
  float accelerationY;
  float accelerationZ;
  float gyroscopeX;
  float gyroscopeY;
  float gyroscopeZ;
};

SensorData telemetry;

#include <LoRaSettings.h>

void LoRa_thread(void* pvParameters){
  SX126x LoRa;
  // Begin LoRa radio and set NSS, reset, busy, txen, and rxen pin with connected arduino pins
  Serial.println("Begin LoRa radio");
  int8_t nssPin = 5, resetPin = 34, busyPin = 35, irqPin = -1, txenPin = -1, rxenPin = -1;
  if (!LoRa.begin(nssPin, resetPin, busyPin, irqPin, txenPin, rxenPin)){
    Serial.println("Something wrong, can't begin LoRa radio");
    while(1);
  }

  LoRa.setDio3TcxoCtrl(LORA_DIO3_VOLTAGE, LORA_DIO3_TCXO);
  LoRa.setFrequency(LORA_FREQUENCY);
  LoRa.setRxGain(LORA_RX_GAIN);
  LoRa.setLoRaModulation(LORA_SPREADING_FACTOR, LORA_BANDWIDTH, LORA_CODING_RATE, 1);
  LoRa.setLoRaPacket(SX126X_HEADER_EXPLICIT, LORA_PREAMBLE_LENGTH, LORA_PAYLOAD_LENGTH, LORA_CRC);
  LoRa.setSyncWord(0x1424);
  LoRa.setDio2RfSwitch(true);

  Serial.println("\n-- LORA RECEIVER --\n");
  for(;;){
    // Request for receiving new LoRa packet
    LoRa.request();
    // Wait for incoming LoRa packet
    LoRa.wait();
    // Put received packet to message and counter variable
    // read() and available() method must be called after request() or listen() method
    const uint8_t msgLen = LoRa.available() - 1;
    char encoded_msg[msgLen];
    uint8_t i=0;
    while (LoRa.available() > 1){
      encoded_msg[i++] = LoRa.read();
    }

    Serial.println(encoded_msg);
    memcpy(&telemetry, encoded_msg, sizeof(encoded_msg));
    Serial.println(telemetry.altitudeGPS);
    Serial.printf("Temp: %.2f, Pressure: %.2f, Lat: %d, Long %d, AltGPS: %d Alt: %.2f\n", telemetry.temperature, telemetry.pressure, telemetry.latitude, telemetry.longitude, telemetry.altitudeGPS,telemetry.altitude);

    // Print packet/signal status including package RSSI and SNR
    Serial.print("Packet status: RSSI = ");
    Serial.print(LoRa.packetRssi());
    Serial.print(" dBm | SNR = ");
    Serial.print(LoRa.snr());
    Serial.println(" dB");
    
    lcd.clear();
    lcd.print(telemetry.latitude);
    lcd.setCursor(0, 1);
    lcd.print(telemetry.longitude);
    lcd.setCursor(0, 2);
    lcd.print("RSSI = ");
    lcd.print(LoRa.packetRssi());
    lcd.print(" dBm");
    lcd.setCursor(0, 3);
    lcd.print("SNR = ");
    lcd.print(LoRa.snr());
    lcd.print(" dB");


    // Show received status in case CRC or header error occur
    uint8_t status = LoRa.status();
    if (status == SX126X_STATUS_CRC_ERR) Serial.println("CRC error");
    else if (status == SX126X_STATUS_HEADER_ERR) Serial.println("Packet header error");
    Serial.println();
  }
}
