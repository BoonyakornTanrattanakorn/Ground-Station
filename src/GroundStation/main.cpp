
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

void LoRa_thread(void* pvParameters){
  SX126x LoRa;
  
  // Begin LoRa radio and set NSS, reset, busy, txen, and rxen pin with connected arduino pins
  Serial.println("Begin LoRa radio");
  int8_t nssPin = 5, resetPin = 34, busyPin = 35, irqPin = -1, txenPin = -1, rxenPin = -1;
  if (!LoRa.begin(nssPin, resetPin, busyPin, irqPin, txenPin, rxenPin)){
    Serial.println("Something wrong, can't begin LoRa radio");
    while(1);
  }

  // Configure TCXO or XTAL used in RF module
  uint8_t dio3Voltage = SX126X_DIO3_OUTPUT_1_8;
  uint32_t tcxoDelay = SX126X_TCXO_DELAY_1;
  LoRa.setDio3TcxoCtrl(dio3Voltage, tcxoDelay);
  
  // Set frequency to 915 Mhz
  LoRa.setFrequency(915000000);

  // Set RX gain to boosted gain
  LoRa.setRxGain(SX126X_RX_GAIN_BOOSTED);

  // Configure modulation parameter including spreading factor (SF), bandwidth (BW), and coding rate (CR)
  uint8_t sf = 6;
  uint32_t bw = 500000;
  uint8_t cr = 5;
  LoRa.setLoRaModulation(sf, bw, cr);

  // Configure packet parameter including header type, preamble length, payload length, and CRC type
  uint8_t headerType = SX126X_HEADER_EXPLICIT;
  uint16_t preambleLength = 12;
  uint8_t payloadLength = 15;
  bool crcType = true;
  LoRa.setLoRaPacket(headerType, preambleLength, payloadLength, crcType);

  // Set syncronize word for public network (0x3444)
  LoRa.setSyncWord(0x1424);

  Serial.println("\n-- LORA RECEIVER --\n");

  for(;;){
    // Request for receiving new LoRa packet
    LoRa.request();
    // Wait for incoming LoRa packet
    LoRa.wait();
    // Put received packet to message and counter variable
    // read() and available() method must be called after request() or listen() method
    const uint8_t msgLen = LoRa.available() - 1;
    char message[msgLen];
    uint8_t counter;
    // available() method return remaining received payload length and will decrement each read() or get() method called
    uint8_t i=0;
    while (LoRa.available() > 1){
      message[i++] = LoRa.read();
    }
    counter = LoRa.read();

    // Print received message and counter in serial
    Serial.print(message);
    Serial.print("  ");
    Serial.println(counter);
    


    // Print packet/signal status including package RSSI and SNR
    Serial.print("Packet status: RSSI = ");
    Serial.print(LoRa.packetRssi());
    Serial.print(" dBm | SNR = ");
    Serial.print(LoRa.snr());
    Serial.println(" dB");
    
    lcd.clear();
    lcd.print(message);
    lcd.print("  ");
    lcd.print(counter);
    lcd.setCursor(0, 1);
    lcd.print("RSSI = ");
    lcd.print(LoRa.packetRssi());
    lcd.print(" dBm");
    lcd.setCursor(0, 2);
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