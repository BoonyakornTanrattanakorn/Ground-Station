#define DEBUG 1

#if DEBUG
    #define debug(x) Serial.print(x)
    #define debugln(x) Serial.println(x)
#else
    #define debug(x)
    #define debugln(x)
#endif

#define PSEUDO_DATA 1

// config
struct position{
  double latitude = 32.369875;
  double longitude = -106.752410;
  double altitude = 1297; 
} ground;


/* ESPNOW */
#include <WiFi.h>
#include <esp_now.h>

bool WIFI_available = 0;
// receiver address
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
esp_now_peer_info_t peerInfo;

typedef struct struct_message {
  float el;
  float az;
} struct_message;
struct_message target;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  debug("\r\nLast Packet Send Status:\t");
  debugln(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void ESPNOW_begin(){
  #if DEBUG
    WiFi.mode(WIFI_MODE_STA);
    debugln(WiFi.macAddress());
  #endif

  WiFi.mode(WIFI_STA);
  if (esp_now_init() != 0) {
    debugln("Error initializing ESP-NOW");
    return;
  }

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
  WIFI_available = 1;
  debugln("WIFI available.");
}

/* LCD */
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 20, 4);

void LCD_begin(){
  lcd.init();
  lcd.backlight();
  lcd.autoscroll();
  lcd.print("LCD begin.");
}

#include <Arduino.h>

void LoRa_thread(void* pvParameters);

void setup(){
  Serial.begin(115200);
  Wire.setClock(400000L);
  
  ESPNOW_begin();
  LCD_begin();
  #if(PSEUDO_DATA)
    return;
  #endif
  delay(500);

  xTaskCreatePinnedToCore(
  LoRa_thread,          /* Task function. */
  "LoRa thread",        /* name of task. */
  10000,                /* Stack size of task */
  NULL,                 /* parameter of the task */
  1,                    /* priority of the task */
  NULL,                 /* Task handle to keep track of created task */
  1);                   /* pin task to core 1 */
}

void loop(){
  #if(PSEUDO_DATA)
    Serial.printf("DATA,%.8f,%.8f,%.4f,%.4f,%d\n", random(-180*100000.0, 180*100000.0)/100000.0, random(-90*100000.0, 90*100000.0)/10000000.0, random(1000*1000.0, 20000*1000.0)/1000.0, random(1000*1000.0, 20000*1000.0)/1000.0, random(1, 10));
    target.el = fmod(target.el + random(-1000, 1000)/100.0, 360.0);
    target.az = fmod(random(-1000, 1000)/100.0, 90.0);
    lcd.clear();
    if(WIFI_available){
      esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &target, sizeof(target));
      lcd.printf("EL%.2f AZ%.2f ", target.el, target.az);
      if(result == ESP_OK){
        lcd.print("OK");
      }else{
        lcd.print("SEND ERROR");
      }
    }else{
      lcd.print("WiFi ded");
    }
  #endif
  delay(1000);
}

struct RadioPacket
{
  int32_t latitude;
  int32_t longitude;
  int32_t altitudeGPS;
  float altitude;
  int flightStage;
};

// struct RadioPacket
// {
//   float temperature;
//   float pressure;
//   float altitude;
//   int32_t latitude;
//   int32_t longitude;
//   int32_t altitudeGPS;
//   float ax;
//   float ay;
//   float az;
//   float gx;
//   float gy;
//   float gz;
// };

RadioPacket packet;

double calculateDistance(double lat1, double lon1, double alt1,
                         double lat2, double lon2, double alt2) {
    const double R = 6371.0;
    
    // Convert degrees to radians
    double phi1 = lat1 * DEG_TO_RAD;
    double lambda1 = lon1 * DEG_TO_RAD;
    double phi2 = lat2 * DEG_TO_RAD;
    double lambda2 = lon2 * DEG_TO_RAD;

    // Calculate differences
    double dx = (R + alt1) * cos(phi1) * cos(lambda1) - (R + alt2) * cos(phi2) * cos(lambda2);
    double dy = (R + alt1) * cos(phi1) * sin(lambda1) - (R + alt2) * cos(phi2) * sin(lambda2);
    double dz = (R + alt1) * sin(phi1) - (R + alt2) * sin(phi2);

    // Calculate straight-line 3D distance
    double distance = sqrt(dx*dx + dy*dy + dz*dz);

    return distance;
}

#include <LoRaSettings.h>

void LoRa_thread(void* pvParameters){
  SX126x LoRa;
  // Begin LoRa radio and set NSS, reset, busy, txen, and rxen pin with connected arduino pins
  debugln("Begin LoRa radio");
  int8_t nssPin = 5, resetPin = 34, busyPin = 35, irqPin = -1, txenPin = -1, rxenPin = -1;
  if (!LoRa.begin(nssPin, resetPin, busyPin, irqPin, txenPin, rxenPin)){
    debugln("Something wrong, can't begin LoRa radio");
    while(1);
  }

  LoRa.setDio3TcxoCtrl(LORA_DIO3_VOLTAGE, LORA_DIO3_TCXO);
  LoRa.setFrequency(LORA_FREQUENCY);
  LoRa.setRxGain(LORA_RX_GAIN);
  LoRa.setLoRaModulation(LORA_SPREADING_FACTOR, LORA_BANDWIDTH, LORA_CODING_RATE, 1);
  LoRa.setLoRaPacket(SX126X_HEADER_EXPLICIT, LORA_PREAMBLE_LENGTH, LORA_PAYLOAD_LENGTH, LORA_CRC);
  LoRa.setSyncWord(0x1424);
  LoRa.setDio2RfSwitch(true);

  debugln("\n-- LORA RECEIVER --\n");

  int total_packet_receieved = 0;
  for(;;){
    LoRa.request();
    LoRa.wait();
    // Put received packet to message and counter variable
    // read() and available() method must be called after request() or listen() method
    const uint8_t msgLen = LoRa.available() - 1;
    char encoded_msg[msgLen];
    uint8_t i=0;
    while (LoRa.available() > 1){
      encoded_msg[i++] = LoRa.read();
    }
    memcpy(&packet, encoded_msg, sizeof(encoded_msg));

    // Show received status in case CRC or header error occur
    uint8_t status = LoRa.status();
    if(status == SX126X_STATUS_CRC_ERR){
      debugln("CRC error"); lcd.print("CRC error");
    }
    else if(status == SX126X_STATUS_HEADER_ERR){
      debugln("Packet header error"); lcd.print("Header error");
    }else{
      debugln("Latitude, Longitude, AltitudeGPS, Altitude, FlightState");
      Serial.printf("DATA,%.8f,%.8f,%.4f,%.4f,%d\n", packet.latitude/100000000.0, packet.longitude/10000000.0, packet.altitudeGPS/1000.0, packet.altitude, packet.flightStage);
      #if DEBUG
        Serial.printf("Packet status: RSSI = %d dBm | SNR = %f dB\n", LoRa.packetRssi(), LoRa.snr());
      #endif

      lcd.clear();
      lcd.printf("Lat=%.8f", packet.latitude/100000000.0);
      lcd.setCursor(0, 1);
      lcd.printf("Long=%.8f", packet.longitude/100000000.0);
      lcd.setCursor(0, 2);
      lcd.printf("Dist=%.2fkm|%d", calculateDistance(ground.latitude, ground.longitude, ground.altitude, packet.latitude/100000000.0, packet.longitude/10000000.0, packet.altitudeGPS/1000.0), ++total_packet_receieved);
      lcd.setCursor(0, 3);
      lcd.printf("RSSI=%d,SNR=%.2f", LoRa.packetRssi(), LoRa.snr());

      if(WIFI_available){
        esp_now_send(broadcastAddress, (uint8_t *) &target, sizeof(target));
      }else{
        debugln("WIFI not available.");
      }
    }
    debugln();
  }
}

void print_pseudo_data(){
  Serial.printf("DATA,%.8f,%.8f,%.4f,%.4f,%d\n", random(-180*100000.0, 180*100000.0)/100000.0, random(-90*100000.0, 90*100000.0)/10000000.0, random(1000*1000.0, 20000*1000.0)/1000.0, random(1000*1000.0, 20000*1000.0)/1000.0, random(1, 10));
}