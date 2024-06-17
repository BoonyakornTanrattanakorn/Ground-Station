#include <LiquidCrystal_I2C.h>
#include <Wire.h>

#define DEBUG 1
#if DEBUG
    #define debug(x) Serial.print(x)
    #define debugln(x) Serial.println(x)
#else
    #define debug(x)
    #define debugln(x)
#endif

/* LCD */
LiquidCrystal_I2C lcd(0x27, 16, 2);  // set the LCD address to 0x27 for a 16 chars and 2 line display

struct customChar{
  uint8_t UpArrowArray[8] = {
    0b00100,
    0b01110,
    0b11111,
    0b00100,
    0b00100,
    0b00100,
    0b00100,
    0b00100
  };

  uint8_t DownArrowArray[8] = {
    0b00100,
    0b00100,
    0b00100,
    0b00100,
    0b00100,
    0b11111,
    0b01110,
    0b00100
  };

  uint8_t LeftArrowArray[8] = {
    0b00011,
    0b00110,
    0b01100,
    0b11000,
    0b11000,
    0b1100,
    0b00110,
    0b00011
  };

  uint8_t RightArrowArray[8] = {
    0b11000,
    0b01100,
    0b00110,
    0b00011,
    0b00011,
    0b00110,
    0b01100,
    0b11000
  };
} customChar;

uint8_t UpArrow = 0;
uint8_t DownArrow = 1;
uint8_t LeftArrow = 2;
uint8_t RightArrow = 3;

/* LSM303 */
#include <LSM303_sensor.h>
LSM303_sensor LSM303;

/* ESPNOW */
#include <ESP8266WiFi.h>
#include <espnow.h>

typedef struct struct_message {
  float el;
  float az;
} struct_message;
struct_message target;

bool new_data_received = 0;
bool WIFI_available = 0;

void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len) {
  memcpy(&target, incomingData, sizeof(target));
  #if(DEBUG)
    Serial.printf("EL %.2f AZ %.2f", target.el, target.az);
  #endif
  new_data_received = 1;
}

void ESPNOW_begin(){
  debugln(WiFi.macAddress());
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != 0) {
    debugln("Error initializing ESP-NOW");
    return;
  }
  WIFI_available = 1;
  esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);
  esp_now_register_recv_cb(OnDataRecv);
}

/* Function Declaration */
void display_up();
void display_right();
void display_down();
void display_left();
void display_centered();

/* Misc */
const int el_error = 5;
const int az_error = 5;
float el_bias = -5.48;

void setup() {
  Serial.begin(115200);
  debugln("Serial start.");
  Wire.setClock(400000L);

  lcd.init();  // initialize the lcd
  lcd.createChar(UpArrow, customChar.UpArrowArray);
  lcd.createChar(DownArrow, customChar.DownArrowArray);
  lcd.createChar(LeftArrow, customChar.LeftArrowArray);
  lcd.createChar(RightArrow, customChar.RightArrowArray);
  lcd.backlight();

  LSM303.begin();
  LSM303.set_calibration_data({  -523,   -845,   -348}, {  +337,   +382,   +561});

  ESPNOW_begin();
}


void loop() {
  LSM303.update();
  //Serial.println(LSM303.el+el_bias);
  lcd.clear();
  bool centered = 1;
  if(target.el-(LSM303.el+el_bias) > el_error){
    display_down(); centered = 0;
  }else if(target.el-(LSM303.el+el_bias) < -el_error){
    display_up(); centered = 0;
  }
  float az_diff = fmod(target.az - LSM303.az + 540, 360) - 180;
  Serial.println(az_diff);
  if(az_diff > az_error){
    display_right(); centered = 0;
  }else if(az_diff < -az_error){
    display_left(); centered = 0;
  }
  if(centered){
    display_centered();
  }
  lcd.setCursor(0, 1);
  lcd.printf("EL%.2f AZ%.2f", LSM303.el+el_bias, LSM303.az);
  unsigned long t = millis();
  while(millis() - t <= 50){
    LSM303.update();
    delay(5);
  }
}

void display_up() { 
  lcd.setCursor(7, 0);
  lcd.write(UpArrow);
}

void display_down() {
  lcd.setCursor(8, 0);
  lcd.write(DownArrow);
}

void display_left(){
  lcd.setCursor(6, 0);
  lcd.write(LeftArrow);
}

void display_right(){
  lcd.setCursor(9, 0);
  lcd.write(RightArrow);
}

void display_centered(){
  lcd.setCursor(6, 0);
  lcd.print("Nice");
}

