#include <TB6612FNG.h>
#include <LSM303_sensor.h>
#include <esp_now.h>
#include <WiFi.h>


#define DEBUG 1
#define CALIBRATE 0

#if DEBUG
  #define debug_start(x) Serial.begin(x)
  #define debug(x) Serial.print(x)
  #define debugln(x) Serial.println(x)
  #define debugf(x, y, z) Serial.printf(x, y, z)
#else
  #define debug_start(x)
  #define debug(x)
  #define debugln(x)
#endif

struct el_az
{
  float el = 0;
  float az = 0;
};

el_az target;

TB6612FNG motor;
LSM303_sensor compass;

/* ESPNOW */
String instruction;

void onDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  instruction.clear();
  for (int i = 0; i < len; i++) {
    instruction += (char)incomingData[i];
  }
  char charArray[instruction.length() + 1];
  instruction.toCharArray(charArray, sizeof(charArray));
  char *token = strtok(charArray, " ");
  if (token != NULL) {
    target.az = atoi(token);
  }
  token = strtok(NULL, " ");
  if (token != NULL) {
    target.el = atoi(token);
  }
  Serial.printf("Instruction EZ:%.2f AL:%.2f\n", target.az, target.el);
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
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(onDataRecv);
}

void compass_update(void* pvParameters);
void el_control(void* pvParameters);
void az_control(void* pvParameters);

void setup() {
  debug_start(115200);

  // put your setup code here, to run once:
  motor.begin();
  compass.begin();
  compass.set_calibration_data({  -508,   -441,   -634}, {  +438,   +524,    +86});
  ESPNOW_begin();

  xTaskCreatePinnedToCore(
              compass_update,       /* Task function. */
              "compass update",     /* name of task. */
              10000,                /* Stack size of task */
              NULL,                 /* parameter of the task */
              1,                    /* priority of the task */
              NULL,                 /* Task handle to keep track of created task */
              1);                   /* pin task to core 1 */
  
  xTaskCreatePinnedToCore(
              az_control,           /* Task function. */
              "az control",         /* name of task. */
              10000,                /* Stack size of task */
              NULL,                 /* parameter of the task */
              1,                    /* priority of the task */
              NULL,                 /* Task handle to keep track of created task */
              1);                   /* pin task to core 1 */
  
  xTaskCreatePinnedToCore(
              el_control,           /* Task function. */
              "el control",         /* name of task. */
              10000,                /* Stack size of task */
              NULL,                 /* parameter of the task */
              1,                    /* priority of the task */
              NULL,                 /* Task handle to keep track of created task */
              1);                   /* pin task to core 1 */
}

void loop() {
  delay(2000);
}


void compass_update(void* pvParameters){
  uint8_t i = 0;
  const TickType_t xDelay5ms = 5 / portTICK_PERIOD_MS;
  for(;;){
    compass.update();
    if(i == 20){
      debugf("EL: %.2f AZ: %.2f\n", compass.el, compass.az);
      i = 0;
    }
    ++i;
    vTaskDelay(xDelay5ms);
  }
}

void el_control(void* pvParameters){
  float el_diff;
  int8_t last_rotation = 0; // -1 = ccw, 0 = idle, 1 = cw
  int8_t el_error = 2;

  const TickType_t xDelay5ms = 5 / portTICK_PERIOD_MS;
  const TickType_t xDelay10ms = 10 / portTICK_PERIOD_MS;
  const TickType_t xDelay100ms = 100 / portTICK_PERIOD_MS;
  const TickType_t xDelay500ms = 500 / portTICK_PERIOD_MS;

  for(;;){
    el_diff = target.el - compass.el;
    if(el_diff > el_error){
      motor.el_ccw(); last_rotation = -1;
    }else if(el_diff < -el_error){
      motor.el_cw(); last_rotation = 1;
    }else{
      if(last_rotation == 1){
        motor.el_ccw(); vTaskDelay(xDelay10ms);
      }else if(last_rotation == -1){
        motor.el_cw(); vTaskDelay(xDelay10ms);
      }
      motor.el_stop(); last_rotation = 0; vTaskDelay(xDelay500ms);
    }
  }
}

void az_control(void* pvParameters){
  float az_diff;
  int8_t last_rotation = 0; // -1 = ccw, 0 = idle, 1 = cw
  int8_t az_error = 2;

  const TickType_t xDelay5ms = 5 / portTICK_PERIOD_MS;
  const TickType_t xDelay10ms = 10 / portTICK_PERIOD_MS;
  const TickType_t xDelay100ms = 100 / portTICK_PERIOD_MS;
  const TickType_t xDelay500ms = 500 / portTICK_PERIOD_MS;
  for(;;){
    az_diff = fmod(target.az - compass.az + 540, 360) - 180;
    if(az_diff > az_error){
      motor.az_cw(); last_rotation = 1;
    }else if(az_diff < -az_error){
      motor.az_ccw(); last_rotation = -1;
    }else{
      if(last_rotation == 1){
        motor.az_ccw(); vTaskDelay(xDelay10ms);
      }else if(last_rotation == -1){
        motor.az_cw();vTaskDelay(xDelay10ms);
      }
      motor.az_stop(); last_rotation = 0; vTaskDelay(xDelay500ms);
    }
  }
}
