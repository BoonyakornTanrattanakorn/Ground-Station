#include <TB6612FNG.h>
#include <LSM303_sensor.h>
#include <MPU6050_sensor.h>



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
MPU6050_sensor gyro;


/* ESPNOW */
#include <esp_now.h>
#include <WiFi.h>

void onDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  memcpy(&target, incomingData, sizeof(target));
  Serial.printf("Instruction AZ:%.2f EL:%.2f\n", target.az, target.el);
}

void ESPNOW_begin(){
  WiFi.mode(WIFI_MODE_STA);
  Serial.println(WiFi.macAddress());
  
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    debugln("Error initializing ESP-NOW");
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(onDataRecv);
}

/* Function Declaration */
void compass_update(void* pvParameters);
void el_control(void* pvParameters);
void az_control(void* pvParameters);

void setup() {
  debug_start(115200);

  // put your setup code here, to run once:
  motor.begin();
  gyro.begin();
  compass.begin();
  compass.set_calibration_data({  -508,   -441,   -634}, {  +438,   +524,    +86});
  //ESPNOW_begin();

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
  // motor.az_cw();
  // delay(2000);
  // motor.az_ccw();
  delay(2000);
}


void compass_update(void* pvParameters){
  uint8_t i = 0;
  const TickType_t xDelay5ms = 5 / portTICK_PERIOD_MS;
  
  for(;;){
    compass.update();
    gyro.update();
    if(i >= 10){
      debugf("EL: %.2f AZ: %.2f\n", gyro.raw_el, compass.az);
      i = 0;
    }
    ++i;
    vTaskDelay(xDelay5ms);
  }
}

void el_control(void* pvParameters){
  float el_diff;
  // int8_t last_rotation = 0; // -1 = ccw, 0 = idle, 1 = cw
  int8_t el_error = 5;

  const TickType_t xDelay10ms = 10 / portTICK_PERIOD_MS;

  for(;;){
    el_diff = target.el - gyro.raw_el;
    if(el_diff >= el_error){
      motor.set_el_PWM(1);
      motor.el_ccw();
    }else if(el_diff >= 0){
      motor.set_el_PWM( (el_diff / el_error) );
      motor.el_ccw();
    }else if(el_diff <= -el_error){
      motor.set_el_PWM(1);
      motor.el_cw();
    }else{ // el_diff <= 0
      motor.set_el_PWM( -(el_diff / el_error) );
      motor.el_cw();
    }
    vTaskDelay(xDelay10ms);

    // if(el_diff > el_error){
    //   motor.el_ccw(); last_rotation = -1;
    // }else if(el_diff < -el_error){
    //   motor.el_cw(); last_rotation = 1;
    // }else{
    //   if(last_rotation == 1){
    //     motor.el_ccw(); vTaskDelay(xDelay10ms);
    //   }else if(last_rotation == -1){
    //     motor.el_cw(); vTaskDelay(xDelay10ms);
    //   }
    //   motor.el_stop(); last_rotation = 0; vTaskDelay(xDelay500ms);
    // }
  }
}

void az_control(void* pvParameters){
  float az_diff;
  // int8_t last_rotation = 0; // -1 = ccw, 0 = idle, 1 = cw
  int8_t az_error = 5;
  float precise_az_error = 1;

  const TickType_t xDelay10ms = 10 / portTICK_PERIOD_MS;
  for(;;){
    az_diff = fmod(target.az - compass.az + 540, 360) - 180;
    if(az_diff >= az_error){
      motor.set_az_PWM(1);
      motor.az_ccw();
    }else if(az_diff >= 0){
      motor.set_az_PWM( (az_diff / az_error) );
      motor.az_ccw();
    }else if(az_diff <= -az_error){
      motor.set_az_PWM(1);
      motor.az_cw();
    }else{ // el_diff <= 0
      motor.set_az_PWM( (az_diff / az_error) );
      motor.az_cw();
    }
    vTaskDelay(xDelay10ms);

    // if(az_diff > az_error){
    //   motor.az_cw(); last_rotation = 1;
    // }else if(az_diff < -az_error){
    //   motor.az_ccw(); last_rotation = -1;
    // }else{
    //   if(last_rotation == 1){
    //     motor.az_ccw(); vTaskDelay(xDelay10ms);
    //   }else if(last_rotation == -1){
    //     motor.az_cw();vTaskDelay(xDelay10ms);
    //   }
    //   motor.az_stop(); last_rotation = 0; vTaskDelay(xDelay500ms);
    // }
  }
}
