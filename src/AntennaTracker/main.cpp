#include <TB6612FNG.h>
#include <LSM303_sensor.h>

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

TB6612FNG Motor;
LSM303_sensor compass;

void compass_update(void* pvParameters);
void print_calibration_data(void* pvParameters);
void LSM303_calibration();
void el_control(void* pvParameters);
void az_control(void* pvParameters);

void setup() {
  debug_start(115200);

  // put your setup code here, to run once:
  Motor.begin();
  compass.begin();
  compass.set_calibration_data({  -508,   -441,   -634}, {  +438,   +524,    +86});

  if(CALIBRATE){
    xTaskCreatePinnedToCore(
                compass_update,       /* Task function. */
                "compass update",     /* name of task. */
                10000,                /* Stack size of task */
                NULL,                 /* parameter of the task */
                1,                    /* priority of the task */
                NULL,                 /* Task handle to keep track of created task */
                1);                   /* pin task to core 1 */

    xTaskCreatePinnedToCore(
                print_calibration_data,       /* Task function. */
                "print calibration data",     /* name of task. */
                10000,                        /* Stack size of task */
                NULL,                         /* parameter of the task */
                0,                            /* priority of the task */
                NULL,                         /* Task handle to keep track of created task */
                1);                           /* pin task to core 1 */

    compass.set_calibration_data({-32768, -32768, -32768}, {+32767, +32767, +32767});
    LSM303_calibration();
  }
  
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
                2,                    /* priority of the task */
                NULL,                 /* Task handle to keep track of created task */
                1);                   /* pin task to core 1 */
  
  xTaskCreatePinnedToCore(
              el_control,           /* Task function. */
              "el control",         /* name of task. */
              10000,                /* Stack size of task */
              NULL,                 /* parameter of the task */
              2,                    /* priority of the task */
              NULL,                 /* Task handle to keep track of created task */
              1);                   /* pin task to core 1 */
}

void loop() {
  Motor.az_ccw();
  delay(2000);
  Motor.az_cw();
  delay(2000);
}


void compass_update(void* pvParameters){
  uint8_t i = 0;
  const TickType_t xDelay5ms = 5 / portTICK_PERIOD_MS;
  for(;;){
    compass.update();
    if(i == 10){
      debugf("EL: %.2f AZ: %.2f\n", compass.el, compass.az);
      i = 0;
    }
    ++i;
    vTaskDelay(xDelay5ms);
  }
}

void print_calibration_data(void* pvParameters){
  LSM303::vector<int16_t> running_min = {32767, 32767, 32767}, running_max = {-32768, -32768, -32768};
  char report[80];
  const TickType_t xDelay10ms = 10 / portTICK_PERIOD_MS;
  for(;;){
    compass.compass.read();

    running_min.x = min(running_min.x, compass.compass.m.x);
    running_min.y = min(running_min.y, compass.compass.m.y);
    running_min.z = min(running_min.z, compass.compass.m.z);

    running_max.x = max(running_max.x, compass.compass.m.x);
    running_max.y = max(running_max.y, compass.compass.m.y);
    running_max.z = max(running_max.z, compass.compass.m.z);

    snprintf(report, sizeof(report), "min: {%+6d, %+6d, %+6d}    max: {%+6d, %+6d, %+6d}",
    running_min.x, running_min.y, running_min.z,
    running_max.x, running_max.y, running_max.z);
    debugln(report);

    vTaskDelay(xDelay10ms);
  }
}

void LSM303_calibration(){
  debugln("Begin LSM303 calibration");

  // Set elevation to 90 degrees
  // if(compass.el > 90){Motor.el_cw();}
  // else{Motor.el_ccw();}
  // while(abs(compass.el-90) > 1){
  //   delay(5);
  // }
  // Motor.el_stop();

  // calibration loop
  while(1){
    // Motor.az_ccw(); delay(6000);
    // Motor.az_stop(); delay(100);
    // Motor.az_cw(); delay(6000);
    // Motor.az_stop(); delay(100);
    // Motor.el_ccw(); delay(100); Motor.el_stop();
    delay(2000);
  }
}

void el_control(void* pvParameters){
  float el_diff;
  int8_t last_rotation = 0; // -1 = ccw, 0 = idle, 1 = cw
  int8_t el_error = 2;

  const TickType_t xDelay5ms = 5 / portTICK_PERIOD_MS;
  const TickType_t xDelay10ms = 10 / portTICK_PERIOD_MS;
  const TickType_t xDelay100ms = 100 / portTICK_PERIOD_MS;

  for(;;){
    el_diff = target.el - compass.el;
    if(el_diff > el_error){
      Motor.el_ccw();
      last_rotation = -1;
    }else if(el_diff < -el_error){
      Motor.el_cw();
      last_rotation = 1;
    }else{
      if(last_rotation == 0){}
      else if(last_rotation == 1){
        Motor.el_ccw();
        vTaskDelay(xDelay10ms);
      }else{
        Motor.el_cw();
        vTaskDelay(xDelay10ms);
      }
      Motor.el_stop();
      vTaskDelay(xDelay100ms);
      last_rotation = 0;
    }
    vTaskDelay(xDelay5ms);
  }
}

void az_control(void* pvParameters){
  float az_diff;
  int8_t last_rotation = 0; // -1 = ccw, 0 = idle, 1 = cw
  int8_t az_error = 2;

  const TickType_t xDelay5ms = 5 / portTICK_PERIOD_MS;
  const TickType_t xDelay10ms = 10 / portTICK_PERIOD_MS;
  const TickType_t xDelay100ms = 100 / portTICK_PERIOD_MS;

  for(;;){
    az_diff = fmod(target.az - compass.az + 540, 360) - 180;
    if(az_diff > az_error){
      Motor.az_cw();
    last_rotation = 1;
    }else if(az_diff < -az_error){
      Motor.az_ccw();
      last_rotation = -1;
    }else{
      if(last_rotation == 0){}
      else if(last_rotation == 1){
        Motor.az_ccw();
        vTaskDelay(xDelay10ms);
      }else{
        Motor.az_cw();
        vTaskDelay(xDelay10ms);
      }
      Motor.az_stop();
      vTaskDelay(xDelay100ms);
      last_rotation = 0;
    }
    vTaskDelay(xDelay5ms);
  }
}