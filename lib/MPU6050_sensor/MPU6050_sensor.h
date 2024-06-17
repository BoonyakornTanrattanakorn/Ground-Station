#ifndef MPU6050_SENSOR_H
#define MPU6050_SENSOR_H
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

class MPU6050_sensor
{
    private:
        Adafruit_MPU6050 mpu;
        sensors_event_t a, g, temp;

    public:
        MPU6050_sensor();
        void begin();
        void update();

        float el, raw_el;

};

#endif