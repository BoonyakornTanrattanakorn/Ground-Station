#include <MPU6050_sensor.h>

#define DEBUG 1

#if DEBUG
    #define debug(x) Serial.print(x)
    #define debugln(x) Serial.println(x)
#else
    #define debug(x)
    #define debugln(x)
#endif

MPU6050_sensor::MPU6050_sensor()
{
}

void MPU6050_sensor::begin(){
    if (!mpu.begin()) {
        debugln("Failed to find MPU6050 chip");
        return;
    }
    debugln("MPU6050 begin.");
}

void MPU6050_sensor::update(){
    mpu.getEvent(&a, &g, &temp);

    raw_el = RAD_TO_DEG * (atan2(a.acceleration.y, -a.acceleration.x));
    // el = el_estimate.updateEstimate(raw_el);
}

// Serial.print("Acceleration X: ");
// Serial.print(a.acceleration.x);
// Serial.print(", Y: ");
// Serial.print(a.acceleration.y);
// Serial.print(", Z: ");
// Serial.print(a.acceleration.z);
// Serial.println(" m/s^2");

// Serial.print("Rotation X: ");
// Serial.print(g.gyro.x);
// Serial.print(", Y: ");
// Serial.print(g.gyro.y);
// Serial.print(", Z: ");
// Serial.print(g.gyro.z);
// Serial.println(" rad/s");

// Serial.print("Temperature: ");
// Serial.print(temp.temperature);
// Serial.println(" degC");