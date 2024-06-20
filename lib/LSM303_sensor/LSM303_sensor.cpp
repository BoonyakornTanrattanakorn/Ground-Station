#include "LSM303_sensor.h"

LSM303_sensor::LSM303_sensor(){
    
}

void LSM303_sensor::begin(){
    Wire.begin();
    compass.init();
    // see LSM303DLHC datasheet
    compass.writeAccReg(0x20, 0b01100111); // accelerometer 200 Hz ODR
    compass.writeAccReg(0x23, 0b00001000); // (+/- 2 g full scale)
    compass.writeMagReg(0x0, 0b10011100); // enable temp sensor and data output rate 220 Hz
    compass.writeMagReg(0x1, 0b00100000); // (+/- 1.3 gauss full scale)
    compass.writeMagReg(0x2, 0b00000000); // (continuous-conversion mode)
}

void LSM303_sensor::update(){
    compass.read();
    
    raw_az_reading = compass.heading((LSM303::vector<int>){0, 0, -1});
    raw_az_vector[0] = cos(raw_az_reading * DEG_TO_RAD);
    raw_az_vector[1] = sin(raw_az_reading * DEG_TO_RAD);
    estimated_az_vector[0] = azX_estimate.updateEstimate(raw_az_vector[0]);
    estimated_az_vector[1] = azY_estimate.updateEstimate(raw_az_vector[1]);
    az = RAD_TO_DEG * atan2(estimated_az_vector[1], estimated_az_vector[0]);
    az += az < 0 ? 360 : 0;

    raw_el = -RAD_TO_DEG * (atan2(compass.a.z, compass.a.y));
    el = el_estimate.updateEstimate(raw_el);
}

void LSM303_sensor::set_calibration_data(LSM303::vector<int16_t> min, LSM303::vector<int16_t> max){
    compass.m_min = min;
    compass.m_max = max;
}

