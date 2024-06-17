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
    Xm_off = compass.m.x*(100000.0/1100.0) - 13363.592201;
    Ym_off = compass.m.y*(100000.0/1100.0) - 20655.611588;
    Zm_off = compass.m.z*(100000.0/980.0) + 9242.456348;

    Xm_cal =  0.000860*Xm_off + 0.000031*Ym_off + 0.000105*Zm_off;
    Ym_cal =  0.000031*Xm_off + 0.000889*Ym_off + -0.000066*Zm_off;
    Zm_cal =  0.000105*Xm_off + -0.000066*Ym_off + 0.001030*Zm_off;
    
    fXm = Xm_cal * alpha + (fXm * (1.0 - alpha));
    fYm = Ym_cal * alpha + (fYm * (1.0 - alpha));
    fZm = Zm_cal * alpha + (fZm * (1.0 - alpha));

    fXm_comp = fXm*cos(pitch)+fZm*sin(pitch);
    fYm_comp = fXm*sin(roll)*sin(pitch)+fYm*cos(roll)-fZm*sin(roll)*cos(pitch);
    
    estimated_az_vector[0] = azX_estimate.updateEstimate(fXm_comp);
    estimated_az_vector[1] = azY_estimate.updateEstimate(fYm_comp);
    az = RAD_TO_DEG * atan2(estimated_az_vector[1], estimated_az_vector[0]);
    az += az < 0 ? 360 : 0;

    // raw_az_reading = compass.heading((LSM303::vector<int>){0, 0, -1});
    // raw_az_vector[0] = cos(raw_az_reading * DEG_TO_RAD);
    // raw_az_vector[1] = sin(raw_az_reading * DEG_TO_RAD);
    // estimated_az_vector[0] = azX_estimate.updateEstimate(raw_az_vector[0]);
    // estimated_az_vector[1] = azY_estimate.updateEstimate(raw_az_vector[1]);
    // az = RAD_TO_DEG * atan2(estimated_az_vector[1], estimated_az_vector[0]);
    // az += az < 0 ? 360 : 0;

    raw_el = RAD_TO_DEG * (atan2(compass.a.z, compass.a.y));
    el = el_estimate.updateEstimate(raw_el);
}

void LSM303_sensor::set_calibration_data(LSM303::vector<int16_t> min, LSM303::vector<int16_t> max){
    compass.m_min = min;
    compass.m_max = max;
}

