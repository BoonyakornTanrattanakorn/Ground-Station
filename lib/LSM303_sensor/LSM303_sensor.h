#ifndef LSM303_sensor_h
#define LSM303_sensor_h

#include <LSM303.h>
#include <Wire.h>
#include <SimpleKalmanFilter.h>

class LSM303_sensor
{
    private:
        // edit these :)

        // don't touch these >:(
        float raw_az_reading = 0;
        float raw_az_vector[2] = {0};
        float estimated_az_vector[2] = {0};
        float raw_el;
        const float alpha = 0.15;
        float fXa = 0;
        float fYa = 0;
        float fZa = 0;
        float fXm = 0;
        float fYm = 0;
        float fZm = 0;
        float pitch, pitch_print, roll, roll_print, Heading, Xa_off, Ya_off, Za_off, Xa_cal, Ya_cal, Za_cal, Xm_off, Ym_off, Zm_off, Xm_cal, Ym_cal, Zm_cal, fXm_comp, fYm_comp;


        SimpleKalmanFilter el_estimate = SimpleKalmanFilter(0.5, 0.5, 0.1);
        SimpleKalmanFilter azX_estimate = SimpleKalmanFilter(10, 10, 10);
        SimpleKalmanFilter azY_estimate = SimpleKalmanFilter(10, 10, 10);

    public:
        LSM303 compass;
        float az;
        float el;

        LSM303_sensor();
        void begin();
        void update();
        void set_calibration_data(LSM303::vector<int16_t> min, LSM303::vector<int16_t> max);
};

#endif