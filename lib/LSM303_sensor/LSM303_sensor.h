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