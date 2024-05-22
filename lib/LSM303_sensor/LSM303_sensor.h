#ifndef LSM303_sensor_h
#define LSM303_sensor_h

#include <LSM303.h>
#include <Wire.h>
#include <SimpleKalmanFilter.h>

class LSM303_sensor
{
    private:
        // edit these :)
        int moving_average_window = 5;

        // don't touch these >:(
        float az_reading = 0;
        float current_az_vector[2] = {0};
        const double S = 2.0 / (1.0 + moving_average_window); // Smoothing factor for exponential moving average

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