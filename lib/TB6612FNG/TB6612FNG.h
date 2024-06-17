#ifndef TB6612FNG_h
#define TB6612FNG_h

#include <Arduino.h>

class TB6612FNG
{
    private:
        uint8_t _PWMA_pin;
        uint8_t _AIN2_pin;
        uint8_t _AIN1_pin;
        uint8_t _STBY_pin;
        uint8_t _BIN1_pin;
        uint8_t _BIN2_pin;
        uint8_t _PWMB_pin;

        const int PWMA_channel = 0;
        const int PWMB_channel = 1;

        const int PWM_FREQ = 10000;
        const int PWM_RESOLUTION = 8;
        const int MAX_DUTY_CYCLE = pow(2, PWM_RESOLUTION) - 1;
    public:
        TB6612FNG();
        void begin(uint8_t PWMA_pin = 33, uint8_t AIN2_pin = 25, uint8_t AIN1_pin = 26, uint8_t STBY_pin = 27, uint8_t BIN1_pin = 14, uint8_t BIN2_pin = 12, uint8_t PWMB_pin = 13);
        void el_cw();
        void el_ccw();
        void el_stop();
        void set_el_PWM(float duty_cycle_percentage);
        void az_cw();
        void az_ccw();
        void az_stop();
        void set_az_PWM(float duty_cycle_percentage);
};

#endif