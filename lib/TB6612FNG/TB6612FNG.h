#ifndef TB6612FNG_h
#define TB6612FNG_h

#include <Arduino.h>

class TB6612FNG
{
    private:
        uint8_t _AIN2_pin;
        uint8_t _AIN1_pin;
        uint8_t _STBY_pin;
        uint8_t _BIN1_pin;
        uint8_t _BIN2_pin;
    public:
        TB6612FNG();
        void begin(uint8_t AIN2_pin = 25, uint8_t AIN1_pin = 33, uint8_t STBY_pin = 32, uint8_t BIN1_pin = 26, uint8_t BIN2_pin = 27);
        void el_cw();
        void el_ccw();
        void el_stop();
        void az_cw();
        void az_ccw();
        void az_stop();
};

#endif