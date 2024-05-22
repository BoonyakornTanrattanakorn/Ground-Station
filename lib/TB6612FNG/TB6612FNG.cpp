#include "TB6612FNG.h"

TB6612FNG::TB6612FNG(){

}

void TB6612FNG::begin(uint8_t AIN2_pin, uint8_t AIN1_pin, uint8_t STBY_pin, uint8_t BIN1_pin, uint8_t BIN2_pin){
    _AIN2_pin = AIN2_pin;
    _AIN1_pin = AIN1_pin;
    _STBY_pin = STBY_pin;
    _BIN1_pin = BIN1_pin;
    _BIN2_pin = BIN2_pin;
    pinMode(AIN2_pin, OUTPUT);
    pinMode(AIN1_pin, OUTPUT);
    pinMode(STBY_pin, OUTPUT);
    pinMode(BIN1_pin, OUTPUT);
    pinMode(BIN2_pin, OUTPUT);
    digitalWrite(AIN2_pin, LOW);
    digitalWrite(AIN1_pin, LOW);
    digitalWrite(STBY_pin, HIGH);
    digitalWrite(BIN1_pin, LOW);
    digitalWrite(BIN2_pin, LOW);
}

void TB6612FNG::el_cw(){
  digitalWrite(_AIN2_pin, LOW);
  digitalWrite(_AIN1_pin, HIGH);
  //Serial.println("EL-CW");
}

void TB6612FNG::el_ccw(){
  digitalWrite(_AIN2_pin, HIGH);
  digitalWrite(_AIN1_pin, LOW);
  //Serial.println("EL_CCW");
}

void TB6612FNG::el_stop(){
  digitalWrite(_AIN2_pin, LOW);
  digitalWrite(_AIN1_pin, LOW);
  //Serial.println("EL_STOP");
}

void TB6612FNG::az_cw(){
  digitalWrite(_BIN1_pin, LOW);
  digitalWrite(_BIN2_pin, HIGH);
  //Serial.println("AZ-CW");
}

void TB6612FNG::az_ccw(){
  digitalWrite(_BIN1_pin, HIGH);
  digitalWrite(_BIN2_pin, LOW);
  //Serial.println("AZ_CCW");
}

void TB6612FNG::az_stop(){
  digitalWrite(_BIN1_pin, LOW);
  digitalWrite(_BIN2_pin, LOW);
  //Serial.println("AZ_STOP");
}