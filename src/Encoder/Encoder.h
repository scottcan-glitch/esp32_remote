#pragma once

#include <Arduino.h>
#include "driver/pcnt.h"

class Encoder {
public:
    Encoder(gpio_num_t pinA,
            gpio_num_t pinB,
            pcnt_unit_t unit = PCNT_UNIT_0,
            int16_t lowLimit = -32768,
            int16_t highLimit = 32767,
            uint32_t speedUpdateIntervalMs = 10);

    ~Encoder();

    void begin();
    void reset();
    int32_t getCount();
    float getSpeedCPS();
    void setInvert(bool invert) { _invert = invert; }
    bool getInvert() const { return _invert; }
    void setSpeedUpdateRate(uint32_t intervalMs) { _speedUpdateIntervalMs = intervalMs; }
    uint32_t getSpeedUpdateRate() const { return _speedUpdateIntervalMs; }

private:
    gpio_num_t _pinA;
    gpio_num_t _pinB;
    pcnt_unit_t _unit;
    int16_t _lowLimit;
    int16_t _highLimit;
    bool _started;
    bool _invert;

    // Overflow protection. (pcnt hardware counter is 16-bit, extend to 32 bit)
    int16_t _lastRaw;
    int32_t _accum;

    // Speed calculation
    float _lastSpeed;
    uint32_t _lastSpeedTimeMs;
    int32_t  _lastSpeedPos;
    uint32_t _speedUpdateIntervalMs;
};
