#include "Encoder.h"

Encoder::Encoder(gpio_num_t pinA,
                 gpio_num_t pinB,
                 pcnt_unit_t unit,
                 int16_t lowLimit,
                 int16_t highLimit,
                 uint32_t speedUpdateIntervalMs)
    : _pinA(pinA),
      _pinB(pinB),
      _unit(unit),
      _lowLimit(lowLimit),
      _highLimit(highLimit),
      _started(false),
      _invert(false),
      _speedUpdateIntervalMs(speedUpdateIntervalMs)
{
}

Encoder::~Encoder()
{
    if (_started) {
        pcnt_counter_pause(_unit);
        pcnt_counter_clear(_unit);
        _started = false;
    }
}


void Encoder::begin()
{
    pcnt_config_t pcnt_config = {};

    // Channel 0
    pcnt_config.pulse_gpio_num = _pinA;
    pcnt_config.ctrl_gpio_num  = _pinB;
    pcnt_config.unit           = _unit;
    pcnt_config.channel        = PCNT_CHANNEL_0;
    pcnt_config.pos_mode       = PCNT_COUNT_INC;
    pcnt_config.neg_mode       = PCNT_COUNT_DEC;
    pcnt_config.lctrl_mode     = PCNT_MODE_REVERSE;
    pcnt_config.hctrl_mode     = PCNT_MODE_KEEP;
    pcnt_config.counter_h_lim  = _highLimit;
    pcnt_config.counter_l_lim  = _lowLimit;

    pcnt_unit_config(&pcnt_config);

    // Channel 1
    pcnt_config.pulse_gpio_num = _pinB;
    pcnt_config.ctrl_gpio_num  = _pinA;
    pcnt_config.channel        = PCNT_CHANNEL_1;
    pcnt_config.pos_mode       = PCNT_COUNT_DEC;
    pcnt_config.neg_mode       = PCNT_COUNT_INC;
    pcnt_config.lctrl_mode     = PCNT_MODE_REVERSE;
    pcnt_config.hctrl_mode     = PCNT_MODE_KEEP;

    pcnt_unit_config(&pcnt_config);

    pcnt_counter_pause(_unit);
    pcnt_counter_clear(_unit);
    pcnt_counter_resume(_unit);

    _lastSpeedTimeMs = millis();
    _lastSpeedPos    = _accum;  // use software-extended position
    _lastSpeed       = 0.0f;


    _started = true;
}

void Encoder::reset()
{
    pcnt_counter_clear(_unit);
    pcnt_get_counter_value(_unit, &_lastRaw);
    _accum = 0;
    _lastSpeedPos = 0;
    _lastSpeedTimeMs = millis();
    _lastSpeed = 0.0f;
}


int32_t Encoder::getCount()
{
    int16_t raw = 0;
    pcnt_get_counter_value(_unit, &raw);

    int32_t delta = (int32_t)raw - (int32_t)_lastRaw;

    _accum += delta;
    _lastRaw = raw;

    return _invert ? -_accum : _accum;
}


float Encoder::getSpeedCPS() //counts per second
{
    uint32_t now = millis();
    uint32_t dt_ms = now - _lastSpeedTimeMs;

    if (dt_ms < _speedUpdateIntervalMs) { // only update every 10ms (100Hz)
        return _lastSpeed; // return last computed speed
    }
    

    int32_t pos = getCount();
    int32_t delta = pos - _lastSpeedPos;

    _lastSpeed = (float)delta / ((float)dt_ms / 1000.0f);

    _lastSpeedPos = pos;
    _lastSpeedTimeMs = now;

    return _lastSpeed;
}


