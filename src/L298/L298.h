#pragma once
#include <Arduino.h>

class Encoder; // forward declaration

class L298 {
public:
    // Constructor
    L298(uint8_t pinENA, uint8_t pinIN1, uint8_t pinIN2);

    // Basic control
    void setDirection(bool forward);       // true = forward, false = backward
    void setSpeed(uint8_t pwm);            // 0-255 PWM
    void setSpeed(int signedPwm);          // -255 to +255 (handles direction)
    void stop();                            // Stop motor

    // PI speed control
    void setTargetSpeed(float target);     // counts/sec
    void updateSpeedControl(Encoder& enc, unsigned long now); // call periodically

    // PI parameters
    void setPI(float Kp, float Ki);
    void resetIntegral();              // Reset integral term
    float getKp() const { return _Kp; }
    float getKi() const { return _Ki; }
    float getIntegral() const { return _integral; }
    void setDirectionInvert(bool invert) { _invertDirection = invert; }
    bool getDirectionInvert() const { return _invertDirection; }
    void setPIUpdateRate(uint32_t intervalMs) { _piControllerUpdateIntervalMs = intervalMs; }
    uint32_t getPIUpdateRate() const { return _piControllerUpdateIntervalMs; }

private:
    uint8_t _ENA, _IN1, _IN2;
    bool _forward;
    bool _invertDirection;
    float _targetSpeed;

    // PI controller state
    float _Kp, _Ki;
    float _integral;
    uint32_t _piControllerUpdateIntervalMs; // Update interval in ms
    unsigned long _lastUpdate; // millis of last update
};
