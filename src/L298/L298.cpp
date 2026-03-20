#include "L298.h"
#include "../Encoder/Encoder.h"

L298::L298(uint8_t pinENA, uint8_t pinIN1, uint8_t pinIN2)
    : _ENA(pinENA), _IN1(pinIN1), _IN2(pinIN2),
      _forward(true), _invertDirection(false), _targetSpeed(0), _Kp(1.0f), _Ki(0.1f), _integral(0.0f),
      _lastUpdate(0), _piControllerUpdateIntervalMs(15)
{
    pinMode(_ENA, OUTPUT);
    pinMode(_IN1, OUTPUT);
    pinMode(_IN2, OUTPUT);
    
    // ESP32 PWM setup - 5kHz, 8-bit resolution
    ledcSetup(0, 5000, 8);  // PWM channel 0, 5kHz, 8-bit
    ledcAttachPin(_ENA, 0);  // Attach ENA pin to PWM channel 0
    
    stop();
}

// Set direction
void L298::setDirection(bool forward) {
    _forward = forward;
    // Apply inversion if enabled
    if (_invertDirection) forward = !forward;
    digitalWrite(_IN1, forward ? LOW : HIGH);
    digitalWrite(_IN2, forward ? HIGH: LOW);
}

// Set PWM speed
void L298::setSpeed(uint8_t pwm) {
    ledcWrite(0, pwm);  // Write to PWM channel 0
}

// Set PWM speed with direction (signed)
void L298::setSpeed(int signedPwm) {
    bool forward = (signedPwm >= 0);
    uint8_t pwm = abs(signedPwm);
    if (pwm > 255) pwm = 255;
    setDirection(forward);
    ledcWrite(0, pwm);  // Write to PWM channel 0
}

// Stop motor
void L298::stop() {
    digitalWrite(_IN1, LOW);
    digitalWrite(_IN2, LOW);
    ledcWrite(0, 0);  // Write 0 to PWM channel 0
}

// Set target speed for PI controller
void L298::setTargetSpeed(float target) {
    _targetSpeed = target;
}

// Set PI parameters
void L298::setPI(float Kp, float Ki) {
    _Kp = Kp;
    _Ki = Ki;
}

// Reset integral term
void L298::resetIntegral() {
    _integral = 0.0f;
    _lastUpdate = 0;
}

// Call periodically to update motor speed using PI
void L298::updateSpeedControl(Encoder& enc, unsigned long now)
{
    if (_lastUpdate == 0) {
        _lastUpdate = now;
        return;
    }

    float dt = (now - _lastUpdate) * 0.001f; // ms → seconds
    
    // Rate limit: only update PI controller every _piControllerUpdateIntervalMs
    if (dt < _piControllerUpdateIntervalMs * 0.001f) {
        return;  // Skip this update, too soon
    }
    
    _lastUpdate = now;

    if (dt > 0.1f) return; // guard against bad timing

    float actualSpeed = enc.getSpeedCPS();   // counts/sec
    float error = _targetSpeed - actualSpeed;

    // --- PI calculation ---
    float proportional = _Kp * error;
    float integralCandidate = _integral + error * dt;
    
    // Clamp integral term to prevent excessive windup
    const float maxIntegral = 100.0f;  // Limit integral contribution
    if (integralCandidate > maxIntegral) integralCandidate = maxIntegral;
    if (integralCandidate < -maxIntegral) integralCandidate = -maxIntegral;
    
    float integralTerm = _Ki * integralCandidate;
    float output = proportional + integralTerm;

    // Debug output every 1 second
    static unsigned long lastDebug = 0;
    if (now - lastDebug >= 1000) {
        Serial.printf("PI_DEBUG: Target=%.1f Actual=%.1f Err=%.1f P=%.1f I=%.1f Out=%.1f\n",
                     _targetSpeed, actualSpeed, error, proportional, integralTerm, output);
        lastDebug = now;
    }

    // Direction from sign
    bool forward = (output >= 0.0f);
    float pwm = fabs(output);

    // --- Saturation + anti-windup ---
    if (pwm > 255.0f) {
        pwm = 255.0f;
        // ❌ reject integral growth while saturated
    } else {
        // ✅ accept integral only if not saturated
        _integral = integralCandidate;
    }

    setDirection(forward);
    ledcWrite(0, (uint8_t)pwm);  // Write to PWM channel 0
}

