#include "CommandProcessor.h"

// Static instances
static L298* motor = nullptr;
static Encoder* encoder = nullptr;

// Motor state
MotorState motorState = {0, true};

// Direction inversion state
static bool motorDirectionInverted = false;

// ===================
// Initialization
// ===================
void initCommandProcessor(L298* motorInstance, Encoder* encoderInstance) {
    motor = motorInstance;
    encoder = encoderInstance;
}

// ===================
// Command Processing
// Format: M{speed}D{direction}
// Examples: 
//   M100D1  -> Closed loop 100 cps, CW
//   M-50D0  -> Closed loop -50 cps, CCW
//   OL150D1 -> Open loop PWM 150, CW
//   STOP    -> Stop motor
// ===================
void processCommand(const String &cmd) {
    if (motor == nullptr || encoder == nullptr) {
        Serial.println("ERR: Not initialized");
        return;
    }
    
    String trimmed = cmd;
    trimmed.trim();
    
    // STOP command
    if (trimmed == "STOP" || trimmed == "S") {
        currentMode = MODE_STOPPED;
        motor->stop();
        Serial.println("OK STOPPED");
        return;
    }
    
    // Set P gain: P{value}
    if (trimmed.startsWith("P")) {
        float kp = trimmed.substring(1).toFloat();
        float ki = motor->getKi();
        motor->setPI(kp, ki);
        Serial.printf("OK Kp=%.3f Ki=%.3f\n", kp, ki);
        return;
    }
    
    // Set I gain: I{value}
    if (trimmed.startsWith("I")) {
        float ki = trimmed.substring(1).toFloat();
        float kp = motor->getKp();
        motor->setPI(kp, ki);
        Serial.printf("OK Kp=%.3f Ki=%.3f\n", kp, ki);
        return;
    }
    
    // Reset integral: RESET or R
    if (trimmed == "RESET" || trimmed == "R") {
        motor->resetIntegral();
        Serial.println("OK Integral reset");
        return;
    }
    
    // Get PI values: GET_PI or G
    if (trimmed == "GET_PI" || trimmed == "G") {
        Serial.printf("Kp=%.3f Ki=%.3f Integral=%.3f\n", 
                     motor->getKp(), motor->getKi(), motor->getIntegral());
        return;
    }
    
    // Get encoder values: GET_ENC or E
    if (trimmed == "GET_ENC" || trimmed == "E") {
        //int32_t pos = encoder->getCount();
        float speed = encoder->getSpeedCPS();
        Serial.printf("%f", speed);
        //Serial.printf("Encoder: Pos=%ld Speed=%.1f cps\n", pos, speed);
        return;
    }
    
    // Invert motor direction: INVERT or INV
    if (trimmed == "INVERT" || trimmed == "INV") {
        motorDirectionInverted = !motorDirectionInverted;
        motor->setDirectionInvert(motorDirectionInverted);
        Serial.printf("OK Motor direction invert: %s (Hardware check: %s)\n", 
                     motorDirectionInverted ? "ON" : "OFF",
                     motor->getDirectionInvert() ? "ON" : "OFF");
        return;
    }
    
    // Invert encoder direction: INVERT_ENC or IE
    if (trimmed == "INVERT_ENC" || trimmed == "IE") {
        static bool encoderInverted = false;
        encoderInverted = !encoderInverted;
        encoder->setInvert(encoderInverted);
        Serial.printf("OK Encoder invert: %s\n", encoderInverted ? "ON" : "OFF");
        return;
    }

    // PI_RATE{ms} - Set PI controller update rate in milliseconds
    if (trimmed.startsWith("PI_RATE")) {
        uint32_t rate = trimmed.substring(7).toInt();
        if (rate >= 5 && rate <= 500) {  // Limit to reasonable range
            motor->setPIUpdateRate(rate);
            Serial.printf("OK PI_RATE=%d\n", rate);
        } else {
            Serial.println("ERR: PI_RATE must be 5-500 ms");
        }
        return;
    }

    // ENC_RATE{ms} - Set encoder speed update rate in milliseconds
    if (trimmed.startsWith("ENC_RATE")) {
        uint32_t rate = trimmed.substring(8).toInt();
        if (rate >= 5 && rate <= 500) {  // Limit to reasonable range
            encoder->setSpeedUpdateRate(rate);
            Serial.printf("OK ENC_RATE=%d\n", rate);
        } else {
            Serial.println("ERR: ENC_RATE must be 5-500 ms");
        }
        return;
    }

    // GET_RATES / GR - Get current PI and encoder update rates
    if (trimmed == "GET_RATES" || trimmed == "GR") {
        Serial.printf("PI_RATE=%d ENC_RATE=%d\n", 
                     motor->getPIUpdateRate(), 
                     encoder->getSpeedUpdateRate());
        return;
    }
    
    // Open Loop: OL{pwm}D{dir} or O{pwm}D{dir}
    if (trimmed.startsWith("OL") || trimmed.startsWith("O")) {
        int mIdx = trimmed.startsWith("OL") ? 2 : 1;
        int dIdx = trimmed.indexOf('D', mIdx);
        
        if (dIdx == -1) {
            Serial.println("ERR: Invalid format");
            return;
        }
        
        int pwm = trimmed.substring(mIdx, dIdx).toInt();
        bool direction = trimmed.substring(dIdx + 1).toInt() != 0;
        
        if (pwm < 0 || pwm > 255) {
            Serial.println("ERR: PWM 0-255");
            return;
        }
        
        currentMode = MODE_OL;
        motorState.targetSpeed = pwm;
        motorState.cwDirection = direction;
        
        Serial.printf("OK OL PWM=%d DIR=%d\n", pwm, direction ? 1 : 0);
        return;
    }
    
    // Closed Loop: M{speed_cps}D{dir} or CL{speed}D{dir}
    if (trimmed.startsWith("M") || trimmed.startsWith("CL")) {
        int mIdx = trimmed.startsWith("CL") ? 2 : 1;
        int dIdx = trimmed.indexOf('D', mIdx);
        
        if (dIdx == -1) {
            Serial.println("ERR: Invalid format");
            return;
        }
        
        int speedCps = trimmed.substring(mIdx, dIdx).toInt();
        bool direction = trimmed.substring(dIdx + 1).toInt() != 0;
        
        // Apply direction sign to speed for PI controller
        int signedSpeed = direction ? speedCps : -speedCps;
        
        currentMode = MODE_CL;
        motorState.targetSpeed = speedCps;
        motorState.cwDirection = direction;
        motor->setTargetSpeed(signedSpeed);
        
        Serial.printf("OK CL SPEED=%d DIR=%d\n", speedCps, direction ? 1 : 0);
        return;
    }
    
    Serial.println("ERR: Unknown command");
}
