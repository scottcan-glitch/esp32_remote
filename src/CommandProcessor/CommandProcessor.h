#pragma once
#include <Arduino.h>
#include "../Serial/serial.h"
#include "../L298/L298.h"
#include "../Encoder/Encoder.h"

// ===================
// Motor control state
// ===================
struct MotorState {
    int targetSpeed;     // For CL mode: counts/sec, For OL mode: PWM 0-255
    bool cwDirection;
};

// External access to motor state
extern MotorState motorState;

// ===================
// Command Processor Functions
// ===================

// Main command processing function
void processCommand(const String &cmd);

// Initialize command processor with motor and encoder instances
void initCommandProcessor(L298* motorInstance, Encoder* encoderInstance);
