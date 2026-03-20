#pragma once
#include <Arduino.h>

// ===================
// Control mode enum
// ===================
enum ControlMode {
    MODE_STOPPED,
    MODE_OL,  // Open Loop
    MODE_CL   // Closed Loop
};

// ===================
// Serial interface
// ===================

void SerialInit();                 // initialize serial
void SerialPoll();                 // call this in loop() to handle incoming bytes
void RegisterCommandHandler(void (*handler)(const String &)); // set callback for processing commands

// buffer size for received commands
constexpr size_t RX_BUFFER_SIZE = 64;

// ===================
// Exposed shared variables
// ===================
extern volatile ControlMode currentMode;
