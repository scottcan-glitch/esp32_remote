#include <Arduino.h>
#include "Serial/serial.h"
#include "Encoder/Encoder.h"
#include "L298/L298.h"
#include "CommandProcessor/CommandProcessor.h"

// LED pin
constexpr int LED_PIN = 2;

// ESP32-S3-DevKitC safe GPIOs
#define ENC_A GPIO_NUM_12
#define ENC_B GPIO_NUM_13

Encoder encoder(ENC_A, ENC_B, PCNT_UNIT_0);

L298 motor(40, 41, 42); // ENA, IN1, IN2

// ===================
// Setup & loop
// ===================
void setup() {
    delay(1000);
    pinMode(LED_PIN, OUTPUT);

    SerialInit();
    
    // Initialize command processor with motor and encoder
    initCommandProcessor(&motor, &encoder);
    RegisterCommandHandler(processCommand);

    // Start the encoder
    encoder.begin();

    // Setup motor/L298 PI gains
    // Tuned for 12V motor reaching ~4000 cps max speed
    // PWM output is 0-255, error in thousands of cps, so need small gains
    motor.setPI(0.05, 0.01);  // Kp=0.05, Ki=0.01
    
    Serial.println("ESP32 Motor Controller Ready");
    Serial.println("Commands: M{speed}D{dir}, OL{pwm}D{dir}, STOP");
}

void loop() {
    SerialPoll(); // Check for new commands

    uint32_t now = millis();
    
    // Blink LED to show activity
    static uint32_t lastBlink = 0;
    if (now - lastBlink >= 500) {
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        lastBlink = now;
    }

    // ===================
    // State machine
    // ===================
    switch (currentMode) {
        case MODE_STOPPED:
            // Motor already stopped by command handler
            break;

        case MODE_OL: {
            // Open loop - use PWM directly from motorState
            extern MotorState motorState;
            int signedPwm = motorState.cwDirection ? motorState.targetSpeed : -motorState.targetSpeed;
            motor.setSpeed(signedPwm);

            // Debug output every 500ms
            static uint32_t lastPrintOL = 0;
            if (now - lastPrintOL >= 500) {
                Serial.printf("OL: PWM=%d DIR=%d\n", motorState.targetSpeed, motorState.cwDirection ? 1 : 0);
                lastPrintOL = now;
            }
            break;
        }

        case MODE_CL: {
            // Closed loop - update speed control continuously
            motor.updateSpeedControl(encoder, now);
            
            // Print encoder feedback and PI state every 500ms
            static uint32_t lastPrint = 0;
            if (now - lastPrint >= 500) {
                int32_t pos = encoder.getCount();
                float speed = encoder.getSpeedCPS();
                extern MotorState motorState;
                Serial.printf("CL: Target=%d Actual=%.1f Integral=%.2f\n", 
                             motorState.targetSpeed, speed, motor.getIntegral());
                lastPrint = now;
            }
            break;
        }
    }
}
