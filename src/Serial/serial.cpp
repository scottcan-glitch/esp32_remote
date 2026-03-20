#include "serial.h"

// ===================
// Internal buffer variables
// ===================
volatile char rxBuffer[RX_BUFFER_SIZE];
volatile uint8_t rxHead = 0;
volatile uint8_t rxTail = 0;
volatile bool commandReady = false;

static void (*commandHandler)(const String &) = nullptr;

// ===================
// Shared state
// ===================
volatile ControlMode currentMode = MODE_STOPPED;

// ===================
// ISR for serial RX (works only with native USB serial)
// ===================
void IRAM_ATTR SerialISR() {
    while (Serial.available()) {
        char c = Serial.read();
        uint8_t nextHead = (rxHead + 1) % RX_BUFFER_SIZE;

        if (nextHead != rxTail) { // buffer not full
            rxBuffer[rxHead] = c;
            rxHead = nextHead;
            if (c == '\n') {
                commandReady = true;
            }
        }
    }
}

// ===================
// Public functions
// ===================
void SerialInit() {
    Serial.begin(115200);
    while (!Serial) { ; } // wait for native USB

    // Attach ISR for native USB boards; for USB-UART, just call SerialPoll()
    Serial.onReceive(SerialISR);
}

void SerialPoll() {
    // Only process if a full command arrived
    if (commandReady && commandHandler != nullptr) {
        String cmd;
        while (rxTail != rxHead) {
            char c = rxBuffer[rxTail];
            rxTail = (rxTail + 1) % RX_BUFFER_SIZE;
            if (c == '\n') break;
            cmd += c;
        }

        commandHandler(cmd); // safe context
        commandReady = false;
    }
}

void RegisterCommandHandler(void (*handler)(const String &)) {
    commandHandler = handler;
}
