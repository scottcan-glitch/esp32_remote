#include <Arduino.h>

String rxLine;

// Define states for esp32
enum ControlMode {
    MODE_IDLE,
    MODE_FAST_BLINK,
    MODE_SLOW_BLINK
};

// onSerialEvent ISR will modify this variable
volatile ControlMode currentMode = MODE_IDLE;


void setMode(String m) {
    if (m == "MODE_IDLE") currentMode = MODE_IDLE;
    else if (m == "MODE_FAST_BLINK") currentMode = MODE_FAST_BLINK;
    else if (m == "MODE_SLOW_BLINK") currentMode = MODE_SLOW_BLINK;
    else {
        Serial.println("ERR invalid mode. Must belong to ControlMode enum");
        return;
    }

    Serial.print("Valid mode set to: ");
    Serial.println(m);
}

// when '\n' is received, this function called, process the command
void processCommand(String cmd) {
    cmd.trim();

    if (cmd.startsWith("SET_MODE")) {
        setMode(cmd.substring(9));
    }
    /*
    else if (cmd.startsWith("SET_PARAM")) {
        setParam(cmd.substring(10));
    }
    */
    else {
        Serial.println("ERR UNKNOWN_CMD");
    }
}

void handleSerial() {
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n') {
            processCommand(rxLine);
            rxLine = "";
        } else {
            rxLine += c;
        }
    }
}






void onSerialEvent() {
    Serial.println("Data received:");
    // This ISR is called when data is received on the serial port. while loop
    // grabs all available data from buffer, byte at a time.

    handleSerial();
    /*
    while (Serial.available()) {
        char byteReceived = Serial.read();
        Serial.write(byteReceived);  // echo the byte back
        fullString += byteReceived;
    }
        */
    //return fullString;
}



void setup() {
    pinMode(2, OUTPUT);

    // Setup ISR to handle incoming serial data
    Serial.begin(115200);
    Serial.onReceive(onSerialEvent);
    while (!Serial) {
        ; // wait for serial port to connect. Needed for native USB
    }

}

void loop() {

    Serial.print("In main loop. currentMode: " + String(currentMode));
    Serial.println("");
    // STATE MACHINE!
    switch (currentMode) {
        case MODE_IDLE:
            Serial.println("IDLE mode - LED ON");
            digitalWrite(2, HIGH);
            delay(100);
            break;
        case MODE_FAST_BLINK:
            // fast blink LED
            Serial.println("MODE_FAST_BLINK mode");
            digitalWrite(2, HIGH);
            delay(100);
            digitalWrite(2, LOW);
            delay(100);
            break;
        case MODE_SLOW_BLINK:
            // slow blink LED
            digitalWrite(2, HIGH);
            delay(1000);
            digitalWrite(2, LOW);
            delay(1000);
            break;
        default:
            //digitalWrite(2, LOW);
            //delay(100);
            break;
    }

/*
    Serial.println("Full string so far: " + fullString);

    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');
        Serial.print("string from pi is: ");
        Serial.println(input);
    }
*/
}