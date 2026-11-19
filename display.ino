#include "Screen.h"

PulseOximeter pulseOximeter;
Screen screen;
PulseOximeter::Result latestPulse = {};
bool pulseSensorConnected = false;

// sample usage of screen.
void setup() {
    Serial.begin(ScreenConfiguration::BAUD_RATE);
    const unsigned long serialStart = millis();
    while (!Serial && millis() - serialStart < 5000) {}
    Serial.println("BOOT: shared I2C test, 100 kHz, OLED 2 Hz");
    Serial.flush();

    if (!screen.Init()) {
        Serial.println("OLED ERROR");
        while (true) {
            delay(100);
        }
    }

    Serial.println("MAX30102 init begin");
    Serial.flush();
    pulseSensorConnected = pulseOximeter.begin();
    if (pulseSensorConnected) {
        Serial.println("MAX30102 ready");
    } else {
        Serial.println("MAX30102 not detected");
    }

    Serial.println("System initialized");
}

void loop() {
    if (pulseSensorConnected) {
        pulseOximeter.update();
        if (pulseOximeter.getResult(latestPulse)) {
            screen.debugPrint(latestPulse);
        }
    }

    screen.updateDisplay(latestPulse);
}
