
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include "Accelerometer.h"
#include "PulseOximeter.h"

// OLED settings
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define OLED_ADDRESS 0x3C

Adafruit_SSD1306 display(
    SCREEN_WIDTH,
    SCREEN_HEIGHT,
    &Wire,
    OLED_RESET
);

// Sensors
Accelerometer accelerometer;
PulseOximeter pulseOximeter;

// Accelerometer values (m/s^2)
float accelX = 0.0f;
float accelY = 0.0f;
float accelZ = 0.0f;

// Pulse sensor values
int heartRate = -1;
int spo2 = -1;

// Timing
unsigned long lastAccelUpdate = 0;
unsigned long lastDisplayUpdate = 0;

const unsigned long ACCEL_INTERVAL = 200;
const unsigned long DISPLAY_INTERVAL = 250;

// bool pulseSensorConnected = false;

void setup() {

    Serial.begin(115200);

    Wire.begin();

    // Initialize OLED
    if (!display.begin(
            SSD1306_SWITCHCAPVCC,
            OLED_ADDRESS
        )) {

        Serial.println("OLED ERROR");

        while (true) {
            delay(100);
        }
    }

    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);

    display.setCursor(0, 0);
    display.println("Initializing...");
    display.display();

    // Initialize accelerometer
    accelerometer.begin();

    // Initialize pulse sensor
    pulseSensorConnected = pulseOximeter.begin();

    if (!pulseSensorConnected) {
        Serial.println("MAX30102 not detected");
    } else {
        Serial.println("MAX30102 ready");
    }

    Serial.println("System initialized");
}

void loop() {

    unsigned long currentTime = millis();

    // Continuously collect pulse sensor samples
    if (pulseSensorConnected) {
    
        pulseOximeter.update();
        PulseOximeter::Result result;
    
        if (pulseOximeter.getResult(result)) {
            heartRate =
            result.isHeartRateValid
            ? result.heartRate
            : -1;

            spo2 =
            result.isSPO2Valid
            ? result.spo2
            : -1;
        }
    }

    // Accelerometer update
    if (currentTime - lastAccelUpdate
        >= ACCEL_INTERVAL) {

        lastAccelUpdate = currentTime;

        Accelerometer::Reading reading =
            accelerometer.read();

        accelX =
            reading.x.smoothedAcceleration;

        accelY =
            reading.y.smoothedAcceleration;

        accelZ =
            reading.z.smoothedAcceleration;
    }

    // OLED refresh
    if (currentTime - lastDisplayUpdate
        >= DISPLAY_INTERVAL) {

        lastDisplayUpdate = currentTime;

        updateOLED();
    }
}

void updateOLED() {

    display.clearDisplay();

    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);

    // Accelerometer heading
    display.setCursor(0, 0);
    display.println("ACCELERATION m/s2");

    display.setCursor(0, 12);
    display.print("X: ");
    display.println(accelX, 2);

    display.setCursor(0, 22);
    display.print("Y: ");
    display.println(accelY, 2);

    display.setCursor(0, 32);
    display.print("Z: ");
    display.println(accelZ, 2);

    // Divider
    display.drawLine(
        0, 43,
        SCREEN_WIDTH - 1, 43,
        SSD1306_WHITE
    );

    Heart rate
    display.setCursor(0, 49);
    display.print("HR:");

    if (heartRate >= 0) {
        display.print(heartRate);
    } else {
        display.print("--");
    }

    SpO2
    display.setCursor(70, 49);
    display.print("O2:");
    
    if (spo2 >= 0) {
        display.print(spo2);
        display.print("%");
    } else {
        display.print("--");
    }

    display.display();
}
