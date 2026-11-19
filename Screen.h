//
// Written by: Frank Fan @github
//

#pragma once

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include "Accelerometer.h"
#include "PulseOximeter.h"

// configuration of the screen.
/// @remark
namespace ScreenConfiguration {
    constexpr uint32_t I2C_CLOCK = 100000;
    constexpr int SCREEN_WIDTH = 128;
    constexpr int SCREEN_HEIGHT = 64;
    constexpr int OLED_RESET = -1;
    constexpr uint8_t OLED_ADDRESS = 0x3C;
    constexpr unsigned long BAUD_RATE = 115200;
    constexpr unsigned long REFRESH_INTERVAL_MS = 500;
    constexpr bool TRACE_OLED = true;
}

class Screen {
public:
    bool Init() {
        Wire.begin();
        Wire.setClock(ScreenConfiguration::I2C_CLOCK);
        Wire.setWireTimeout(25000);

        //Serial.println("OLED init begin");

        OLEDReady_ = display_.begin(
            SSD1306_SWITCHCAPVCC,
            ScreenConfiguration::OLED_ADDRESS,
            true, false
        );
        if (!OLEDReady_) return false;
        //Serial.println("OLED init end");

        clearDisplay();
        display_.setCursor(0, 0);
        display_.println("Initializing...");
        profiling("OLED first transfer begin", "OLED first transfer end, elapsed_us=");
        showingHeartRateOnly_ = false;
        lastRefresh_ = millis();
        return true;
    }


    ///@remark Heart-rate-only display.
    void updateDisplay(const PulseOximeter::Result& pulse, bool forceRefresh = false) {
        const int32_t heartRate = pulse.isHeartRateValid ? pulse.heartRate : -1;
        if (!canRefresh(forceRefresh)) return;
        if (!forceRefresh && showingHeartRateOnly_ && heartRate == lastDisplayedHeartRate_) return;

        clearDisplay();
        display_.setCursor(0, 0);
        display_.println("HEART RATE");
        display_.setTextSize(3);
        display_.setCursor(0, 20);
        print(heartRate);
        display_.setTextSize(1);
        display_.setCursor(0, 52);
        display_.print("BPM");

        refresh();
        lastDisplayedHeartRate_ = heartRate;
        showingHeartRateOnly_ = true;
    }

    ///@remark Combined layout with BPM and accelerations.
    void updateDisplay(const PulseOximeter::Result& pulse,
                       const Accelerometer::Reading& acceleration,
                       bool forceRefresh = false) {
        if (!canRefresh(forceRefresh)) return;

        clearDisplay();
        display_.setCursor(0, 0);
        display_.println("ACCELERATION m/s2");
        display_.setCursor(0, 12);
        display_.print("X: ");
        display_.println(acceleration.x.smoothedAcceleration, 2);
        display_.setCursor(0, 22);
        display_.print("Y: ");
        display_.println(acceleration.y.smoothedAcceleration, 2);
        display_.setCursor(0, 32);
        display_.print("Z: ");
        display_.println(acceleration.z.smoothedAcceleration, 2);

        display_.drawLine(0, 43, ScreenConfiguration::SCREEN_WIDTH - 1, 43, SSD1306_WHITE);
        display_.setCursor(0, 49);
        display_.print("HR:");
        print(pulse.isHeartRateValid ? pulse.heartRate : -1);
        display_.setCursor(70, 49);
        display_.print("O2:");
        print(pulse.isSPO2Valid ? pulse.spo2 : -1);
        if (pulse.isSPO2Valid) display_.print("%");

        refresh();
        showingHeartRateOnly_ = false;
    }

private:
    Adafruit_SSD1306 display_{
        ScreenConfiguration::SCREEN_WIDTH,
        ScreenConfiguration::SCREEN_HEIGHT,
        &Wire,
        ScreenConfiguration::OLED_RESET,
        ScreenConfiguration::I2C_CLOCK,
        ScreenConfiguration::I2C_CLOCK
    };
    bool OLEDReady_ = false;
    bool showingHeartRateOnly_ = false;
    unsigned long lastRefresh_ = 0;
    int32_t lastDisplayedHeartRate_ = -1;

    ///@remark Only refresh if the OLED is ready and the refresh interval is passed.
    bool canRefresh(bool forceRefresh) const {
        return OLEDReady_ &&
               (forceRefresh || millis() - lastRefresh_ >= ScreenConfiguration::REFRESH_INTERVAL_MS);
    }

    void clearDisplay() {
        display_.clearDisplay();
        display_.setTextColor(SSD1306_WHITE);
        display_.setTextSize(1);
    }

    void print(int32_t value) {
        if (value >= 0) {
            display_.print(value);
        } else {
            display_.print("--");
        }
    }

    void refresh() {
        lastRefresh_ = millis();
        profiling("OLED transfer begin", "OLED transfer end, elapsed_us=");
    }

    ///@remark Mostly for debugging usage. Operate if and only if OLED trace is on and the
    ///listed issues occured: slow refresh, low FPS, compilation error.
    void profiling(const char* beginMessage, const char* endMessage) {
        if (ScreenConfiguration::TRACE_OLED) {
            Serial.println(beginMessage);
            Serial.flush();
        }
        const unsigned long startUs = micros();
        display_.display();
        if (ScreenConfiguration::TRACE_OLED) {
            Serial.print(endMessage);
            Serial.println(micros() - startUs);
        }
    }
};
