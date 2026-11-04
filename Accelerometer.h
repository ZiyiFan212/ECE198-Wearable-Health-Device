//
// Author: Frank Fan, ZiyiFan212 @GitHub
//
#ifndef PROJECT_ACCELEROMETER_H
#define PROJECT_ACCELEROMETER_H
#include <Arduino.h>

namespace AccelerometerConfig {
constexpr uint8_t ADC_BITS = 10;
constexpr float ADC_REFERENCE_VOLTAGE = 5.0f; // ADC reference, not sensor supply.
constexpr float ADC_MAX = (1UL << ADC_BITS) - 1;

constexpr uint8_t SAMPLES = 10;
constexpr unsigned int SAMPLE_DELAY_US = 500;
constexpr float GRAVITY = 9.81f; // Standard gravity of 9.81 m/s^2
constexpr float FILTER_ALPHA = 0.5f; // Previous reading's weight.



// Since different axis has different resting voltage (it represents 0.0g!),
// we encapsolate the pin, resting volatge, and sensitivity all together.
// int pin: Analog pin on the Arduino.
// float zeroVoltage: resting voltage (i.e. 1.61f -> 1.61V).
// voltsPerG: sensitivity, used in smoothing function.
struct Axis { 
    int pin; 
    float zeroVoltage; 
    float voltsPerG; 
};
// X/Y zero voltages assume the corresponding axes were horizontal.
// X/Y sensitivities are provisional: calibrate each axis separately.
constexpr Axis X = {A5, 1.681f, 0.850f};
constexpr Axis Y = {A4, 1.885f, 0.850f};
constexpr Axis Z = {A3, 1.441f, 0.850f}; // Existing Z calibration.
}



// All public API of the accelerometer. When editing, explictly show the contract to the caller.
// Do NOT expose complicated cleaning logic to public usage!
class Accelerometer {
public:
    struct AxisReading {
        float raw; // Mean ADC count before exponential filtering.
        float voltage; 
        float g; // Includes gravity conversion.
        float acceleration; // m/s^2 before exponential filtering.
        float smoothedAcceleration; // m/s^2 smoothed
    };


    struct Reading {
        unsigned long timestampUs; // Start of sequential XYZ acquisition.
        AxisReading x, y, z;
    };

    /* Initialize pins to read data */
    void begin() {
        analogReadResolution(AccelerometerConfig::ADC_BITS);
        pinMode(AccelerometerConfig::X.pin, INPUT);
        pinMode(AccelerometerConfig::Y.pin, INPUT);
        pinMode(AccelerometerConfig::Z.pin, INPUT);
        initialized_ = false;
        // SL is wired to 3.3V; 0G is unconnected.
    }

    // Read three axis data.
    // Return a struct containing the timestamp and axis reading:
    // Reading.timestampUs: XYZ acquisition
    // Reading.AxisReading.x/y/z: acceration reading.
    Reading read() {
        Reading result;
        result.timestampUs = micros();
        result.x = readAxis(AccelerometerConfig::X, previousX_);
        result.y = readAxis(AccelerometerConfig::Y, previousY_);
        result.z = readAxis(AccelerometerConfig::Z, previousZ_);
        initialized_ = true;
        return result;
    }




private:
    bool initialized_ = false;
    float previousX_ = 0, previousY_ = 0, previousZ_ = 0;


    AxisReading readAxis(const AccelerometerConfig::Axis &axis, float &previous) {
        unsigned long sum = 0;
        for (uint8_t i = 0; i < AccelerometerConfig::SAMPLES; ++i) {
            sum += analogRead(axis.pin);
            delayMicroseconds(AccelerometerConfig::SAMPLE_DELAY_US);
        }


        AxisReading result;
        result.raw = sum / static_cast<float>(AccelerometerConfig::SAMPLES);
        result.voltage = result.raw * AccelerometerConfig::ADC_REFERENCE_VOLTAGE
                         / AccelerometerConfig::ADC_MAX;
        result.g = (result.voltage - axis.zeroVoltage) / axis.voltsPerG;
        result.acceleration = result.g * AccelerometerConfig::GRAVITY;


        // Seed from the first measurement to avoid an artificial startup ramp.
        result.smoothedAcceleration = initialized_
            ? AccelerometerConfig::FILTER_ALPHA * previous
              + (1.0f - AccelerometerConfig::FILTER_ALPHA) * result.acceleration
            : result.acceleration;
        previous = result.smoothedAcceleration;
        return result;
    }
};
#endif
