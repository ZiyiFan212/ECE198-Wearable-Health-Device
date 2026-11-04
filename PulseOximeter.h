//
// Author: Frank Fan, ZiyiFan212 @GitHub
//
#ifndef PROJECT_PULSE_OXIMETER_H
#define PROJECT_PULSE_OXIMETER_H

#include <Arduino.h>
#include <Wire.h>
#include <MAX30105.h>
#include <spo2_algorithm.h>

namespace PulseOximeterConfig {
    constexpr uint8_t LED_POWER = 0x1F;
    constexpr uint8_t LED_MODE = 2; // Red and infrared; MAX30102 has no green LED.
    
    constexpr int SAMPLE_RATE = 100;
    constexpr uint8_t SAMPLE_AVERAGE = 4;// Should give 100/4=25
    constexpr int PULSE_WIDTH = 411;
    constexpr int ADC_RANGE = 4096;

    // 100 sets of data as one calculation window, taking approximately 4 seconds.
    // Then, updating every second, each requires 25 sets of data.
    constexpr uint16_t WINDOW_SAMPLES = 100;
    constexpr uint16_t UPDATE_SAMPLES = 25;

    static_assert(SAMPLE_RATE / SAMPLE_AVERAGE == FreqS,
                "SpO2 algorithm requires 25 samples per second");
    static_assert(WINDOW_SAMPLES == BUFFER_SIZE, "Algorithm window size mismatch");
}

// Independent implementation of the pulse oximeter logics. Including data recording, updating, and
// parsing. Public APIs provided as below.
class PulseOximeter {
public:
    /*
    // Raw samples for debugging; normal use only needs Result.
    struct DebugReading {
        unsigned long timestampUs; // Time read from FIFO, not acquisition time.
        uint32_t red; // Raw counts, not blood oxygen percentage.
        uint32_t ir;
    };
    // Debug ONLY: consumes one FIFO sample, displays the raw data.
    // True means a new sample; false leaves reading unchanged (not ready/no data).
    // Use instead of update() when inspecting every raw sample.
    bool debugRead(DebugReading &reading) {
        if (!initialized_) return false;
        if (!sensor_.available()) sensor_.check();
        if (!sensor_.available()) return false;
        DebugReading.timestampUs = micros();
        DebugReading.red = sensor_.getFIFORed();
        DebugReading.ir = sensor_.getFIFOIR();
        sensor_.nextSample();
        return true;
    }
    */


    // spo2: Percent; use only when spo2Valid is true.
    // heartRate: BPM; use only when heartRateValid is true.
    struct Result {
        unsigned long timestampUs; 
        int32_t spo2; 
        int32_t heartRate; 
        bool isSPO2Valid;
        bool isHeartRateValid;
    };

    // Initialization. Returns false on init failure.
    bool begin() {
        sampleCount = 0;
        lastSampleTimeStampUs = 0;
        isResultReady = false;
        result_ = {};
        initialized_ = sensor_.begin(Wire, I2C_SPEED_STANDARD);
        if (!initialized_) return false;
        sensor_.setup(PulseOximeterConfig::LED_POWER,
                      PulseOximeterConfig::SAMPLE_AVERAGE,
                      PulseOximeterConfig::LED_MODE,
                      PulseOximeterConfig::SAMPLE_RATE,
                      PulseOximeterConfig::PULSE_WIDTH,
                      PulseOximeterConfig::ADC_RANGE);
        sensor_.clearFIFO();
        return true;
    }


    // Update Result when a full calculation window is ready, follow the rule FIFO.
    void update() {
        if (!initialized_) return;
        sensor_.check();
        while (sensor_.available()) {
            const unsigned long timestampUs = micros();
            const uint32_t red = sensor_.getFIFORed();
            const uint32_t ir = sensor_.getFIFOIR();
            sensor_.nextSample();
            processSample(timestampUs, red, ir);
        }
    }

    // Consume the latest calculated result, including invalid results.
    // False means no new result; output is left unchanged.
    bool getResult(Result &result) {
        if (!isResultReady) return false;
        result = result_;
        isResultReady = false;
        return true;
    }

private:
    MAX30105 sensor_;
    bool initialized_ = false;
    // Our buffers.
    uint32_t redBuffer_[PulseOximeterConfig::WINDOW_SAMPLES] = {};
    uint32_t irBuffer_[PulseOximeterConfig::WINDOW_SAMPLES] = {};

    uint16_t sampleCount = 0;
    unsigned long lastSampleTimeStampUs = 0;
    Result result_ = {};
    bool isResultReady = false;

    void processSample(unsigned long timestampUs, uint32_t red, uint32_t ir) {
        // A long polling gap may lose FIFO samples. Rebuild a full window.
        if (sampleCount > 0 && timestampUs - lastSampleTimeStampUs > 200000UL) {
            sampleCount = 0;
            isResultReady = false;
        }
        lastSampleTimeStampUs = timestampUs;
        redBuffer_[sampleCount] = red;
        irBuffer_[sampleCount] = ir;
        ++sampleCount;
        if (sampleCount < PulseOximeterConfig::WINDOW_SAMPLES) return;

        calculateResult();

        const uint16_t retained = PulseOximeterConfig::WINDOW_SAMPLES - PulseOximeterConfig::UPDATE_SAMPLES;
        for (uint16_t i = 0; i < retained; ++i) {
            redBuffer_[i] = redBuffer_[i + PulseOximeterConfig::UPDATE_SAMPLES];
            irBuffer_[i] = irBuffer_[i + PulseOximeterConfig::UPDATE_SAMPLES];
        }
        sampleCount = retained;
    }

    // Private implementation of SPO2 and heart rate parsing.
    // Heart rate parsing uses library function from MAX301.h.
    void calculateResult() {
        int32_t spo2 = -999;
        int32_t heartRate = -999;
        int8_t spo2Valid = 0;
        int8_t heartRateValid = 0;

        maxim_heart_rate_and_oxygen_saturation(
            irBuffer_, PulseOximeterConfig::WINDOW_SAMPLES, redBuffer_,
            &spo2, &spo2Valid, &heartRate, &heartRateValid);
        result_.timestampUs = micros();
        result_.isSPO2Valid = spo2Valid != 0 && spo2 >= 0 && spo2 <= 100;
        result_.isHeartRateValid = heartRateValid != 0 && heartRate > 0;
        result_.spo2 = result_.isSPO2Valid ? spo2 : -1;
        result_.heartRate = result_.isHeartRateValid ? heartRate : -1;
        isResultReady = true;
    }
};

#endif
