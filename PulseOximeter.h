//
// Author: Frank Fan, ZiyiFan212 @GitHub
//
#pragma once


#ifndef PROJECT_PULSE_OXIMETER_H
#define PROJECT_PULSE_OXIMETER_H

#include <Arduino.h>
#include <Wire.h>
#include <MAX30105.h>
#include <spo2_algorithm.h>

namespace PulseOximeterConfig {
    // 32 hardware FIFO slots at 25 pairs/s hold about 1.28 seconds.
    // A 200 ms read-time gap is not sample loss when FIFO data is retained.
    constexpr unsigned long MAX_POLL_GAP_US = 1000000UL;
    constexpr bool TRACE_SAMPLING = true;
    constexpr bool TRACE_CALCULATION = true; // Temporary serial diagnostics.
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
        totalSamples_ = windowResets_ = maxGapUs_ = 0;
        lastTraceMs_ = 0;
        lastSampleTimeStampUs = 0;
        isResultReady = false;
        result_ = {};
        previousHeartRate_ = 0;
        acceptedAtMs_ = 0;
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
        if (PulseOximeterConfig::TRACE_SAMPLING && millis() - lastTraceMs_ >= 1000) {
            lastTraceMs_ = millis();
            Serial.print("pulse samples=");
            Serial.print(totalSamples_);
            Serial.print(", window=");
            Serial.print(sampleCount);
            Serial.print(", resets=");
            Serial.print(windowResets_);
            Serial.print(", max_gap_us=");
            Serial.println(maxGapUs_);
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
    static constexpr int32_t MAX_HEART_RATE_CHANGE = 35; // BPM tolerance
    static constexpr uint32_t REFERENCE_TIMEOUT_MS = 5000;
    static constexpr uint8_t MAX_INTERVAL_DEVIATION_PERCENT = 40;

    MAX30105 sensor_;
    bool initialized_ = false;
    // Our buffers.
    uint32_t redBuffer_[PulseOximeterConfig::WINDOW_SAMPLES] = {};
    uint32_t irBuffer_[PulseOximeterConfig::WINDOW_SAMPLES] = {};

    unsigned long totalSamples_ = 0, windowResets_ = 0;
    unsigned long maxGapUs_ = 0, lastTraceMs_ = 0;
    uint16_t sampleCount = 0;
    unsigned long lastSampleTimeStampUs = 0;
    int32_t previousHeartRate_ = 0;
    uint32_t acceptedAtMs_ = 0;
    Result result_ = {};
    bool isResultReady = false;

    void processSample(unsigned long timestampUs, uint32_t red, uint32_t ir) {
        const unsigned long gapUs = totalSamples_ > 0 ? timestampUs - lastSampleTimeStampUs : 0;
        if (gapUs > maxGapUs_) maxGapUs_ = gapUs;
        ++totalSamples_;
        // Timestamps describe FIFO reads, not the individual acquisition times.
        if (sampleCount > 0 && gapUs > PulseOximeterConfig::MAX_POLL_GAP_US) {
            ++windowResets_;
            sampleCount = 0;
            previousHeartRate_ = 0;
            acceptedAtMs_ = 0;
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

    bool hasConsistentIntervals() const {
        using PulseOximeterConfig::WINDOW_SAMPLES;
        int32_t signal[WINDOW_SAMPLES];
        uint64_t sum = 0;
        for (uint16_t i = 0; i < WINDOW_SAMPLES; ++i) sum += irBuffer_[i];
        const int32_t mean = sum / WINDOW_SAMPLES;

        for (uint16_t i = 0; i < WINDOW_SAMPLES; ++i) {
            signal[i] = mean - static_cast<int32_t>(irBuffer_[i]);
        }

        // Match the algorithm's preprocessing and peak separation.
        for (uint16_t i = 0; i < WINDOW_SAMPLES - MA4_SIZE; ++i) {
            const int64_t total = static_cast<int64_t>(signal[i]) + signal[i + 1]
                                + signal[i + 2] + signal[i + 3];
            signal[i] = total / MA4_SIZE;
        }

        int64_t thresholdSum = 0;
        for (uint16_t i = 0; i < WINDOW_SAMPLES; ++i) thresholdSum += signal[i];
        int32_t threshold = thresholdSum / WINDOW_SAMPLES;
        if (threshold < 30) threshold = 30;
        if (threshold > 60) threshold = 60;

        constexpr int32_t MAX_PEAKS = 15;
        int32_t peaks[MAX_PEAKS];
        int32_t peakCount = 0;
        int32_t i = 1;
        while (i < WINDOW_SAMPLES - 1) {
            if (signal[i] > threshold && signal[i] > signal[i - 1]) {
                int32_t width = 1;

                while (i + width < WINDOW_SAMPLES && signal[i] == signal[i + width]) ++width;

                if (i + width < WINDOW_SAMPLES && signal[i] > signal[i + width] && peakCount < MAX_PEAKS) {
                    peaks[peakCount++] = i;
                    i += width + 1;
                } else {
                    i += width;
                }
            } else {
                ++i;
            }
        }

        maxim_remove_close_peaks(peaks, &peakCount, signal, 4);
        if (peakCount < 3) return false;
        int32_t intervals[MAX_PEAKS - 1];
        const int32_t intervalCount = peakCount - 1;
        for (int32_t j = 0; j < intervalCount; ++j) {
            if (peaks[j] < 0 || peaks[j + 1] >= WINDOW_SAMPLES || peaks[j + 1] <= peaks[j]) return false;
            intervals[j] = peaks[j + 1] - peaks[j];
        }

        maxim_sort_ascend(intervals, intervalCount);
        const int32_t median = intervalCount % 2 ? intervals[intervalCount / 2]
          : (intervals[intervalCount / 2 - 1] + intervals[intervalCount / 2]) / 2;
        if (median <= 0) return false;
        for (int32_t j = 0; j < intervalCount; ++j) {
            int64_t deviation = static_cast<int64_t>(intervals[j]) - median;
            if (deviation < 0) deviation = -deviation;
            if (deviation > 2 && deviation * 100
              > static_cast<int64_t>(median) * MAX_INTERVAL_DEVIATION_PERCENT) return false;
        }
        return true;
    }

    bool acceptHeartRate(int32_t heartRate) {
        // separates peaks by at least five samples.
        if (heartRate <= 0 || heartRate > (FreqS * 60) / 5) return false;

        const uint32_t nowMs = millis();
        if (previousHeartRate_ > 0 && uint32_t(nowMs - acceptedAtMs_) < REFERENCE_TIMEOUT_MS) {
            const int32_t change = heartRate - previousHeartRate_;
            if (change > MAX_HEART_RATE_CHANGE || change < -MAX_HEART_RATE_CHANGE) return false;
        }

        previousHeartRate_ = heartRate;
        acceptedAtMs_ = nowMs;
        return true;
    }

    void calculateResult() {
        int32_t spo2 = -999;
        int32_t heartRate = -999;
        int8_t spo2Valid = 0;
        int8_t heartRateValid = 0;

        if (PulseOximeterConfig::TRACE_CALCULATION) {
            uint32_t irMin = irBuffer_[0], irMax = irBuffer_[0];
            for (uint16_t i = 1; i < PulseOximeterConfig::WINDOW_SAMPLES; ++i) {
                if (irBuffer_[i] < irMin) irMin = irBuffer_[i];
                if (irBuffer_[i] > irMax) irMax = irBuffer_[i];
            }
            /*
            Serial.print("calc begin, IR_min=");
            Serial.print(irMin);
            Serial.print(", IR_max=");
            Serial.println(irMax);
            Serial.flush(); // Ensure the marker is sent before entering the algorithm.
            */
        }
        const bool consistentIntervals = hasConsistentIntervals();
        const unsigned long calculationStartUs = micros();
        maxim_heart_rate_and_oxygen_saturation(
            irBuffer_, PulseOximeterConfig::WINDOW_SAMPLES, redBuffer_,
            &spo2, &spo2Valid, &heartRate, &heartRateValid);
        if (PulseOximeterConfig::TRACE_CALCULATION) {
            Serial.print("calc end, elapsed_us=");
            Serial.println(micros() - calculationStartUs);
        }
        result_.timestampUs = micros();
        result_.isSPO2Valid = spo2Valid != 0 && spo2 >= 0 && spo2 <= 100;
        result_.isHeartRateValid = heartRateValid != 0 && consistentIntervals && acceptHeartRate(heartRate);
        result_.spo2 = result_.isSPO2Valid ? spo2 : -1;
        result_.heartRate = result_.isHeartRateValid ? heartRate : -1;
        isResultReady = true;
    }
};

#endif
