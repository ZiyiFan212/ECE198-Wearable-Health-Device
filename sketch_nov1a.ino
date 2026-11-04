#include "Accelerometer.h"
#include "PulseOximeter.h"

Accelerometer accelerometer;
PulseOximeter pulseoximeter;
const bool USE_SMOOTHED_OUTPUT = true;
bool pulseReady = false;

void setup() {
    Serial.begin(9600);
    //accelerometer.begin();
    pulseReady = pulseoximeter.begin();
}

void loop() {
    if (!pulseReady) return;
    /*
    const Accelerometer::Reading data = accelerometer.read();

    // Serial Plotter: X,Y,Z in m/s^2, including gravity.
    Serial.print(USE_SMOOTHED_OUTPUT ? data.x.smoothedAcceleration : data.x.acceleration);
    Serial.print(',');
    Serial.print(USE_SMOOTHED_OUTPUT ? data.y.smoothedAcceleration : data.y.acceleration);
    Serial.print(',');
    Serial.println(USE_SMOOTHED_OUTPUT ? data.z.smoothedAcceleration : data.z.acceleration);
    */
    
    pulseoximeter.update();
    PulseOximeter::Result result;
    if (!pulseoximeter.getResult(result)) return;

    // euh this is just some debug prints
    // CSV: calculation timestamp (us), SpO2 (%), heart rate (BPM), validity flags.
    // Invalid measurements are -1; they must not be treated as measured values.
    Serial.print(result.timestampUs);
    Serial.print(',');
    Serial.print(result.spo2);
    Serial.print(',');
    Serial.print(result.heartRate);
    Serial.print(',');
    Serial.print(result.isSPO2Valid);
    Serial.print(',');
    Serial.println(result.isHeartRateValid);
}
