#include "Logger.h"
#include <Arduino.h>

namespace Data {

static bool headerPrinted = false;

Logger::Logger() {}

void Logger::logFeatures(const MotionFeatures& f, MotionState state) {
    if (!headerPrinted) {
        Serial.println("==========================================================================================================");
        Serial.println("TELEMETRY HEADER: Timestamp(ms),State,GyroRMS(rad/s),GyroPeak,GyroVar,AutocorrPeak,DomAxis,Roll(deg),Pitch,YawRate(rad/s),RelYaw,AccelRMS(g),LinAccelRMS(g)");
        Serial.println("==========================================================================================================");
        headerPrinted = true;
    }

    // Convert state to human readable string
    const char* stateStr;
    switch (state) {
        case CALIBRATING:          stateStr = "CALIBRATING"; break;
        case RESTING:              stateStr = "RESTING"; break;
        case NORMAL_MOVEMENT:      stateStr = "NORMAL_MOVEMENT"; break;
        case REPETITIVE_MOVEMENT:  stateStr = "REPETITIVE_MOVEMENT"; break;
        case IMPACT_EVENT:         stateStr = "IMPACT_EVENT"; break;
        default:                   stateStr = "UNKNOWN"; break;
    }

    // Print CSV telemetry row
    Serial.print(millis()); Serial.print(",");
    Serial.print(stateStr); Serial.print(",");
    Serial.print(f.gyroRMS, 4); Serial.print(",");
    Serial.print(f.gyroPeak, 4); Serial.print(",");
    Serial.print(f.gyroVariance, 4); Serial.print(",");
    Serial.print(f.normalizedAutocorrelationPeak, 4); Serial.print(",");
    Serial.print(f.dominantAxis); Serial.print(",");
    Serial.print(f.roll, 2); Serial.print(",");
    Serial.print(f.pitch, 2); Serial.print(",");
    Serial.print(f.yawRate, 4); Serial.print(",");
    Serial.print(f.relativeYaw, 2); Serial.print(",");
    Serial.print(f.accelerationRMS, 4); Serial.print(",");
    Serial.println(f.linearAccelerationRMS, 4);
}

} // namespace Data
