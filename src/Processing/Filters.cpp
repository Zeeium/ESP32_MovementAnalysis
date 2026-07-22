#include "Filters.h"
#include <math.h>

namespace Processing {

ComplementaryFilter::ComplementaryFilter(float alpha)
    : _alpha(alpha), _roll(0.0f), _pitch(0.0f), _relativeYaw(0.0f), _yawRate(0.0f) {}

void ComplementaryFilter::reset(float initialRoll, float initialPitch) {
    _roll = initialRoll;
    _pitch = initialPitch;
    _relativeYaw = 0.0f;
    _yawRate = 0.0f;
}

void ComplementaryFilter::update(const SensorSample& sample, float dt) {
    // 1. Convert gyroscope rates from rad/s to deg/s
    float gyroX_deg = sample.gyroX * 180.0f / M_PI;
    float gyroY_deg = sample.gyroY * 180.0f / M_PI;
    float gyroZ_deg = sample.gyroZ * 180.0f / M_PI;

    _yawRate = sample.gyroZ; // Store raw yaw rate (rad/s)

    // 2. Calculate Roll and Pitch tilt angles from accelerometer gravity vector
    // Pitch is rotation around the Y axis
    float accelPitch = atan2(-sample.accelX, sqrt(sample.accelY * sample.accelY + sample.accelZ * sample.accelZ)) * 180.0f / M_PI;
    // Roll is rotation around the X axis
    float accelRoll = atan2(sample.accelY, sample.accelZ) * 180.0f / M_PI;

    // 3. Apply complementary filter logic
    // High-pass filter on gyro integration + Low-pass filter on accel tilt
    _pitch = _alpha * (_pitch + gyroY_deg * dt) + (1.0f - _alpha) * accelPitch;
    _roll = _alpha * (_roll + gyroX_deg * dt) + (1.0f - _alpha) * accelRoll;

    // Yaw cannot be corrected without a magnetometer, so it is relative integration only
    _relativeYaw += gyroZ_deg * dt;
}

} // namespace Processing
