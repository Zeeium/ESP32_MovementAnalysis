#ifndef MOTION_ENGINE_H
#define MOTION_ENGINE_H

#include "../Config.h"
#include "../Sensors/MPUInterface.h"
#include "Filters.h"
#include "FFTProcessor.h"

// System Motion States
enum MotionState {
    CALIBRATING,
    RESTING,
    NORMAL_MOVEMENT,
    REPETITIVE_MOVEMENT
};

// Time-series window abstraction — consumed by FFT, autocorrelation, and statistics
struct MotionWindow {
    SensorSample samples[SAMPLE_COUNT];
    uint16_t count;
    uint32_t startTime;
    uint32_t endTime;
};

// Processed Motion Features Engine Output
struct MotionFeatures {
    // Gyro stats (rad/s)
    float gyroRMS;
    float gyroPeak;
    float gyroVariance;

    // Spectral metrics (computed per axis)
    float dominantFrequency[3];  // X, Y, Z
    float dominantMagnitude[3];  // X, Y, Z
    float tremorBandEnergy[3];   // X, Y, Z

    // Periodicity & Axis Info
    float normalizedAutocorrelationPeak; // 0.0 to 1.0
    int dominantAxis; // 0 = X, 1 = Y, 2 = Z
    float axisXEnergyRatio;
    float axisYEnergyRatio;
    float axisZEnergyRatio;

    // Acceleration stats (g)
    float accelerationRMS;       // raw magnitude RMS in g
    float linearAccelerationRMS;  // gravity-removed (AC) RMS in g

    // Orientation (deg)
    float roll;
    float pitch;
    float yawRate;      // rad/s
    float relativeYaw;  // deg

    // Window properties
    float duration;     // seconds
};

namespace Processing {

class MotionEngine {
public:
    MotionEngine();
    
    // Process a single raw sample (performs calibration bias correction, updates filter)
    void addSample(SensorSample& sample);
    
    // Returns true when a full window is accumulated and ready for extraction
    bool hasWindow() const;
    
    // Extract motion features from the accumulated window and resets buffer count
    MotionFeatures extract();

    // Getters for status
    MotionState getState() const { return _state; }
    bool isCalibrated() const { return _calibrated; }
    
    // Calibration parameters
    float getGyroBiasX() const { return _gyroBiasX; }
    float getGyroBiasY() const { return _gyroBiasY; }
    float getGyroBiasZ() const { return _gyroBiasZ; }
    float getRestingNoise() const { return _restingNoise; }

private:
    MotionState _state;
    bool _calibrated;
    
    // Calibration accumulators
    uint32_t _calibrationCount;
    float _calSumGyroX;
    float _calSumGyroY;
    float _calSumGyroZ;
    float _calSumGyroMagSq; // For noise floor calculation

    // Calibration offsets
    float _gyroBiasX;
    float _gyroBiasY;
    float _gyroBiasZ;
    float _restingNoise; // Gyro RMS noise floor

    // Ring window buffer
    MotionWindow _window;
    
    // Filter helper
    ComplementaryFilter _filter;
    uint32_t _lastTimestamp; // For dt calculation

    // FFT helper
    FFTProcessor _fft;

    // Internal calculation methods
    float calculateAutocorrelationPeak(const float* signal, uint16_t size, float& meanOut);
};

} // namespace Processing

#endif // MOTION_ENGINE_H
