#include "MotionEngine.h"
#include <Arduino.h>

namespace Processing {

MotionEngine::MotionEngine()
    : _state(CALIBRATING),
      _calibrated(false),
      _calibrationCount(0),
      _calSumGyroX(0.0f),
      _calSumGyroY(0.0f),
      _calSumGyroZ(0.0f),
      _calSumGyroMagSq(0.0f),
      _gyroBiasX(0.0f),
      _gyroBiasY(0.0f),
      _gyroBiasZ(0.0f),
      _restingNoise(0.0f),
      _lastTimestamp(0) {
    _window.count = 0;
}

void MotionEngine::addSample(SensorSample& sample) {
    // 1. Calibration Phase
    if (!_calibrated) {
        _calSumGyroX += sample.gyroX;
        _calSumGyroY += sample.gyroY;
        _calSumGyroZ += sample.gyroZ;
        
        float magSq = sample.gyroX * sample.gyroX + 
                      sample.gyroY * sample.gyroY + 
                      sample.gyroZ * sample.gyroZ;
        _calSumGyroMagSq += magSq;
        
        _calibrationCount++;
        
        if (_calibrationCount >= CALIBRATION_SAMPLES) {
            _gyroBiasX = _calSumGyroX / CALIBRATION_SAMPLES;
            _gyroBiasY = _calSumGyroY / CALIBRATION_SAMPLES;
            _gyroBiasZ = _calSumGyroZ / CALIBRATION_SAMPLES;
            
            // Calculate resting noise floor (gyro RMS noise)
            _restingNoise = sqrt(_calSumGyroMagSq / CALIBRATION_SAMPLES);
            
            _calibrated = true;
            _state = RESTING;
            _filter.reset(0.0f, 0.0f);
            _window.count = 0;
        }
        return;
    }

    // 2. Subtract Gyro Calibration Biases
    sample.gyroX -= _gyroBiasX;
    sample.gyroY -= _gyroBiasY;
    sample.gyroZ -= _gyroBiasZ;

    // 3. Compute delta time dt (seconds)
    float dt;
    if (_lastTimestamp == 0) {
        dt = 1.0f / SAMPLE_RATE;
    } else {
        dt = (sample.timestamp - _lastTimestamp) / 1000.0f;
    }
    
    // Bounds check dt for robustness
    if (dt <= 0.0f || dt > 1.0f) {
        dt = 1.0f / SAMPLE_RATE;
    }
    _lastTimestamp = sample.timestamp;

    // 4. Update complementary orientation filter
    _filter.update(sample, dt);

    // 5. Accumulate sample in current window
    if (_window.count < SAMPLE_COUNT) {
        if (_window.count == 0) {
            _window.startTime = sample.timestamp;
        }
        _window.samples[_window.count] = sample;
        _window.count++;
        
        if (_window.count == SAMPLE_COUNT) {
            _window.endTime = sample.timestamp;
        }
    }
}

bool MotionEngine::hasWindow() const {
    return _calibrated && (_window.count >= SAMPLE_COUNT);
}

MotionFeatures MotionEngine::extract() {
    MotionFeatures f;
    
    if (_window.count < SAMPLE_COUNT) {
        return f; // Buffer not full yet
    }

    // 1. Calculate Gyroscope Magnitude statistics
    float gyroMagSum = 0.0f;
    float gyroMagSqSum = 0.0f;
    f.gyroPeak = 0.0f;

    for (uint16_t i = 0; i < SAMPLE_COUNT; i++) {
        float gx = _window.samples[i].gyroX;
        float gy = _window.samples[i].gyroY;
        float gz = _window.samples[i].gyroZ;
        
        float mag = sqrt(gx*gx + gy*gy + gz*gz);
        _gyroMag[i] = mag;
        gyroMagSum += mag;
        gyroMagSqSum += mag * mag;
        
        if (mag > f.gyroPeak) {
            f.gyroPeak = mag;
        }
    }

    float meanGyroMag = gyroMagSum / SAMPLE_COUNT;
    f.gyroRMS = sqrt(gyroMagSqSum / SAMPLE_COUNT);
    
    float varSum = 0.0f;
    for (uint16_t i = 0; i < SAMPLE_COUNT; i++) {
        float diff = _gyroMag[i] - meanGyroMag;
        varSum += diff * diff;
    }
    f.gyroVariance = varSum / SAMPLE_COUNT;

    // 2. Compute Periodicity (Normalized Autocorrelation Peak in 3-7 Hz band)
    float meanOut = 0.0f;
    f.normalizedAutocorrelationPeak = calculateAutocorrelationPeak(_gyroMag, SAMPLE_COUNT, meanOut);

    // 3. FFT Analysis on individual gyro axes
    
    // Gyro X FFT
    for (uint16_t i = 0; i < SAMPLE_COUNT; i++) _tempAxis[i] = _window.samples[i].gyroX;
    _fft.analyze(_tempAxis, SAMPLE_COUNT, SAMPLE_RATE, f.dominantFrequency[0], f.dominantMagnitude[0], f.tremorBandEnergy[0]);

    // Gyro Y FFT
    for (uint16_t i = 0; i < SAMPLE_COUNT; i++) _tempAxis[i] = _window.samples[i].gyroY;
    _fft.analyze(_tempAxis, SAMPLE_COUNT, SAMPLE_RATE, f.dominantFrequency[1], f.dominantMagnitude[1], f.tremorBandEnergy[1]);

    // Gyro Z FFT
    for (uint16_t i = 0; i < SAMPLE_COUNT; i++) _tempAxis[i] = _window.samples[i].gyroZ;
    _fft.analyze(_tempAxis, SAMPLE_COUNT, SAMPLE_RATE, f.dominantFrequency[2], f.dominantMagnitude[2], f.tremorBandEnergy[2]);

    // 4. Energy Distribution and Dominant Axis
    float energyX = 0.0f;
    float energyY = 0.0f;
    float energyZ = 0.0f;

    for (uint16_t i = 0; i < SAMPLE_COUNT; i++) {
        energyX += _window.samples[i].gyroX * _window.samples[i].gyroX;
        energyY += _window.samples[i].gyroY * _window.samples[i].gyroY;
        energyZ += _window.samples[i].gyroZ * _window.samples[i].gyroZ;
    }

    float totalEnergy = energyX + energyY + energyZ;
    if (totalEnergy > 0.0f) {
        f.axisXEnergyRatio = energyX / totalEnergy;
        f.axisYEnergyRatio = energyY / totalEnergy;
        f.axisZEnergyRatio = energyZ / totalEnergy;
    } else {
        f.axisXEnergyRatio = 0.0f;
        f.axisYEnergyRatio = 0.0f;
        f.axisZEnergyRatio = 0.0f;
    }

    if (energyX > energyY && energyX > energyZ) {
        f.dominantAxis = 0;
    } else if (energyY > energyZ) {
        f.dominantAxis = 1;
    } else {
        f.dominantAxis = 2;
    }

    // 5. Acceleration statistics (g)
    float accelMagSqSum = 0.0f;
    float accelMagSum = 0.0f;

    for (uint16_t i = 0; i < SAMPLE_COUNT; i++) {
        float ax = _window.samples[i].accelX;
        float ay = _window.samples[i].accelY;
        float az = _window.samples[i].accelZ;
        
        // Acceleration magnitude in g
        float gMag = sqrt(ax*ax + ay*ay + az*az) / GRAVITY_MSS;
        _accelMag[i] = gMag;
        accelMagSum += gMag;
        accelMagSqSum += gMag * gMag;
    }

    f.accelerationRMS = sqrt(accelMagSqSum / SAMPLE_COUNT);
    
    // Linear acceleration (AC part) RMS in g
    float meanAccelMag = accelMagSum / SAMPLE_COUNT;
    float linAccelVarSum = 0.0f;
    for (uint16_t i = 0; i < SAMPLE_COUNT; i++) {
        float diff = _accelMag[i] - meanAccelMag;
        linAccelVarSum += diff * diff;
    }
    f.linearAccelerationRMS = sqrt(linAccelVarSum / SAMPLE_COUNT);

    // 6. Orientation from complementary filter
    f.roll = _filter.getRoll();
    f.pitch = _filter.getPitch();
    f.yawRate = _filter.getYawRate();
    f.relativeYaw = _filter.getRelativeYaw();

    // 7. Window properties
    f.duration = (_window.endTime - _window.startTime) / 1000.0f;

    // Shift window buffer to create a sliding window
    int shiftAmount = SAMPLE_COUNT - WINDOW_STEP_SAMPLES;
    memmove(&_window.samples[0], &_window.samples[WINDOW_STEP_SAMPLES], shiftAmount * sizeof(SensorSample));
    _window.count = shiftAmount;
    _window.startTime = _window.samples[0].timestamp;

    // 8. State Machine Transitions based on window features
    // Experimental Averages: Rest GyroRMS=0.21. Mild Tremor GyroRMS=0.83, Autocorr=0.39
    if (f.gyroRMS < 0.40f) {
        _state = RESTING;
    } else if (f.normalizedAutocorrelationPeak > 0.25f && f.gyroRMS > 0.50f) {
        _state = REPETITIVE_MOVEMENT; // Tremor candidate
    } else {
        _state = NORMAL_MOVEMENT;
    }

    return f;
}

float MotionEngine::calculateAutocorrelationPeak(const float* signal, uint16_t size, float& meanOut) {
    // 1. Calculate mean of signal
    float sum = 0.0f;
    for (uint16_t i = 0; i < size; i++) {
        sum += signal[i];
    }
    float mean = sum / size;
    meanOut = mean;

    // 2. Subtract mean to extract AC component
    for (uint16_t i = 0; i < size; i++) {
        _acBuffer[i] = signal[i] - mean;
    }

    // 3. Compute R(0) (variance * size)
    float r0 = 0.0f;
    for (uint16_t i = 0; i < size; i++) {
        r0 += _acBuffer[i] * _acBuffer[i];
    }

    if (r0 <= 0.0f) {
        return 0.0f;
    }

    // 4. Find peak in lag range [12, 33] (corresponding to ~3-8 Hz at 100Hz)
    // Lag 33 = 3.03 Hz, Lag 12 = 8.33 Hz. This inherently filters out 1-2Hz walking!
    float maxRN = -1.0f;
    for (uint16_t k = 12; k <= 33; k++) {
        float rk = 0.0f;
        for (uint16_t t = 0; t < size - k; t++) {
            rk += _acBuffer[t] * _acBuffer[t + k];
        }
        float rn = rk / r0;
        if (rn > maxRN) {
            maxRN = rn;
        }
    }

    return maxRN;
}

} // namespace Processing
