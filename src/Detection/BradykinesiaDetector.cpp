#include "BradykinesiaDetector.h"
#include <math.h>

namespace Detection {

BradykinesiaDetector::BradykinesiaDetector()
    : _testActive(false), _tapCount(0), _aboveThreshold(false),
      _currentPeakMag(0.0f), _currentPeakTime(0), _lastTapTime(0) {}

void BradykinesiaDetector::startTest() {
    _testActive = true;
    _tapCount = 0;
    _aboveThreshold = false;
    _currentPeakMag = 0.0f;
    _currentPeakTime = 0;
    _lastTapTime = 0;
}

void BradykinesiaDetector::updateSample(const SensorSample& sample) {
    if (!_testActive) return;
    if (_tapCount >= MAX_TAPS) return;

    // Use gyro magnitude as the tap indicator
    float gyroMag = sqrt(sample.gyroX * sample.gyroX +
                         sample.gyroY * sample.gyroY +
                         sample.gyroZ * sample.gyroZ);

    if (!_aboveThreshold) {
        // Looking for the start of a tap (signal rises above threshold)
        if (gyroMag > TAP_THRESHOLD) {
            _aboveThreshold = true;
            _currentPeakMag = gyroMag;
            _currentPeakTime = sample.timestamp;
        }
    } else {
        // Currently tracking a tap peak
        if (gyroMag > _currentPeakMag) {
            _currentPeakMag = gyroMag;
            _currentPeakTime = sample.timestamp;
        }

        // Signal dropped back below threshold — tap complete
        if (gyroMag < TAP_THRESHOLD * 0.5f) {
            _aboveThreshold = false;

            // Apply debounce: reject taps that are too close together
            if (_lastTapTime == 0 || (_currentPeakTime - _lastTapTime) >= TAP_DEBOUNCE_MS) {
                _tapTimestamps[_tapCount] = _currentPeakTime;
                _tapAmplitudes[_tapCount] = _currentPeakMag;
                _tapCount++;
                _lastTapTime = _currentPeakTime;
            }

            _currentPeakMag = 0.0f;
        }
    }
}

BradykinesiaResult BradykinesiaDetector::endTest() {
    _testActive = false;

    BradykinesiaResult result;
    result.testActive = false;
    result.testComplete = true;
    result.tapCount = _tapCount;
    result.averageTapInterval = 0.0f;
    result.averageAmplitude = 0.0f;
    result.performanceDropRatio = 1.0f;

    if (_tapCount < 2) {
        result.testComplete = (_tapCount > 0);
        return result;
    }

    // Average tap interval (seconds)
    float totalInterval = (_tapTimestamps[_tapCount - 1] - _tapTimestamps[0]) / 1000.0f;
    result.averageTapInterval = totalInterval / (_tapCount - 1);

    // Average amplitude
    float ampSum = 0.0f;
    for (uint16_t i = 0; i < _tapCount; i++) {
        ampSum += _tapAmplitudes[i];
    }
    result.averageAmplitude = ampSum / _tapCount;

    // Performance drop: compare first 5 taps vs. last 5 taps
    if (_tapCount >= 10) {
        float firstAvg = 0.0f;
        float lastAvg = 0.0f;
        uint16_t compareCount = 5;

        for (uint16_t i = 0; i < compareCount; i++) {
            firstAvg += _tapAmplitudes[i];
            lastAvg += _tapAmplitudes[_tapCount - compareCount + i];
        }
        firstAvg /= compareCount;
        lastAvg /= compareCount;

        if (firstAvg > 0.0f) {
            result.performanceDropRatio = lastAvg / firstAvg;
        }
    } else if (_tapCount >= 4) {
        // Fewer taps: compare first half vs second half
        uint16_t half = _tapCount / 2;
        float firstAvg = 0.0f;
        float lastAvg = 0.0f;

        for (uint16_t i = 0; i < half; i++) {
            firstAvg += _tapAmplitudes[i];
            lastAvg += _tapAmplitudes[_tapCount - half + i];
        }
        firstAvg /= half;
        lastAvg /= half;

        if (firstAvg > 0.0f) {
            result.performanceDropRatio = lastAvg / firstAvg;
        }
    }

    return result;
}

} // namespace Detection
