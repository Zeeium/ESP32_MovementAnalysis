#include "TurnDetector.h"
#include "../Config.h"
#include <math.h>

namespace Detection {

TurnDetector::TurnDetector()
    : _turnThreshold(55.0f),
      _isTurning(false),
      _turnStartTime(0),
      _lastStepTime(0),
      _isPeakDetected(false),
      _fogFlaggedForCurrentTurn(false),
      _fogConfidence(0),
      _fogEventCount(0),
      _currentTurnSpeed(0.0f),
      _turnDurationMs(0) {}

void TurnDetector::setTurnThreshold(float threshold) {
    _turnThreshold = threshold;
}

float TurnDetector::calculateMagnitude(float x, float y, float z) const {
    return sqrt(x*x + y*y + z*z);
}

void TurnDetector::updateSample(const SensorSample& sample) {
    // 1. Calculate step recovery metric (linear acceleration magnitude)
    // The user's code expects roughly ~9.8 for 1g, so we assume m/s^2 scaling.
    float accelMag = calculateMagnitude(sample.accelX, sample.accelY, sample.accelZ);
    
    // User's step thresholds
    const float HIGH_STEP_THRESH = 10.7f;
    const float LOW_STEP_THRESH = 9.3f;

    if (accelMag > HIGH_STEP_THRESH && !_isPeakDetected) {
        _isPeakDetected = true;
        _lastStepTime = sample.timestamp;
    } else if (accelMag < LOW_STEP_THRESH) {
        _isPeakDetected = false;
    }

    // 2. Calculate turn speed (gyro magnitude)
    // sample.gyro in MotionEngine is kept in rad/s, convert to deg/s
    float gx = sample.gyroX * 57.2958f;
    float gy = sample.gyroY * 57.2958f;
    float gz = sample.gyroZ * 57.2958f;
    
    _currentTurnSpeed = calculateMagnitude(gx, gy, gz);

    // 3. State Machine for Turning & FOG
    if (!_isTurning) {
        if (_currentTurnSpeed > _turnThreshold) {
            _isTurning = true;
            _turnStartTime = sample.timestamp;
            _fogFlaggedForCurrentTurn = false;
            _fogConfidence = 0;
            _turnDurationMs = 0;
            // Also reset step timer to now, so we don't instantly flag FOG if they've been standing still before the turn
            _lastStepTime = sample.timestamp; 
        }
    } else {
        _turnDurationMs = sample.timestamp - _turnStartTime;
        
        // If we took a step recently, or the rotation speed dropped heavily, we consider the turn finished/recovered
        if ((sample.timestamp - _lastStepTime) < 1500) {
            _isTurning = false;
            _turnDurationMs = 0;
            _fogFlaggedForCurrentTurn = false;
            _fogConfidence = 0;
        } else if (_turnDurationMs > 3000) { // 3 seconds turning without a clear step = FOG
            if (!_fogFlaggedForCurrentTurn) {
                _fogFlaggedForCurrentTurn = true;
                _fogEventCount++;
            }
            
            // Map duration from 3s to 13s into a confidence scale of 10 to 90
            float conf = 10.0f + 80.0f * (static_cast<float>(_turnDurationMs) - 3000.0f) / 10000.0f;
            if (conf > 95.0f) conf = 95.0f; // Cap at 95% to avoid claiming 100% certainty
            _fogConfidence = static_cast<int>(conf);
        }
    }
}

TurnResult TurnDetector::getResult() const {
    TurnResult res;
    res.isTurning = _isTurning;
    res.fogDetected = _fogFlaggedForCurrentTurn; 
    res.fogConfidence = _fogConfidence;
    res.fogEventCount = _fogEventCount;
    res.currentTurnSpeed = _currentTurnSpeed;
    res.turnDurationMs = _turnDurationMs;
    return res;
}

} // namespace Detection
