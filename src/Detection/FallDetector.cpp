#include "FallDetector.h"
#include <math.h>

namespace Detection {

FallDetector::FallDetector()
    : _stage(FallResult::SAFE),
      _freeFallCount(0),
      _impactMagnitude(0.0f),
      _preImpactRoll(0.0f),
      _preImpactPitch(0.0f),
      _inactivityStartMs(0),
      _postImpactVarianceSum(0.0f),
      _postImpactSampleCount(0) {}

void FallDetector::updateSample(const SensorSample& sample, float restingNoise) {
    float gForce = sqrt(sample.accelX * sample.accelX +
                        sample.accelY * sample.accelY +
                        sample.accelZ * sample.accelZ) / GRAVITY_MSS;

    switch (_stage) {
        case FallResult::SAFE:
            // Look for free-fall: acceleration magnitude < 0.3g
            if (gForce < 0.3f) {
                _freeFallCount++;
                if (_freeFallCount >= 3) { // 3 consecutive samples (~30ms at 100Hz)
                    _stage = FallResult::FREE_FALL;
                    // Record pre-impact orientation
                    _preImpactRoll = atan2(sample.accelY, sample.accelZ) * 180.0f / M_PI;
                    _preImpactPitch = atan2(-sample.accelX,
                        sqrt(sample.accelY * sample.accelY + sample.accelZ * sample.accelZ)) * 180.0f / M_PI;
                }
            } else {
                _freeFallCount = 0;
            }
            break;

        case FallResult::FREE_FALL:
            // Look for impact spike: > 3.0g
            if (gForce > 3.0f) {
                _impactMagnitude = gForce;
                _stage = FallResult::IMPACT;
            } else if (gForce > 0.7f) {
                // Free-fall ended without impact — false alarm
                _freeFallCount = 0;
                _stage = FallResult::SAFE;
            }
            break;

        case FallResult::IMPACT: {
            // Check orientation change after impact
            float currentRoll = atan2(sample.accelY, sample.accelZ) * 180.0f / M_PI;
            float currentPitch = atan2(-sample.accelX,
                sqrt(sample.accelY * sample.accelY + sample.accelZ * sample.accelZ)) * 180.0f / M_PI;

            float rollChange = fabs(currentRoll - _preImpactRoll);
            float pitchChange = fabs(currentPitch - _preImpactPitch);

            if (rollChange > 45.0f || pitchChange > 45.0f) {
                _stage = FallResult::ORIENTATION_SHIFT;
                _inactivityStartMs = sample.timestamp;
                _postImpactVarianceSum = 0.0f;
                _postImpactSampleCount = 0;
            } else {
                // Impact without orientation change — likely not a fall (e.g., table bump)
                // Give it a few more samples before resetting
                _postImpactSampleCount++;
                if (_postImpactSampleCount > 50) { // ~500ms timeout
                    reset();
                }
            }
            break;
        }

        case FallResult::ORIENTATION_SHIFT: {
            // Monitor for post-impact inactivity (adaptive threshold)
            float deviation = gForce - 1.0f; // deviation from static 1g
            _postImpactVarianceSum += deviation * deviation;
            _postImpactSampleCount++;

            float adaptiveThreshold = restingNoise * 3.0f;
            if (adaptiveThreshold < 0.05f) adaptiveThreshold = 0.05f;

            uint32_t elapsed = sample.timestamp - _inactivityStartMs;

            if (elapsed >= 2000) { // 2 seconds of monitoring
                float postVariance = _postImpactVarianceSum / _postImpactSampleCount;
                if (postVariance < adaptiveThreshold * adaptiveThreshold) {
                    _stage = FallResult::TRIGGERED;
                } else {
                    // Person is moving after impact — probably fine
                    reset();
                }
            }
            break;
        }

        case FallResult::INACTIVE:
        case FallResult::TRIGGERED:
            // Remain in triggered state until externally reset
            break;
    }
}

FallResult FallDetector::evaluate(const MotionFeatures& f) {
    FallResult result;
    result.stage = _stage;

    if (_stage == FallResult::TRIGGERED) {
        result.fallSuspected = true;
        // Confidence based on impact magnitude and orientation change
        float impactScore = (_impactMagnitude > 3.0f) ? fmin(_impactMagnitude / 6.0f, 1.0f) : 0.5f;
        result.confidence = impactScore * 100.0f;
        if (result.confidence > 100.0f) result.confidence = 100.0f;
    } else {
        result.fallSuspected = false;
        result.confidence = 0.0f;
    }

    return result;
}

void FallDetector::reset() {
    _stage = FallResult::SAFE;
    _freeFallCount = 0;
    _impactMagnitude = 0.0f;
    _postImpactVarianceSum = 0.0f;
    _postImpactSampleCount = 0;
}

} // namespace Detection
