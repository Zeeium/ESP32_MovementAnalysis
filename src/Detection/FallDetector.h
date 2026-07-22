#ifndef FALL_DETECTOR_H
#define FALL_DETECTOR_H

#include "../Processing/MotionEngine.h"

namespace Detection {

struct FallResult {
    bool fallSuspected;
    float confidence;   // 0.0 to 100.0

    enum FallStage {
        SAFE,
        FREE_FALL,
        IMPACT,
        ORIENTATION_SHIFT,
        INACTIVE,
        TRIGGERED
    } stage;
};

class FallDetector {
public:
    FallDetector();

    // Must be called every sample for real-time fall tracking
    void updateSample(const SensorSample& sample, float restingNoise);

    // Called per-window to evaluate accumulated state
    FallResult evaluate(const MotionFeatures& f);

    // Reset the fall state machine (e.g., after user acknowledges)
    void reset();

private:
    FallResult::FallStage _stage;
    
    // Free-fall tracking
    uint8_t _freeFallCount;       // consecutive low-g samples

    // Impact tracking
    float _impactMagnitude;       // peak g during impact window

    // Orientation tracking
    float _preImpactRoll;
    float _preImpactPitch;

    // Inactivity tracking
    uint32_t _inactivityStartMs;
    float _postImpactVarianceSum;
    uint16_t _postImpactSampleCount;
};

} // namespace Detection

#endif // FALL_DETECTOR_H
