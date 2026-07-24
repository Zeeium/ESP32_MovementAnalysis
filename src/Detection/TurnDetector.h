#ifndef TURN_DETECTOR_H
#define TURN_DETECTOR_H

#include "../Processing/MotionEngine.h"

namespace Detection {

struct TurnResult {
    bool isTurning;
    bool fogDetected;
    int fogConfidence;
    int fogEventCount;
    float currentTurnSpeed; // deg/s
    uint32_t turnDurationMs;
};

class TurnDetector {
public:
    TurnDetector();

    // Set dynamic threshold based on calibration (deg/s)
    void setTurnThreshold(float threshold);

    // Call this every sample to track real-time turn state
    void updateSample(const SensorSample& sample);

    TurnResult getResult() const;

private:
    float _turnThreshold;
    bool _isTurning;
    uint32_t _turnStartTime;
    uint32_t _lastStepTime;
    
    // For step detection (recovery from turn)
    bool _isPeakDetected;

    // FOG Tracking
    bool _fogFlaggedForCurrentTurn;
    int _fogConfidence;
    int _fogEventCount;
    
    // Current state outputs
    float _currentTurnSpeed;
    uint32_t _turnDurationMs;

    // Helper
    float calculateMagnitude(float x, float y, float z) const;
};

} // namespace Detection

#endif // TURN_DETECTOR_H
