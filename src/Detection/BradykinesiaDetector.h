#ifndef BRADYKINESIA_DETECTOR_H
#define BRADYKINESIA_DETECTOR_H

#include "../Processing/MotionEngine.h"

namespace Detection {

struct BradykinesiaResult {
    bool testActive;
    bool testComplete;
    
    uint16_t tapCount;
    float averageTapInterval;    // seconds
    float averageAmplitude;      // rad/s (peak gyro per tap)
    float performanceDropRatio;  // ratio of last-5 vs first-5 amplitude (< 1.0 = decline)
};

class BradykinesiaDetector {
public:
    BradykinesiaDetector();

    // Start a new tap test session
    void startTest();

    // Feed raw samples during the test; detects tap peaks
    void updateSample(const SensorSample& sample);

    // End the test and compute results
    BradykinesiaResult endTest();

    // Check if test is currently running
    bool isTestActive() const { return _testActive; }

    // Get current tap count (for live display)
    uint16_t getCurrentTapCount() const { return _tapCount; }

private:
    bool _testActive;

    // Tap detection
    static const uint16_t MAX_TAPS = 64;
    uint32_t _tapTimestamps[MAX_TAPS];
    float _tapAmplitudes[MAX_TAPS];
    uint16_t _tapCount;

    // Peak detection state
    bool _aboveThreshold;
    float _currentPeakMag;
    uint32_t _currentPeakTime;
    uint32_t _lastTapTime;

    // Thresholds
    static constexpr float TAP_THRESHOLD = 0.5f;     // rad/s to count as tap start
    static constexpr uint32_t TAP_DEBOUNCE_MS = 150;  // minimum time between taps
};

} // namespace Detection

#endif // BRADYKINESIA_DETECTOR_H
