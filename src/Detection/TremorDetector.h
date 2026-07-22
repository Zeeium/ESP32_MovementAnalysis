#ifndef TREMOR_DETECTOR_H
#define TREMOR_DETECTOR_H

#include "../Processing/MotionEngine.h"

namespace Detection {

struct TremorResult {
    bool detected;
    float confidence;        // 0.0 to 100.0

    enum Severity {
        NONE,
        MILD,
        MODERATE,
        SEVERE
    } severity;

    float dominantFrequency; // Hz
    int dominantAxis;        // 0=X, 1=Y, 2=Z
};

class TremorDetector {
public:
    TremorDetector();

    // Analyze motion features and produce a tremor assessment
    TremorResult analyze(const MotionFeatures& f);

private:
    // Sustained oscillation tracking across windows
    uint8_t _sustainedWindowCount;
    int _lastDominantAxis;

    // Scoring sub-functions (each returns 0.0 to max_points)
    float scoreFrequencyMatch(const MotionFeatures& f);
    float scoreEnergyConcentration(const MotionFeatures& f);
    float scorePeriodicity(const MotionFeatures& f);
    float scoreAxisStability(const MotionFeatures& f);
    float scoreAmplitudeConsistency(const MotionFeatures& f);
};

} // namespace Detection

#endif // TREMOR_DETECTOR_H
