#include "TremorDetector.h"
#include <math.h>

namespace Detection {

TremorDetector::TremorDetector()
    : _sustainedWindowCount(0), _lastDominantAxis(-1) {}

TremorResult TremorDetector::analyze(const MotionFeatures& f) {
    TremorResult result;
    result.detected = false;
    result.confidence = 0.0f;
    result.severity = TremorResult::NONE;
    result.dominantFrequency = 0.0f;
    result.dominantAxis = f.dominantAxis;

    // Find the axis with the highest tremor band energy
    int bestAxis = 0;
    float bestEnergy = f.tremorBandEnergy[0];
    for (int a = 1; a < 3; a++) {
        if (f.tremorBandEnergy[a] > bestEnergy) {
            bestEnergy = f.tremorBandEnergy[a];
            bestAxis = a;
        }
    }
    result.dominantAxis = bestAxis;
    result.dominantFrequency = f.dominantFrequency[bestAxis];

    // Calculate confidence sub-scores
    float freqScore   = scoreFrequencyMatch(f);       // max 30
    float energyScore = scoreEnergyConcentration(f);   // max 25
    float periodScore = scorePeriodicity(f);            // max 25
    float axisScore   = scoreAxisStability(f);          // max 10
    float ampScore    = scoreAmplitudeConsistency(f);   // max 10

    result.confidence = freqScore + energyScore + periodScore + axisScore + ampScore;

    // Clamp to [0, 100]
    if (result.confidence > 100.0f) result.confidence = 100.0f;
    if (result.confidence < 0.0f) result.confidence = 0.0f;

    // Track sustained oscillation across consecutive windows
    // Require at least ~3 seconds of sustained periodic activity
    if (result.confidence > 40.0f) {
        _sustainedWindowCount++;
    } else {
        _sustainedWindowCount = 0;
    }

    // At 256 samples / 100 Hz = 2.56s per window, ~2 windows = ~5 seconds
    bool sustained = (_sustainedWindowCount >= 2);

    // Classify severity based on confidence bands (only if sustained)
    if (sustained && result.confidence >= 40.0f) {
        result.detected = true;

        if (result.confidence >= 75.0f) {
            result.severity = TremorResult::SEVERE;
        } else if (result.confidence >= 55.0f) {
            result.severity = TremorResult::MODERATE;
        } else {
            result.severity = TremorResult::MILD;
        }
    } else {
        result.detected = false;
        result.severity = TremorResult::NONE;
    }

    // Track dominant axis for stability scoring
    _lastDominantAxis = bestAxis;

    return result;
}

// ==============================
// Scoring Sub-Functions
// ==============================

float TremorDetector::scoreFrequencyMatch(const MotionFeatures& f) {
    // Max 30 points: dominant frequency falls within 3-7 Hz tremor band
    // Find highest-energy axis frequency
    int bestAxis = 0;
    float bestEnergy = f.tremorBandEnergy[0];
    for (int a = 1; a < 3; a++) {
        if (f.tremorBandEnergy[a] > bestEnergy) {
            bestEnergy = f.tremorBandEnergy[a];
            bestAxis = a;
        }
    }

    float freq = f.dominantFrequency[bestAxis];
    float center = (TREMOR_MIN_FREQ + TREMOR_MAX_FREQ) / 2.0f; // 5.0 Hz
    float halfBand = (TREMOR_MAX_FREQ - TREMOR_MIN_FREQ) / 2.0f; // 2.0 Hz

    if (freq >= TREMOR_MIN_FREQ && freq <= TREMOR_MAX_FREQ) {
        // Perfect if near center, linear falloff toward edges
        float distFromCenter = fabs(freq - center);
        float normalizedDist = distFromCenter / halfBand; // 0.0 at center, 1.0 at edge
        return 30.0f * (1.0f - 0.3f * normalizedDist);   // 30 at center, 21 at edges
    } else {
        // Partial credit for nearby frequencies
        float distOutside = 0.0f;
        if (freq < TREMOR_MIN_FREQ) distOutside = TREMOR_MIN_FREQ - freq;
        else distOutside = freq - TREMOR_MAX_FREQ;

        float falloff = 1.0f - (distOutside / 3.0f); // 0 credit at 3Hz outside band
        if (falloff < 0.0f) falloff = 0.0f;
        return 10.0f * falloff; // max 10 for near-miss
    }
}

float TremorDetector::scoreEnergyConcentration(const MotionFeatures& f) {
    // Max 25 points: how much energy is concentrated in the tremor band
    float bestRatio = 0.0f;
    for (int a = 0; a < 3; a++) {
        if (f.tremorBandEnergy[a] > bestRatio)
            bestRatio = f.tremorBandEnergy[a];
    }

    // Scale: 0.0 ratio = 0 pts, 0.5+ ratio = full 25 pts
    float score = bestRatio / 0.5f * 25.0f;
    if (score > 25.0f) score = 25.0f;
    return score;
}

float TremorDetector::scorePeriodicity(const MotionFeatures& f) {
    // Max 25 points: normalized autocorrelation peak (0.0 to 1.0)
    // Score scales linearly; a peak > 0.6 gets full marks
    float score = f.normalizedAutocorrelationPeak / 0.6f * 25.0f;
    if (score > 25.0f) score = 25.0f;
    if (score < 0.0f) score = 0.0f;
    return score;
}

float TremorDetector::scoreAxisStability(const MotionFeatures& f) {
    // Max 10 points: same dominant axis as previous window
    if (_lastDominantAxis < 0) return 5.0f; // First window, give half credit

    int bestAxis = 0;
    float bestEnergy = f.tremorBandEnergy[0];
    for (int a = 1; a < 3; a++) {
        if (f.tremorBandEnergy[a] > bestEnergy) {
            bestEnergy = f.tremorBandEnergy[a];
            bestAxis = a;
        }
    }

    return (bestAxis == _lastDominantAxis) ? 10.0f : 2.0f;
}

float TremorDetector::scoreAmplitudeConsistency(const MotionFeatures& f) {
    // Max 10 points: low gyro variance relative to mean suggests consistent oscillation
    // High variance = chaotic movement, low variance = regular tremor
    if (f.gyroRMS < 0.01f) return 0.0f; // No meaningful signal

    float coeffOfVariation = sqrt(f.gyroVariance) / f.gyroRMS;
    // CV < 0.5 = very consistent = full marks; CV > 1.5 = random = 0
    float score = (1.5f - coeffOfVariation) / 1.0f * 10.0f;
    if (score > 10.0f) score = 10.0f;
    if (score < 0.0f) score = 0.0f;
    return score;
}

} // namespace Detection
