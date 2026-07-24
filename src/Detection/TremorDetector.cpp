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

    // ==========================================
    // ON-DEVICE MACHINE LEARNING ENGINE
    // 2D Cluster Classification (Autocorr, Energy)
    // ==========================================
    
    // We average the 3 tremor centroids to create one massive "Tremor Super-Cluster"
    // REST: [0.3818, 0.1453]
    // TREMOR: [0.5130, 0.8902] (Averaged from Mild, Mod, Severe)
    
    float tremorCentroid[2] = {0.5130f, 0.8902f};

    float inputVec[2] = {
        f.normalizedAutocorrelationPeak,
        bestEnergy
    };

    // Calculate Euclidean distance to TREMOR cluster safely
    float dx = inputVec[0] - tremorCentroid[0];
    float dy = inputVec[1] - tremorCentroid[1];
    float distToTremor = sqrt((dx * dx) + (dy * dy));

    // Map distance to confidence
    // We map a distance of 0.85 (very far) to 0% confidence, and 0.0 to 100%
    result.confidence = (1.0f - (distToTremor / 0.85f)) * 100.0f;
    
    // BOOST CONFIDENCE WITH PHYSICAL INTENSITY (GyroRMS)
    // Because short tremors get mathematically diluted in a 5.12s window, 
    // their 2D distance drops. We compensate by boosting confidence +20% for every 1.0 GyroRMS!
    // Example: If base ML conf is 20%, but GyroRMS is 2.5, new conf = 20 + 50 = 70%!
    result.confidence += (f.gyroRMS * 20.0f);
    
    // Clamp confidence
    if (result.confidence > 100.0f) result.confidence = 100.0f;
    if (result.confidence < 0.0f) result.confidence = 0.0f;

    // A tremor is detected if it has physical intensity AND resembles the tremor cluster
    if (result.confidence >= 50.0f && f.gyroRMS >= 0.80f) { 
        result.detected = true;
        
        // Use physical intensity (GyroRMS) to classify severity
        if (f.gyroRMS >= 4.5f) {
            result.severity = TremorResult::SEVERE;
        } else if (f.gyroRMS >= 3.0f) {
            result.severity = TremorResult::MODERATE;
        } else {
            result.severity = TremorResult::MILD;
        }
    } else {
        result.detected = false;
        result.severity = TremorResult::NONE;
    }

    // Require sustained oscillation (1 window)
    if (result.detected) {
        _sustainedWindowCount++;
    } else {
        _sustainedWindowCount = 0;
    }

    if (_sustainedWindowCount < 1) {
        result.detected = false;
        result.severity = TremorResult::NONE;
    }

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
    // Experimental data showed tremors have 0.72+ energy in tremor band
    float bestRatio = 0.0f;
    for (int a = 0; a < 3; a++) {
        if (f.tremorBandEnergy[a] > bestRatio)
            bestRatio = f.tremorBandEnergy[a];
    }

    // Scale: 0.0 ratio = 0 pts, 0.65+ ratio = full 25 pts
    float score = bestRatio / 0.65f * 25.0f;
    if (score > 25.0f) score = 25.0f;
    return score;
}

float TremorDetector::scorePeriodicity(const MotionFeatures& f) {
    // Max 25 points: normalized autocorrelation peak (0.0 to 1.0)
    // Experimental data showed tremors naturally achieve ~0.37 to ~0.43 Autocorr peak
    float score = f.normalizedAutocorrelationPeak / 0.35f * 25.0f;
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
