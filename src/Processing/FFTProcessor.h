#ifndef FFT_PROCESSOR_H
#define FFT_PROCESSOR_H

#include "../Config.h"

namespace Processing {

class FFTProcessor {
public:
    FFTProcessor();
    
    // Analyze a float signal array, returning peak frequency, magnitude, and tremor band energy ratio
    void analyze(const float* inputData, uint16_t size, float sampleRate,
                 float& dominantFreq, float& dominantMag, float& tremorBandRatio);

private:
    float _fftReal[SAMPLE_COUNT];
    float _fftImag[SAMPLE_COUNT];
};

} // namespace Processing

#endif // FFT_PROCESSOR_H
