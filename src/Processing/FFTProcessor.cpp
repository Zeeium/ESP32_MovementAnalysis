#include "FFTProcessor.h"
#include <arduinoFFT.h>

namespace Processing {

FFTProcessor::FFTProcessor() {}

void FFTProcessor::analyze(const float* inputData, uint16_t size, float sampleRate,
                           float& dominantFreq, float& dominantMag, float& tremorBandRatio) {
    // 1. Copy input data to workspace buffers, clearing imaginary parts
    for (uint16_t i = 0; i < size && i < SAMPLE_COUNT; i++) {
        _fftReal[i] = inputData[i];
        _fftImag[i] = 0.0f;
    }

    ArduinoFFT<float> fft(_fftReal, _fftImag, size, sampleRate);

    // 2. Perform DC removal (AC coupling)
    fft.dcRemoval();

    // 3. Apply Hamming windowing
    fft.windowing(FFTWindow::Hamming, FFTDirection::Forward);

    // 4. Compute FFT in-place
    fft.compute(FFTDirection::Forward);

    // 5. Convert complex values to magnitude
    fft.complexToMagnitude();

    // 6. Find major peak frequency (returns interpolated frequency)
    dominantFreq = fft.majorPeak();

    // 7. Find dominant peak magnitude (ignoring bin 0 / DC)
    float maxMag = 0.0f;
    for (uint16_t i = 1; i <= size / 2; i++) {
        if (_fftReal[i] > maxMag) {
            maxMag = _fftReal[i];
        }
    }
    dominantMag = maxMag;

    // 8. Calculate energy ratios
    float totalEnergy = 0.0f;
    float tremorEnergy = 0.0f;
    
    // We only traverse the positive frequency bins (1 to size/2)
    for (uint16_t i = 1; i <= size / 2; i++) {
        float freq = (i * sampleRate) / size;
        float binEnergy = _fftReal[i] * _fftReal[i];
        
        totalEnergy += binEnergy;
        if (freq >= TREMOR_MIN_FREQ && freq <= TREMOR_MAX_FREQ) {
            tremorEnergy += binEnergy;
        }
    }

    if (totalEnergy > 0.0f) {
        tremorBandRatio = tremorEnergy / totalEnergy;
    } else {
        tremorBandRatio = 0.0f;
    }
}

} // namespace Processing
