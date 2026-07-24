#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ==============================
// Sampling Configurations
// ==============================
#define SAMPLE_COUNT 512
#define WINDOW_STEP_SAMPLES 64   // UI updates every 0.64s using the last 5.12s of data!
#define SAMPLE_RATE 100          // Sampling rate in Hz
#define SAMPLE_PERIOD_US (1000000UL / SAMPLE_RATE) // Timing in microseconds (10000us)

// ==============================
// Tremor Frequency Configurations
// ==============================
#define TREMOR_MIN_FREQ 3.0f     // Hz
#define TREMOR_MAX_FREQ 7.0f     // Hz

// ==============================
// Calibration Configurations
// ==============================
#define CALIBRATION_SAMPLES 300  // 3 seconds at 100Hz

// ==============================
// Sensor Fusion (Complementary Filter)
// ==============================
#define FILTER_ALPHA 0.96f       // Weight for gyroscope integration

// ==============================
// Gravity Constant
// ==============================
#define GRAVITY_MSS 9.80665f     // Standard gravity acceleration (m/s^2)

// ==============================
// Display / TFT Settings
// ==============================
// The Adafruit Feather ESP32-S3 TFT has predefined pins:
// TFT_CS, TFT_DC, TFT_RST, and TFT_I2C_POWER (needs to be high for power)
// We use these native macros directly in our code.

#endif // CONFIG_H
