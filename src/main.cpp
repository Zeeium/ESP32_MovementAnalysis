#include <Arduino.h>
#include "Config.h"
#include "Sensors/MPUInterface.h"
#include "Processing/MotionEngine.h"
#include "Detection/TremorDetector.h"
#include "Detection/FallDetector.h"
#include "Detection/BradykinesiaDetector.h"
#include "UI/DisplayManager.h"
#include "Data/Logger.h"

// ==============================
// System Components
// ==============================
Sensors::MPUInterface imu;
Processing::MotionEngine motionEngine;
Detection::TremorDetector tremorDetector;
Detection::FallDetector fallDetector;
Detection::BradykinesiaDetector bradykinesiaDetector;
UI::DisplayManager display;
Data::Logger telemetryLogger;

// ==============================
// Timing
// ==============================
unsigned long nextSampleUs = 0;
bool sensorReady = false;

// Latest results (for display refresh)
MotionFeatures latestFeatures;
Detection::TremorResult latestTremor;
Detection::FallResult latestFall;

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("==================================================");
    Serial.println("NeuroMotion Movement Analysis Platform");
    Serial.println("==================================================");

    // Initialize display first so we can show calibration progress
    display.begin();
    display.showCalibration(0, CALIBRATION_SAMPLES);

    if (!imu.begin()) {
        Serial.println("[-] CRITICAL ERROR: MPU6050 sensor not found!");
        while (true) {
            delay(100);
        }
    }

    sensorReady = true;
    Serial.println("[+] MPU6050 initialized successfully.");
    Serial.println("[*] Calibrating... keep hand still for 3 seconds.");

    // Initialize latest results to safe defaults
    latestTremor.detected = false;
    latestTremor.confidence = 0.0f;
    latestTremor.severity = Detection::TremorResult::NONE;
    latestTremor.dominantFrequency = 0.0f;
    latestTremor.dominantAxis = 0;

    latestFall.fallSuspected = false;
    latestFall.confidence = 0.0f;
    latestFall.stage = Detection::FallResult::SAFE;

    nextSampleUs = micros();
}

void loop() {
    if (!sensorReady) return;

    unsigned long nowUs = micros();
    if (nowUs < nextSampleUs) return;

    // Schedule next sample
    nextSampleUs += SAMPLE_PERIOD_US;
    if (micros() > nextSampleUs + SAMPLE_PERIOD_US) {
        nextSampleUs = micros() + SAMPLE_PERIOD_US;
    }

    SensorSample sample;
    if (!imu.readSample(sample)) {
        return;
    }

    // Feed sample to motion engine (handles calibration internally)
    motionEngine.addSample(sample);

    // During calibration, update display progress
    if (!motionEngine.isCalibrated()) {
        // Show calibration screen — estimate progress from internal state
        static uint32_t calDisplayCounter = 0;
        calDisplayCounter++;
        if (calDisplayCounter % 10 == 0) { // Update display every 10 samples
            display.showCalibration(calDisplayCounter, CALIBRATION_SAMPLES);
        }
        return;
    }

    // Announce calibration completion once
    static bool calibrationAnnounced = false;
    if (!calibrationAnnounced) {
        Serial.println("[+] Calibration complete!");
        Serial.print("[*] Gyro offsets: ");
        Serial.print(motionEngine.getGyroBiasX(), 4); Serial.print(", ");
        Serial.print(motionEngine.getGyroBiasY(), 4); Serial.print(", ");
        Serial.println(motionEngine.getGyroBiasZ(), 4);
        Serial.print("[*] Rest Gyro Noise Floor (RMS): ");
        Serial.print(motionEngine.getRestingNoise(), 4);
        Serial.println(" rad/s");
        calibrationAnnounced = true;
    }

    // Feed sample to fall detector (needs per-sample tracking)
    fallDetector.updateSample(sample, motionEngine.getRestingNoise());

    // Feed sample to bradykinesia detector if test is active
    if (bradykinesiaDetector.isTestActive()) {
        bradykinesiaDetector.updateSample(sample);
    }

    // When a full window is ready, extract features and run analysis
    if (motionEngine.hasWindow()) {
        latestFeatures = motionEngine.extract();

        // Run tremor analysis
        latestTremor = tremorDetector.analyze(latestFeatures);

        // Run fall evaluation
        latestFall = fallDetector.evaluate(latestFeatures);

        // Log telemetry
        telemetryLogger.logFeatures(latestFeatures, motionEngine.getState());

        // Update display
        if (latestFall.fallSuspected) {
            display.showFallWarning(latestFall);
        } else {
            display.showMonitoring(latestFeatures, latestTremor, latestFall, motionEngine.getState());
        }
    }
}
