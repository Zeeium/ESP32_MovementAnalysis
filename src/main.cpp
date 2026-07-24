#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include "Config.h"
#include "Sensors/MPUInterface.h"
#include "Processing/MotionEngine.h"
#include "Detection/TremorDetector.h"
#include "Detection/TurnDetector.h"
#include "UI/DisplayManager.h"
#include "Data/Logger.h"

// ==============================
// System Components
// ==============================
Sensors::MPUInterface imu;
Processing::MotionEngine motionEngine;
Detection::TremorDetector tremorDetector;
Detection::TurnDetector turnDetector;
UI::DisplayManager display;
Data::Logger telemetryLogger;

#include <AdafruitIO_WiFi.h>

// ==============================
// Adafruit IO Setup
// ==============================
#define IO_USERNAME  "Aerodynamic"
#define IO_KEY       "aio_ybAZ04XdnPADgTlQS9UzSxfS6Q3G"
#define WIFI_SSID    "DarrensPhone"
#define WIFI_PASS    "ASDF1234"

AdafruitIO_WiFi io(IO_USERNAME, IO_KEY, WIFI_SSID, WIFI_PASS);
AdafruitIO_Feed *tremorLog = io.feed("tremor-log");
AdafruitIO_Feed *fogLog = io.feed("fog-log");

// State Tracking for Rate Limiting
bool _wasTremor = false;
bool _wasFOG = false;

// ==============================
// Timing & State
// ==============================
#include <WiFiClientSecure.h>

const char* DISCORD_WEBHOOK_URL = "https://discord.com/api/webhooks/1529183469106630728/pmZB1b9pc034vfcY9z6yQhSg4QbG6wlc4JSY7xBUBwmgFiO5P9GOnH23-EqzQJ5Q7DRY";
unsigned long lastImpactTime = 0;

void sendDiscordWebhook(float gForce) {
    if (WiFi.status() == WL_CONNECTED) {
        WiFiClientSecure client;
        client.setInsecure();
        HTTPClient http;
        http.begin(client, DISCORD_WEBHOOK_URL);
        http.addHeader("Content-Type", "application/json");
        
        char payload[256];
        snprintf(payload, sizeof(payload), "{\"content\":\"🚨 **HIGH IMPACT DETECTED!** 🚨\\nPotential fall detected with a force of **%.2f G's**!\"}", gForce);
        
        int httpResponseCode = http.POST(payload);
        if (httpResponseCode > 0) {
            Serial.printf("[+] Discord webhook sent (Response: %d)\n", httpResponseCode);
        } else {
            Serial.printf("[-] Error sending Discord webhook: %s\n", http.errorToString(httpResponseCode).c_str());
        }
        http.end();
    }
}

unsigned long nextSampleUs = 0;
bool sensorReady = false;

// Latest results (for display refresh)
MotionFeatures latestFeatures;
Detection::TremorResult latestTremor;
Detection::TurnResult latestTurn;

// ==============================
// Turn Calibration Variables
// ==============================
enum TurnCalibPhase {
    TCAL_WAIT,
    TCAL_COUNTDOWN,
    TCAL_SPIN,
    TCAL_DONE
};
TurnCalibPhase turnCalibPhase = TCAL_WAIT;
int turnCalibRep = 0;
float turnCalibMaxSpeeds[3] = {0.0f, 0.0f, 0.0f};
unsigned long turnCalibTimer = 0;
int turnCountdownSecs = 0;
float currentRepMaxSpeed = 0.0f;

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("==================================================");
    Serial.println("NeuroMotion Movement Analysis Platform");
    Serial.println("==================================================");

    // Initialize display first so we can show boot screen
    display.begin();

    if (!imu.begin()) {
        Serial.println("[-] CRITICAL ERROR: MPU6050 sensor not found!");
        while (true) {
            delay(100);
        }
    }

    sensorReady = true;
    Serial.println("[+] MPU6050 initialized successfully.");
    
    // Connect to Adafruit IO
    Serial.print("Connecting to Adafruit IO");
    io.connect();

    // Wait for connection
    while(io.status() < AIO_CONNECTED) {
        Serial.print(".");
        delay(500);
    }
    Serial.println();
    Serial.println(io.statusText());
    
    Serial.println("[*] Calibrating... keep hand still for 5 seconds.");

    // Initialize latest results to safe defaults
    latestTremor.detected = false;
    latestTremor.confidence = 0.0f;
    latestTremor.severity = Detection::TremorResult::NONE;
    latestTremor.dominantFrequency = 0.0f;
    latestTremor.dominantAxis = 0;

    latestTurn.isTurning = false;
    latestTurn.fogDetected = false;
    latestTurn.fogEventCount = 0;
    latestTurn.currentTurnSpeed = 0.0f;
    latestTurn.turnDurationMs = 0;

    nextSampleUs = micros();
}

// ==============================
// EXPERIMENT MODE
// ==============================
#define EXPERIMENT_MODE 0

#if EXPERIMENT_MODE
enum ExpPhase {
    EXP_WAIT_START,
    EXP_COUNTDOWN,
    EXP_RECORDING,
    EXP_FINISHED
};

ExpPhase expPhase = EXP_WAIT_START;
int currentCategory = 0; // 0=REST, 1=MILD, 2=MODERATE, 3=SEVERE
int currentRep = 0;      // 0, 1, 2
unsigned long expTimer = 0;
int countdownSecs = 0;

struct ExpStats {
    float sumGyroRms = 0;
    float sumAutocorr = 0;
    float sumEnergy = 0;
    int count = 0;
} stats[4];

const char* catNames[] = {"REST", "MILD TREMOR", "MODERATE TREMOR", "SEVERE TREMOR"};

void runExperimentStateMachine(bool windowReady) {
    if (expPhase == EXP_WAIT_START) {
        static unsigned long lastPrompt = 0;
        if (millis() - lastPrompt > 3000) {
            Serial.println("\n=================================");
            Serial.println("INTERACTIVE CALIBRATION EXPERIMENT");
            Serial.println("Type 'START' and press enter when ready!");
            Serial.println("=================================");
            lastPrompt = millis();
        }
        if (Serial.available()) {
            String in = Serial.readStringUntil('\n');
            in.trim();
            if (in.equalsIgnoreCase("START")) {
                currentCategory = 0;
                currentRep = 0;
                expPhase = EXP_COUNTDOWN;
                countdownSecs = 3;
                expTimer = millis();
                Serial.printf("\n--- PREPARE FOR %s ---\n", catNames[currentCategory]);
            }
        }
    } 
    else if (expPhase == EXP_COUNTDOWN) {
        if (millis() - expTimer >= 1000) {
            expTimer = millis();
            if (countdownSecs > 0) {
                Serial.printf("Starting in %d...\n", countdownSecs);
                countdownSecs--;
            } else {
                Serial.println(">>> GO! <<< Keep doing it for 5 seconds!");
                expPhase = EXP_RECORDING;
                // We don't rely on millis() for recording duration, 
                // we rely on motionEngine.hasWindow() which takes exactly 5.12s
            }
        }
        // Purge any windows generated during countdown so we get a fresh one
        if (windowReady) {
            motionEngine.extract();
        }
    } 
    else if (expPhase == EXP_RECORDING) {
        if (windowReady) {
            MotionFeatures f = motionEngine.extract();
            Serial.printf("Recorded! GyroRMS: %.4f | Autocorr: %.4f | Energy: %.4f\n", 
                          f.gyroRMS, f.normalizedAutocorrelationPeak, f.tremorBandEnergy[f.dominantAxis]);
            
            stats[currentCategory].sumGyroRms += f.gyroRMS;
            stats[currentCategory].sumAutocorr += f.normalizedAutocorrelationPeak;
            stats[currentCategory].sumEnergy += f.tremorBandEnergy[f.dominantAxis];
            stats[currentCategory].count++;

            currentRep++;
            if (currentRep >= 3) {
                currentCategory++;
                currentRep = 0;
                if (currentCategory >= 4) {
                    expPhase = EXP_FINISHED;
                    Serial.println("\n=================================");
                    Serial.println("       EXPERIMENT COMPLETE       ");
                    Serial.println("=================================");
                    Serial.println("Category\tAvg GyroRMS\tAvg Autocorr\tAvg Energy");
                    for (int i=0; i<4; i++) {
                        float avgGyro = stats[i].sumGyroRms / 3.0f;
                        float avgAuto = stats[i].sumAutocorr / 3.0f;
                        float avgEner = stats[i].sumEnergy / 3.0f;
                        Serial.printf("%s\t%.4f\t\t%.4f\t\t%.4f\n", catNames[i], avgGyro, avgAuto, avgEner);
                    }
                    Serial.println("=================================");
                    Serial.println("Copy and paste these results to your AI assistant!");
                } else {
                    expPhase = EXP_COUNTDOWN;
                    countdownSecs = 3;
                    expTimer = millis();
                    Serial.printf("\n--- PREPARE FOR %s ---\n", catNames[currentCategory]);
                }
            } else {
                // Next rep
                expPhase = EXP_COUNTDOWN;
                countdownSecs = 3;
                expTimer = millis();
                Serial.printf("\n--- NEXT REP for %s ---\n", catNames[currentCategory]);
            }
        }
    }
}
#endif

void loop() {
    if (!sensorReady) return;

    unsigned long nowUs = micros();
    if (nowUs < nextSampleUs) return;

    // Schedule next sample
    nextSampleUs += SAMPLE_PERIOD_US;
    if (micros() > nextSampleUs + SAMPLE_PERIOD_US) {
        nextSampleUs = micros() + SAMPLE_PERIOD_US; // prevent lag spiral
    }

    SensorSample sample;
    if (!imu.readSample(sample)) {
        return;
    }

    // --- IMPACT / FALL DETECTION ---
    // Calculate raw acceleration magnitude in G's (Assuming sample is in m/s^2)
    float accelMagG = sqrt(sample.accelX * sample.accelX + 
                           sample.accelY * sample.accelY + 
                           sample.accelZ * sample.accelZ) / 9.81f;
    
    // Check if impact exceeds 4.0 G's (with a 5 second cooldown to prevent spam)
    if (accelMagG > 4.0f) {
        if (millis() - lastImpactTime > 5000) {
            lastImpactTime = millis();
            Serial.printf("[!] HIGH IMPACT DETECTED: %.2f Gs\n", accelMagG);
            sendDiscordWebhook(accelMagG);
        }
    }
    // -------------------------------

    // Feed sample to motion engine (handles calibration internally)
    motionEngine.addSample(sample);

#if EXPERIMENT_MODE
    if (motionEngine.isCalibrated()) {
        runExperimentStateMachine(motionEngine.hasWindow());
    }
    return; // Bypass the rest of the loop during experiment mode
#endif

    // Keep Adafruit IO connection alive
    io.run();

    // Determine UI State
    UI::UIState currentUIState = UI::UIState::BOOT;
    if (!motionEngine.isCalibrated()) {
        currentUIState = UI::UIState::BOOT;
    } else if (millis() - lastImpactTime < 5000 && lastImpactTime != 0) {
        currentUIState = UI::UIState::FALL;
    } else if (latestTurn.fogDetected) {
        currentUIState = UI::UIState::FREEZING;
    } else if (latestTremor.detected) {
        currentUIState = UI::UIState::TREMOR;
    } else {
        currentUIState = UI::UIState::NORMAL;
    }

#define SCREEN_DEBUG_MODE 0

    // Update animated UI (non-blocking) or debug view
#if SCREEN_DEBUG_MODE
    display.updateDebug(latestFeatures, latestTremor);
#else
    display.update(currentUIState);
#endif

    // During calibration, just return after adding sample
    if (!motionEngine.isCalibrated()) {
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
        
        // Dynamic turn threshold can be set here if needed, currently static 55 deg/s in TurnDetector constructor
        
        calibrationAnnounced = true;
    }

    // ============================================
    // TURN CALIBRATION SEQUENCE
    // ============================================
    if (turnCalibPhase != TCAL_DONE) {
        if (turnCalibPhase == TCAL_WAIT) {
            turnCalibPhase = TCAL_COUNTDOWN;
            turnCountdownSecs = 3;
            turnCalibTimer = millis();
        } else if (turnCalibPhase == TCAL_COUNTDOWN) {
            // Allow Adafruit IO and Display to keep processing to prevent crashes
            io.run();
            if (millis() - turnCalibTimer >= 1000) {
                turnCalibTimer = millis();
                if (turnCountdownSecs > 0) {
                    char titleBuf[32];
                    snprintf(titleBuf, sizeof(titleBuf), "Rep %d/3", turnCalibRep + 1);
                    char subBuf[32];
                    snprintf(subBuf, sizeof(subBuf), "Spin in %d...", turnCountdownSecs);
                    display.showMessage(titleBuf, subBuf);
                    Serial.println(subBuf);
                    turnCountdownSecs--;
                } else {
                    turnCalibPhase = TCAL_SPIN;
                    turnCalibTimer = millis();
                    currentRepMaxSpeed = 0.0f;
                    display.showMessage("SPIN!", "Keep hand stable");
                    Serial.println(">>> SPIN! <<<");
                }
            }
        } else if (turnCalibPhase == TCAL_SPIN) {
            io.run(); // Keep connection alive
            // Track max speed
            float gx = sample.gyroX * 57.2958f;
            float gy = sample.gyroY * 57.2958f;
            float gz = sample.gyroZ * 57.2958f;
            float currentTurnSpeed = sqrt(gx*gx + gy*gy + gz*gz);
            if (currentTurnSpeed > currentRepMaxSpeed) {
                currentRepMaxSpeed = currentTurnSpeed;
            }

            // Spin for 3.5 seconds
            if (millis() - turnCalibTimer >= 3500) {
                turnCalibMaxSpeeds[turnCalibRep] = currentRepMaxSpeed;
                Serial.printf("Rep %d Max Speed: %.2f deg/s\n", turnCalibRep + 1, currentRepMaxSpeed);
                turnCalibRep++;
                if (turnCalibRep >= 3) {
                    turnCalibPhase = TCAL_DONE;
                    // Calculate average max speed
                    float avgMax = (turnCalibMaxSpeeds[0] + turnCalibMaxSpeeds[1] + turnCalibMaxSpeeds[2]) / 3.0f;
                    // Cap it to sane values (don't let it go below 20 or above 150)
                    float finalThresh = avgMax * 0.5f;
                    if (finalThresh < 20.0f) finalThresh = 20.0f;
                    if (finalThresh > 150.0f) finalThresh = 150.0f;
                    
                    turnDetector.setTurnThreshold(finalThresh);
                    
                    Serial.printf("[+] Turn Calibration Complete! Avg Max Speed: %.2f, New Threshold: %.2f deg/s\n", avgMax, finalThresh);
                    display.showMessage("Done!", "Calibration complete");
                    delay(5000); // Wait 5s to let the user read "Done!"
                    
                    // Reset normal UI state flags so it transitions properly
                    latestTurn.fogDetected = false;
                    latestTurn.fogConfidence = 0;
                    latestTremor.detected = false;
                    display.update(UI::UIState::NORMAL);
                } else {
                    turnCalibPhase = TCAL_COUNTDOWN;
                    turnCountdownSecs = 3;
                    turnCalibTimer = millis();
                }
            }
        }
        return; // BLOCK REST OF LOOP UNTIL TCAL_DONE
    }
    // ============================================

    // Feed sample to turn detector
    turnDetector.updateSample(sample);
    latestTurn = turnDetector.getResult();

    // Removed Discord FOG Alert logic

    // When a full window is ready, extract features and run analysis
    if (motionEngine.hasWindow()) {
        latestFeatures = motionEngine.extract();

        // Run tremor analysis
        latestTremor = tremorDetector.analyze(latestFeatures);

        // Log telemetry locally to Serial
        telemetryLogger.logFeatures(latestFeatures, motionEngine.getState());

        // ==============================
        // Adafruit IO Sync (Rate Limited)
        // ==============================
        
        // 1. FOG Sync
        if (latestTurn.fogDetected) {
            if (!_wasFOG) {
                fogLog->save(latestTurn.fogConfidence);
                _wasFOG = true;
            } else {
                fogLog->save(latestTurn.fogConfidence); // Keep sending active confidence
            }
        } else {
            if (_wasFOG) {
                fogLog->save(0); // Drop back to baseline 0
                _wasFOG = false;
            }
        }

        // 2. Tremor Sync
        if (latestTremor.detected) {
            int severityVal = 0;
            switch (latestTremor.severity) {
                case Detection::TremorResult::MILD: severityVal = 1; break;
                case Detection::TremorResult::MODERATE: severityVal = 2; break;
                case Detection::TremorResult::SEVERE: severityVal = 3; break;
                default: severityVal = 0; break;
            }
            if (!_wasTremor) {
                tremorLog->save(severityVal);
                _wasTremor = true;
            } else {
                tremorLog->save(severityVal); // Keep sending active spike
            }
        } else {
            if (_wasTremor) {
                tremorLog->save(0); // Drop back to baseline 0
                _wasTremor = false;
            }
        }
    }
}