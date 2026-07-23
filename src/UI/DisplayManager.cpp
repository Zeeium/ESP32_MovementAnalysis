#include "DisplayManager.h"

namespace UI {

DisplayManager::DisplayManager()
    : _tft(TFT_CS, TFT_DC, TFT_RST),
      _initialized(false),
      _lastState(CALIBRATING),
      _needsFullRedraw(true) {}

void DisplayManager::begin() {
    // Power on the TFT backlight
    pinMode(TFT_I2C_POWER, OUTPUT);
    digitalWrite(TFT_I2C_POWER, HIGH);
    delay(10);

    _tft.init(135, 240);
    _tft.setRotation(3); // Landscape, USB on right
    _tft.fillScreen(COLOR_BG);
    _tft.setTextWrap(false);
    _initialized = true;
}

void DisplayManager::showCalibration(uint32_t progress, uint32_t total) {
    if (!_initialized) return;

    if (_needsFullRedraw) {
        _tft.fillScreen(COLOR_BG);
        drawHeader("CALIBRATING", COLOR_CYAN);
        _tft.setTextSize(1);
        _tft.setTextColor(COLOR_TEXT);
        _tft.setCursor(10, 50);
        _tft.print("Keep hand still...");
        _needsFullRedraw = false;
    }

    // Progress bar
    int barWidth = 200;
    int barHeight = 14;
    int barX = 20;
    int barY = 80;
    int filled = (int)((float)progress / total * barWidth);
    if (filled > barWidth) filled = barWidth;

    _tft.drawRect(barX, barY, barWidth, barHeight, COLOR_DIM);
    _tft.fillRect(barX + 1, barY + 1, filled - 2, barHeight - 2, COLOR_GREEN);
}

void DisplayManager::showMonitoring(const MotionFeatures& features,
                                     const Detection::TremorResult& tremor,
                                     const Detection::FallResult& fall,
                                     MotionState state) {
    if (!_initialized) return;

    // Full redraw only on state transitions
    if (state != _lastState || _needsFullRedraw) {
        _tft.fillScreen(COLOR_BG);
        _needsFullRedraw = false;
        _lastState = state;
    }

    // Header bar with state color
    uint16_t stateColor;
    const char* stateStr;
    switch (state) {
        case RESTING:             stateColor = COLOR_GREEN;  stateStr = "RESTING"; break;
        case NORMAL_MOVEMENT:     stateColor = COLOR_CYAN;   stateStr = "MOVING"; break;
        case REPETITIVE_MOVEMENT: stateColor = COLOR_YELLOW; stateStr = "REPETITIVE"; break;
        default:                  stateColor = COLOR_DIM;    stateStr = "---"; break;
    }
    drawHeader(stateStr, stateColor);

    // Tremor info
    const char* sevStr;
    uint16_t sevColor;
    switch (tremor.severity) {
        case Detection::TremorResult::MILD:     sevStr = "MILD";     sevColor = COLOR_YELLOW; break;
        case Detection::TremorResult::MODERATE:  sevStr = "MODERATE"; sevColor = COLOR_ORANGE; break;
        case Detection::TremorResult::SEVERE:    sevStr = "SEVERE";   sevColor = COLOR_RED; break;
        default:                                 sevStr = "NONE";     sevColor = COLOR_GREEN; break;
    }

    drawLabelStr(25, "Tremor:", sevStr, sevColor);
    drawLabelValue(40, "Confidence:", tremor.confidence, 0, COLOR_TEXT);
    drawLabelValue(55, "Freq:", tremor.dominantFrequency, 1, COLOR_TEXT);

    // Motion data
    drawLabelValue(75, "Gyro RMS:", features.gyroRMS, 3, COLOR_CYAN);
    drawLabelValue(90, "Accel:", features.accelerationRMS, 2, COLOR_CYAN);
    drawLabelValue(105, "Autocorr:", features.normalizedAutocorrelationPeak, 2, COLOR_CYAN);

    // Orientation
    _tft.setTextSize(1);
    _tft.setTextColor(COLOR_DIM, COLOR_BG);
    _tft.setCursor(150, 75);
    _tft.print("R:"); _tft.print(features.roll, 0); _tft.print("  ");
    _tft.setCursor(150, 90);
    _tft.print("P:"); _tft.print(features.pitch, 0); _tft.print("  ");

    // Fall status
    if (fall.fallSuspected) {
        _tft.fillRect(0, 120, 240, 15, COLOR_RED);
        _tft.setTextColor(COLOR_TEXT);
        _tft.setCursor(10, 122);
        _tft.print("!! FALL DETECTED !!");
    }
}

void DisplayManager::showFallWarning(const Detection::FallResult& fall) {
    if (!_initialized) return;

    _tft.fillScreen(COLOR_RED);
    _tft.setTextSize(2);
    _tft.setTextColor(COLOR_TEXT);
    _tft.setCursor(20, 20);
    _tft.print("FALL");
    _tft.setCursor(20, 45);
    _tft.print("SUSPECTED!");

    _tft.setTextSize(1);
    _tft.setCursor(20, 80);
    _tft.print("Confidence: ");
    _tft.print(fall.confidence, 0);
    _tft.print("%");

    _tft.setCursor(20, 110);
    _tft.print("Remain still for help");
}

void DisplayManager::showTapTest(uint16_t tapCount, bool active) {
    if (!_initialized) return;

    if (_needsFullRedraw) {
        _tft.fillScreen(COLOR_BG);
        drawHeader("TAP TEST", COLOR_CYAN);
        _needsFullRedraw = false;
    }

    _tft.setTextSize(2);
    _tft.setTextColor(COLOR_TEXT, COLOR_BG);
    _tft.setCursor(20, 50);
    if (active) {
        _tft.print("Taps: ");
        _tft.print(tapCount);
        _tft.print("   "); // Clear trailing digits
    } else {
        _tft.print("TAP NOW!  ");
    }
}

void DisplayManager::showTapResults(const Detection::BradykinesiaResult& result) {
    if (!_initialized) return;

    _tft.fillScreen(COLOR_BG);
    drawHeader("TAP RESULTS", COLOR_GREEN);

    drawLabelValue(30, "Total Taps:", result.tapCount, 0, COLOR_TEXT);
    drawLabelValue(50, "Avg Interval:", result.averageTapInterval, 2, COLOR_CYAN);
    drawLabelValue(70, "Avg Amplitude:", result.averageAmplitude, 2, COLOR_CYAN);

    uint16_t dropColor = (result.performanceDropRatio < 0.7f) ? COLOR_RED : COLOR_GREEN;
    drawLabelValue(90, "Perf Drop:", result.performanceDropRatio, 2, dropColor);
}

// ==============================
// Helper Drawing Functions
// ==============================

void DisplayManager::drawHeader(const char* title, uint16_t color) {
    _tft.fillRect(0, 0, 240, 18, color);
    _tft.setTextSize(1);
    _tft.setTextColor(COLOR_BG);
    _tft.setCursor(5, 5);
    _tft.print(title);
}

void DisplayManager::drawLabelValue(int y, const char* label, float value, int decimals, uint16_t color) {
    _tft.setTextSize(1);
    _tft.setTextColor(COLOR_DIM, COLOR_BG);
    _tft.setCursor(5, y);
    _tft.print(label);
    _tft.setTextColor(color, COLOR_BG);
    _tft.setCursor(90, y);
    _tft.print(value, decimals);
    _tft.print("     "); // Clear trailing chars
}

void DisplayManager::drawLabelStr(int y, const char* label, const char* value, uint16_t color) {
    _tft.setTextSize(1);
    _tft.setTextColor(COLOR_DIM, COLOR_BG);
    _tft.setCursor(5, y);
    _tft.print(label);
    _tft.setTextColor(color, COLOR_BG);
    _tft.setCursor(90, y);
    _tft.print(value);
    _tft.print("       "); // Clear trailing chars
}

} // namespace UI
