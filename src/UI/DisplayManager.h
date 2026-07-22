#ifndef DISPLAY_MANAGER_H
#define DISPLAY_MANAGER_H

#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>
#include "../Processing/MotionEngine.h"
#include "../Detection/TremorDetector.h"
#include "../Detection/FallDetector.h"
#include "../Detection/BradykinesiaDetector.h"

namespace UI {

class DisplayManager {
public:
    DisplayManager();

    void begin();

    // Primary monitoring screen
    void showMonitoring(const MotionFeatures& features,
                        const Detection::TremorResult& tremor,
                        const Detection::FallResult& fall,
                        MotionState state);

    // Calibration screen
    void showCalibration(uint32_t progress, uint32_t total);

    // Fall warning screen
    void showFallWarning(const Detection::FallResult& fall);

    // Tap test screen
    void showTapTest(uint16_t tapCount, bool active);
    void showTapResults(const Detection::BradykinesiaResult& result);

private:
    Adafruit_ST7789 _tft;
    bool _initialized;
    MotionState _lastState;
    bool _needsFullRedraw;

    // Color palette
    static const uint16_t COLOR_BG       = 0x0000; // Black
    static const uint16_t COLOR_TEXT      = 0xFFFF; // White
    static const uint16_t COLOR_GREEN    = 0x07E0;
    static const uint16_t COLOR_RED      = 0xF800;
    static const uint16_t COLOR_YELLOW   = 0xFFE0;
    static const uint16_t COLOR_CYAN     = 0x07FF;
    static const uint16_t COLOR_ORANGE   = 0xFD20;
    static const uint16_t COLOR_DIM      = 0x7BEF; // Gray
    
    void drawHeader(const char* title, uint16_t color);
    void drawLabelValue(int y, const char* label, float value, int decimals, uint16_t color);
    void drawLabelStr(int y, const char* label, const char* value, uint16_t color);
};

} // namespace UI

#endif // DISPLAY_MANAGER_H
