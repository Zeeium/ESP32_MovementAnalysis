#ifndef DISPLAY_MANAGER_H
#define DISPLAY_MANAGER_H

#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>
#include "../Processing/MotionEngine.h"
#include "../Detection/TremorDetector.h"
#include "../Detection/TurnDetector.h"

namespace UI {

enum class UIState {
    BOOT,
    NORMAL,
    TREMOR,
    FREEZING,
    FALL
};

class DisplayManager {
public:
    DisplayManager();

    void begin();
    
    // Custom messaging
    void showMessage(const char* title, const char* subtitle = nullptr);
    
    // Non-blocking update called every loop iteration
    void update(UIState newState);
    
    // Debug view for raw metrics
    void updateDebug(const MotionFeatures& f, const Detection::TremorResult& tremor);

private:
    Adafruit_ST7789 _tft;
    bool _initialized;
    
    UIState _currentState;
    UIState _lastState;
    bool _stateJustChanged;

    // Animation state
    uint32_t _lastFrameTime;
    int _animFrame;
    
    static const int CIRCLE_X = 120;
    static const int CIRCLE_Y = 80;

    // Color palettes
    static const uint16_t GREEN_PALETTE[6];
    static const uint16_t YELLOW_PALETTE[6];
    static const uint16_t BLUE_PALETTE[6];
    static const uint16_t RED_PALETTE[6];

    void drawARCH();
    void drawHeader(const char* title);
    void drawBreathingCircle(const uint16_t colors[]);
};

} // namespace UI

#endif // DISPLAY_MANAGER_H
