#include "DisplayManager.h"

namespace UI {

const uint16_t DisplayManager::GREEN_PALETTE[6]  = { 0x0320, 0x04A0, 0x05E0, 0x07E0, 0x05E0, 0x04A0 }; // ST77XX_GREEN = 0x07E0
const uint16_t DisplayManager::YELLOW_PALETTE[6] = { 0x6300, 0xB500, 0xDEA0, 0xFFE0, 0xDEA0, 0xB500 }; // ST77XX_YELLOW = 0xFFE0
const uint16_t DisplayManager::BLUE_PALETTE[6]   = { 0x0018, 0x01DF, 0x03FF, 0x001F, 0x03FF, 0x01DF }; // ST77XX_BLUE = 0x001F
const uint16_t DisplayManager::RED_PALETTE[6]    = { 0x6000, 0x9000, 0xC000, 0xF800, 0xC000, 0x9000 }; // ST77XX_RED = 0xF800

DisplayManager::DisplayManager()
    : _tft(TFT_CS, TFT_DC, TFT_RST),
      _initialized(false),
      _currentState(UIState::BOOT),
      _lastState(UIState::BOOT),
      _stateJustChanged(true),
      _lastFrameTime(0),
      _animFrame(0) {}

void DisplayManager::begin() {
    pinMode(TFT_I2C_POWER, OUTPUT);
    digitalWrite(TFT_I2C_POWER, HIGH);
    
    pinMode(TFT_BACKLITE, OUTPUT);
    digitalWrite(TFT_BACKLITE, HIGH);
    
    delay(10);

    _tft.init(135, 240);
    _tft.setRotation(1);
    _tft.fillScreen(0x0000);
    _initialized = true;

    drawARCH();
}

void DisplayManager::update(UIState newState) {
    if (!_initialized) return;

    if (newState != _currentState) {
        _lastState = _currentState;
        _currentState = newState;
        _stateJustChanged = true;
        _animFrame = 0; // Reset animation on state change
        _lastFrameTime = millis();
    }

    if (_stateJustChanged) {
        _tft.fillScreen(0x0000);
        _stateJustChanged = false;

        switch (_currentState) {
            case UIState::NORMAL:   drawHeader("Normal"); break;
            case UIState::TREMOR:   drawHeader("Tremor"); break;
            case UIState::FREEZING: drawHeader("Freezing"); break;
            case UIState::FALL:     drawHeader("Fall"); break;
            default: break;
        }
    }

    // Animation speeds and radii from original design
    const int animSpeeds[6] = {90, 70, 50, 70, 90, 110};
    
    if (millis() - _lastFrameTime > animSpeeds[_animFrame]) {
        _lastFrameTime = millis();
        
        switch (_currentState) {
            case UIState::NORMAL:   drawBreathingCircle(GREEN_PALETTE); break;
            case UIState::TREMOR:   drawBreathingCircle(YELLOW_PALETTE); break;
            case UIState::FREEZING: drawBreathingCircle(BLUE_PALETTE); break;
            case UIState::FALL:     drawBreathingCircle(RED_PALETTE); break;
            default: break;
        }

        _animFrame++;
        if (_animFrame >= 6) _animFrame = 0;
    }
}

void DisplayManager::drawARCH() {
    _tft.fillScreen(0x0000);
    _tft.setTextColor(0xFFFF);
    _tft.setTextSize(3);
    _tft.setCursor(80, 40);
    _tft.print("ARCH");
}

void DisplayManager::drawHeader(const char* title) {
    _tft.setTextColor(0xFFFF);
    _tft.setTextSize(3);
    
    // Simple centering for standard titles
    int16_t x1, y1;
    uint16_t w, h;
    _tft.getTextBounds(title, 0, 0, &x1, &y1, &w, &h);
    int cursorX = (240 - w) / 2;
    
    _tft.setCursor(cursorX, 20);
    _tft.print(title);
}

void DisplayManager::drawBreathingCircle(const uint16_t colors[]) {
    const int radius[6] = {20, 21, 22, 23, 22, 21};
    int r = radius[_animFrame];
    uint16_t color = colors[_animFrame];

    // Erase the largest possible circle area (r=23+4=27) before drawing the new frame
    _tft.fillCircle(CIRCLE_X, CIRCLE_Y, 28, 0x0000);

    // Draw the new breathing frame
    _tft.drawCircle(CIRCLE_X, CIRCLE_Y, r + 4, color);
    _tft.drawCircle(CIRCLE_X, CIRCLE_Y, r + 2, color);
    _tft.fillCircle(CIRCLE_X, CIRCLE_Y, r, color);
}

void DisplayManager::showMessage(const char* title, const char* subtitle) {
    if (!_initialized) return;
    
    // Clear screen
    _tft.fillScreen(0x0000);
    
    // Draw Title (centered, size 3)
    _tft.setTextColor(0xFFFF);
    _tft.setTextSize(3);
    int16_t x1, y1;
    uint16_t w, h;
    _tft.getTextBounds(title, 0, 0, &x1, &y1, &w, &h);
    int cursorX = (240 - w) / 2;
    _tft.setCursor(cursorX, 30);
    _tft.print(title);
    
    // Draw Subtitle (centered, size 2)
    if (subtitle != nullptr) {
        _tft.setTextSize(2);
        _tft.getTextBounds(subtitle, 0, 0, &x1, &y1, &w, &h);
        cursorX = (240 - w) / 2;
        _tft.setCursor(cursorX, 80);
        _tft.print(subtitle);
    }
}

void DisplayManager::updateDebug(const MotionFeatures& f, const Detection::TremorResult& tremor) {
    if (!_initialized) return;

    // Throttle updates to ~4Hz so it's readable and doesn't flicker
    if (millis() - _lastFrameTime < 250) return;
    _lastFrameTime = millis();

    _tft.fillScreen(0x0000);
    _tft.setCursor(0, 0);
    _tft.setTextColor(0xFFFF);
    _tft.setTextSize(2);
    
    _tft.println("--- DEBUG MODE ---");
    _tft.println("");
    _tft.printf("GyroRMS:  %.2f\n", f.gyroRMS);
    _tft.printf("Energy:   %.2f\n", f.tremorBandEnergy[f.dominantAxis]);
    _tft.printf("Autocorr: %.2f\n", f.normalizedAutocorrelationPeak);
    _tft.println("------------------");
    _tft.printf("Conf:     %.1f%%\n", tremor.confidence);
    _tft.print("Status:   ");
    
    if (tremor.detected) {
        _tft.setTextColor(0xFFE0); // Yellow
        _tft.println("TREMOR");
    } else {
        _tft.setTextColor(0x07E0); // Green
        _tft.println("NORMAL");
    }
}

} // namespace UI
