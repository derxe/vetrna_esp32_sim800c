#pragma once
#include <Arduino.h>

class SimpleButton {
public:
    typedef void (*ClickHandler)();

    SimpleButton() {}

    void begin(uint8_t pin) {
        _pin = pin;
        pinMode(_pin, INPUT_PULLUP);
    }

    void setClickHandler(ClickHandler handler) {
        _handler = handler;
    }

    bool isPressedDown() {
        return digitalRead(_pin) == LOW;
    }

    void ignoreNextPress() {
        _ignorePress = true;
    }

    void loop() {
        bool pressed = isPressedDown();

        unsigned long now = millis();

        if (pressed && !_lastPressed) {
            _pressStart = now;
        }

        if (!pressed && _lastPressed) {
            if (now - _pressStart >= 10) {
                if (_handler) {
                    if(!_ignorePress) {
                        _handler();
                    }
                    _ignorePress = false;
                }
            }
        }

        _lastPressed = pressed;
    }

private:
    uint8_t _pin = 0;
    bool _lastPressed = false;
    unsigned long _pressStart = 0;
    bool _ignorePress = false;
    ClickHandler _handler = nullptr;
};
