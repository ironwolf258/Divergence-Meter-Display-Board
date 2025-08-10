#pragma once
#include <Arduino.h>

class Dots {
public:
    // Construct a tube with its four control pins
    Dots(uint8_t pinA, uint8_t pinB, uint8_t pinC, uint8_t pinD);

    // Setup pins
    void begin();

    // Display a number 0–9, or 10 for blank/off
    void display(uint8_t tube);

    // Turn tube completely off
    void clear();

    // Get current value
    uint8_t value() const;

private:
    uint8_t _pins[4];
    uint8_t _current;
    void _writeBCD(uint8_t tube);
};
