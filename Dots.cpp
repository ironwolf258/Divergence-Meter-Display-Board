#include "Dots.h"

Dots::Dots(uint8_t pinA, uint8_t pinB, uint8_t pinC, uint8_t pinD) {
    _pins[0] = pinA;
    _pins[1] = pinB;
    _pins[2] = pinC;
    _pins[3] = pinD;
    _current = 10; // start blank
}

void Dots::begin() {
    for (uint8_t i = 0; i < 4; i++) {
        pinMode(_pins[i], OUTPUT);
        digitalWrite(_pins[i], HIGH); // default off (adjust if active low)
    }
}

void Dots::_writeBCD(uint8_t tube) {
    // For your wiring: HIGH = bit set, LOW = bit clear
    for (uint8_t bit = 0; bit < 4; bit++) {
        bool bitVal = tube & (1 << bit);
        digitalWrite(_pins[bit], bitVal ? HIGH : LOW);
    }
}

void Dots::display(uint8_t tube) {
    if (tube < 9) {
        _writeBCD(tube);
    } else { // blank/off pattern
        digitalWrite(_pins[0], HIGH);
        digitalWrite(_pins[1], HIGH);
        digitalWrite(_pins[2], HIGH);
        digitalWrite(_pins[3], HIGH);
    }
    _current = digit;
}

void Dots::clear() {
    display(10);
}

uint8_t Dots::value() const {
    return _current;
}
