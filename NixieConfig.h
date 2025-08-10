#pragma once
#include "NixieTubes.h"
#include "Dots.h"

// How many tubes are in your display
static const uint8_t NUM_TUBES = 8;

// Tube pin mapping: each NixieTube() takes pins for A, B, C, D inputs
static NixieTube tubes[NUM_TUBES] = {
    NixieTube(22,23,24,25), // Tube 0
    NixieTube(26,27,28,29), // Tube 1
    NixieTube(30,31,32,33), // Tube 2
    NixieTube(34,35,36,37), // Tube 3
    NixieTube(38,39,40,41), // Tube 4
    NixieTube(42,43,44,45), // Tube 5
    NixieTube(46,47,48,49), // Tube 6
    NixieTube(50,51,52,53)  // Tube 7
};
static Dots dots = Dots(10,11,12,13);  // Decimal Points
