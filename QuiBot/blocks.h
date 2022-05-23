#ifndef BLOCKS_H
#define BLOCKS_H

#include <Arduino.h>

typedef struct {
    const char* name;
    const uint16_t r;
    const uint16_t g;
    const uint16_t b;
} color_t;

// Colors ID
#define BK 0
#define RD 1
#define GN 2
#define BU 3
#define YE 4
#define OG 5
#define VT 6

// Color table size
#define NUM_COLORS 7

// Colors table
const color_t colors[NUM_COLORS] = { 
    {"BK", 80, 80, 80},
    {"RD", 206, 34, 35},
    {"GN", 125, 92, 43},
    {"BU", 96, 96, 66},
    {"YE", 153, 71, 32},
    {"OG", 194, 44, 31},
    {"VT", 134, 74, 54}
};

void blocks_setup();
uint8_t read_block_color();

// SERVO
extern const uint16_t OPEN_POSITION;
extern const uint16_t EJECT_POSITION;

void servo_move_to(uint16_t target_position);

#endif