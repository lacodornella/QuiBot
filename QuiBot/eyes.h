#ifndef EYES_H
#define EYES_H

#include <FastLED.h>

typedef struct {
    const uint8_t len;      // Number of leds to be turned on
    const uint8_t* leds;    // Pointer to a table with the leds ID that must be ON.
} eyes_leds_t;


extern eyes_leds_t EYES_OPEN;
extern eyes_leds_t EYES_FW;

void eyes_setup();
void task_update_leds(void* params);
void eyes_turn_off();
void eyes_turn_on(eyes_leds_t *shape, CRGB color, uint8_t repeat = 1);

#endif