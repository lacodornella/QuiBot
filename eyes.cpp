/**
 * @file eyes.cpp
 * @author Pau Lafoz (pau@lacodornella.cat) & Aleix Badia (aleix@lacodornella.cat)
 * @brief Controls the led eye matrix from Qui-Bot
 * @version 0.1
 * @date 2022-05-23
 * 
 */

#include "eyes.h"
#include "io.h"

const uint8_t ROW_NUM   = 8;                        // Leds per row
const uint8_t COL_NUM   = 8;                        // Leds per column
const uint8_t NUM_LEDS  = ROW_NUM * COL_NUM * 2;    // Total leds number on both eyes

// Table that contains the eye's led information
CRGB eyes[NUM_LEDS];


eyes_leds_t EYES_OPEN = {
    40,
    (uint8_t[]){102, 89, 38, 25, 106, 101, 90, 85, 42, 37, 26, 21,
                107, 100, 91, 84, 43, 36, 27, 20, 108, 99, 92, 83,
                44, 35, 28, 19, 109, 98, 93, 82, 45, 34, 29, 18, 
                97, 94, 33, 30}
};

eyes_leds_t EYES_FW = {
    52,
    (uint8_t[]){103, 88, 39, 24, 105, 102, 89, 86, 41, 38, 25, 22, 
                117, 106, 101, 90, 85, 74, 53, 42, 37, 26, 21, 10, 
                123, 116, 100, 91, 75, 68, 59, 52, 36, 27, 11, 4, 
                99, 92, 35, 28, 98, 93, 34, 29, 97, 94, 33, 30, 96, 
                95, 32, 31}
};


void eyes_setup(){
    FastLED.addLeds<WS2811, LED_MTRX, GRB>(eyes, NUM_LEDS);
}

/**
 * @brief Turn all the leds OFF
 */
void eyes_turn_off(){
    fill_solid(eyes, NUM_LEDS, CRGB::Black);    
}


/**
 * @brief Turn the shape's leds on
 * 
 * @param shape contains the leds to start
 * @param color of the leds to start
 */
void eyes_turn_on(eyes_leds_t *shape, CRGB color) {
    eyes_turn_off();
    for (uint8_t i=0; i < shape->len; i++) {
        eyes[shape->leds[i]] = color;
    }
}

/**
 * @brief Task to update the leds
 * 
 * @param param 
 */
void task_update_leds(void* param) {
    while (true) {
        const uint8_t BRIGHTNESS = 150;     // Range 0 (black) to 255 (Max)
        FastLED.show(BRIGHTNESS);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

