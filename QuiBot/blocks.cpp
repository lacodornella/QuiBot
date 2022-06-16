/**
 * @file blocks.cpp
 * @author Pau Lafoz (pau@lacodornella.cat) & Aleix Badia (aleix@lacodornella.cat)
 * @brief Reads the color sensor and ejects blocks with the servo.
 * @version 0.1
 * @date 2022-05-23
 * 
 */

#include "io.h"
#include "blocks.h"
#include <Wire.h>

// https://github.com/hideakitai/TCS34725
#include "TCS34725.h"

// Servo params
const uint8_t SERVO_CH = 0;
const uint8_t SERVO_PWM_FREQ = 50;
const uint8_t PWM_RESOLUTION = 16;

// SERVO POSITIONS (0-255)
const uint16_t MAX_SERVO_POS     = 8000;
const uint16_t MIN_SERVO_POS     = 3277;
const uint16_t OPEN_POSITION     = 8000;
const uint16_t EJECT_POSITION    = 6450;
uint16_t current_position  = OPEN_POSITION;

// Color sensor object
TCS34725 color_sensor;

void blocks_setup(){
    // Servo PWM setup
    pinMode(SERVO_PWM, OUTPUT);
    ledcSetup(SERVO_CH, SERVO_PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(SERVO_PWM, SERVO_CH);

    // Initialize i2c color sensor
    Wire1.begin(SDA_COL, SCL_COL);
    if (!color_sensor.attach(Wire1)){
        Serial.println("ERROR: TCS34725 NOT FOUND !!!");
    } else {
        Serial.println("Color sensor init OK");
    }
    
    color_sensor.integrationTime(50); // ms
    color_sensor.gain(TCS34725::Gain::X04);
    color_sensor.scale(1.0);
}


/**
 * @brief Move the servo and the cam to the desired position (max range 7-32)
 * 
 * @param target_position 
 */
void servo_move_to(uint16_t target_position) {
    if (target_position > MAX_SERVO_POS) return;
    if (target_position < MIN_SERVO_POS) return;
    const uint8_t INCREMENT = 10;    // Duty cycle increment per cycle
    
    while (true) {
        uint16_t current_position = ledcRead(SERVO_CH);
        if (current_position < target_position - INCREMENT) {
            current_position += INCREMENT;
        } else if (current_position > target_position + INCREMENT) {
            current_position -= INCREMENT;
        } else {
            current_position = target_position;
        }
        ledcWrite(SERVO_CH, current_position);
        vTaskDelay(pdMS_TO_TICKS(1));
        if (current_position == target_position) return;
    }
}


/**
 * @brief Calculates the difference between two RGB colors
 * 
 * @param c1 A `TCS34725::Color` object pointer
 * @param c2 A `color_t` object pointer
 * @return uint16_t Absolute value of the difference
 */
uint32_t calc_colors_difference(const TCS34725::Color * c1, const color_t* c2){
    // As TCS34725::Color is composed by `uint16_t`, we map it to `uint8_t`
    uint16_t diff_r = ((c1->r) > (c2->r)) ? ((c1->r) - (c2->r)) : ((c2->r) - (c1->r));
    uint16_t diff_g = ((c1->g) > (c2->g)) ? ((c1->g) - (c2->g)) : ((c2->g) - (c1->g));
    uint16_t diff_b = ((c1->b) > (c2->b)) ? ((c1->b) - (c2->b)) : ((c2->b) - (c1->b));
    return abs(diff_r) + abs(diff_g) + abs(diff_b);
}

/**
 * @brief Reads the block color from the I2C color sensor. If the sensor is not available or no color matches are found, returns Black.
 * 
 * @return uint8_t the color `id`
 */
uint8_t read_block_color(){
    // return RD;
    if (color_sensor.available()){
        // Color sensor data is received
        const TCS34725::Color measured_color = color_sensor.color();
        uint32_t min_difference = UINT32_MAX;               // Saves the minimum color difference found
        uint8_t min_diff_color_id = BK;                     // Saves the color id which had the smallest difference
        
        Serial.print(measured_color.r);
        Serial.print(":");
        Serial.print(measured_color.g);
        Serial.print(":");
        Serial.println(measured_color.b);

        // Let's compare the colors and find which is less different
        for (uint8_t id = 0; id < NUM_COLORS; id++){
            const uint16_t MAX_DIFFERENCE = 15;            // Differences greater than this value won't count
            uint16_t difference = calc_colors_difference(&measured_color, &colors[id]);
            Serial.print(id);
            Serial.print(": Diff : ");
            Serial.println(difference);
            if (difference < MAX_DIFFERENCE){
                if (difference < min_difference) {
                    min_difference = difference;
                    min_diff_color_id = id;
                }
            }
        }
        Serial.println(min_diff_color_id);
        return min_diff_color_id;
    } else {
        return BK;
    }
}
