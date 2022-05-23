/**
 * @file QuiBot.ino
 * @author Pau Lafoz (pau@lacodornella.cat) & Aleix Badia (aleix@lacodornella.cat)
 * @brief Main Qui-Bot control. Tasks creation and execution
 * @version 0.1
 * @date 2022-05-23
 * 
 */

#include "io.h"
#include "motion.h"
#include "blocks.h"
#include "eyes.h"


TaskHandle_t TaskHandle;
#define CPU_CORE 0
#define APP_CORE 1

void task_read_blocks(void *pvParameters){
    (void) pvParameters;

    // Timeouts beetween servo actions
    const uint16_t INSERT_BLOCK_MS  = 2000;
    const uint16_t EJECT_BLOCK_MS   = 2000;
    const uint16_t CHECK_COLOR_MS   = 1000;

    // Got the expected target information to send to `task_move_to`
    static line_follower_response_t expected_target;

    while(true){
        uint8_t color_id;

        eyes_turn_on(&EYES_OPEN, CRGB::Gray);

        // Insert block
        servo_move_to(OPEN_POSITION);

        // Wait until the block is in place
        vTaskDelay(pdMS_TO_TICKS(INSERT_BLOCK_MS));
        
        // Check the block color recursively
        while(true){
            color_id = read_block_color();

            if (color_id == BK){
                // Black color -> No block
                vTaskDelay(pdMS_TO_TICKS(CHECK_COLOR_MS));
            } else {
                // A color is detected.
                break;
            }
        }

        switch (color_id) {
            case RD:
                /* Red - Go ahead */
                expected_target = CROSSING;
                eyes_turn_on(&EYES_FW, CRGB::Red);
                vTaskDelay(pdMS_TO_TICKS(1000));
                xTaskCreatePinnedToCore(task_move_to, MOVE_TO_CROSSING, 2048, (void*) &expected_target, 1, &TaskHandle, APP_CORE);
                Serial.println(MOVE_TO_CROSSING);
                break;
            
            case GN:
                /* Green - Turn Right */
                eyes_turn_on(&EYES_OPEN, CRGB::Green);
                vTaskDelay(pdMS_TO_TICKS(1000));
                xTaskCreatePinnedToCore(task_rotate, TURN_90_CW, 1024, (void*) &CW, 1, &TaskHandle, APP_CORE);
                Serial.println(TURN_90_CW);
                break;
            
            case BU:
                /* Blue - Turn Left */
                eyes_turn_on(&EYES_OPEN, CRGB::Blue);
                vTaskDelay(pdMS_TO_TICKS(1000));
                xTaskCreatePinnedToCore(task_rotate, TURN_90_CCW, 1024, (void*) &CCW, 1, &TaskHandle, APP_CORE);
                Serial.println(TURN_90_CCW);
                break;
            
            case YE:
                /* Yellow - Take something */
                eyes_turn_on(&EYES_OPEN, CRGB::Yellow);
                vTaskDelay(pdMS_TO_TICKS(1000));
                xTaskCreatePinnedToCore(task_take_or_leave_something, TAKE_SOMETHING, 2048, (void*) &TAKE, 1, &TaskHandle, APP_CORE);
                Serial.println(TAKE_SOMETHING);
                break;
            
            case OG:
                /* Orange - Leave something */
                eyes_turn_on(&EYES_OPEN, CRGB::Orange);
                vTaskDelay(pdMS_TO_TICKS(1000));
                xTaskCreatePinnedToCore(task_take_or_leave_something, LEAVE_SOMETHING, 2048, (void*) &LEAVE, 1, &TaskHandle, APP_CORE);
                Serial.println(LEAVE_SOMETHING);
                break;
            
            case VT:
                /* Violet - Leave something */
                eyes_turn_on(&EYES_OPEN, CRGB::Purple);
                xTaskCreatePinnedToCore(task_idle, DO_NOTHING, 700, NULL, 1, &TaskHandle, APP_CORE);
                break;
        
            default:
                break;
        }

        while(eTaskGetState(TaskHandle) != eSuspended){
            vTaskDelay(pdMS_TO_TICKS(500));
        }
        
        eyes_turn_on(&EYES_OPEN, CRGB::Gray);

        // Delete task
        vTaskDelete(TaskHandle);


        // Eject block
        servo_move_to(EJECT_POSITION);

        // Wait the block to fall
        vTaskDelay(pdMS_TO_TICKS(EJECT_BLOCK_MS));
    }
}



void setup(){
    Serial.begin(9600);
    blocks_setup();
    motion_setup();
    eyes_setup();

    // while (true) {
    //     read_block_color();
    //     delay(500);
    // }

    arms_home();
    syringe_home();

    xTaskCreatePinnedToCore(task_update_leds, "Update leds", 1024, NULL, 2, NULL, APP_CORE);
    xTaskCreatePinnedToCore(task_read_blocks, "Read Blocks", 2048, NULL, 2, NULL, APP_CORE);
    xTaskCreatePinnedToCore(task_update_steppers, "Update Steppers", 1024, NULL, 1, NULL, APP_CORE);
}



void loop(){
   vTaskDelay(pdMS_TO_TICKS(1000));
}