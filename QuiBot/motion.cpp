/**
 * @file motion.cpp
 * @author Pau Lafoz (pau@lacodornella.cat) & Aleix Badia (aleix@lacodornella.cat)
 * @brief Controls the wheels, arms and syringe stepper motors from Qui-Bot
 * @version 0.1
 * @date 2022-04-29
 * 
 */

#include "io.h"
#include <AccelStepper.h>
#include <Wire.h>
#include "Adafruit_VL53L0X.h"
#include "motion.h"

// Wheels AccelStepper objects
AccelStepper wheel_R(AccelStepper::DRIVER, STEP_R_W, DIR_R_W);
AccelStepper wheel_L(AccelStepper::DRIVER, STEP_L_W, DIR_L_W);

// Arms AccelStepper objects
AccelStepper arm_R(AccelStepper::DRIVER, STEP_R_A, DIR_R_A);
AccelStepper arm_L(AccelStepper::DRIVER, STEP_L_A, DIR_L_A);

// Syringe AccelStepper object
AccelStepper syringe(AccelStepper::DRIVER, STEP_SYRINGE, DIR_SYRINGE);

// Distance sensor object
Adafruit_VL53L0X dist_sensor = Adafruit_VL53L0X();

// Task name strings
const char* MOVE_TO_CROSSING    = "Move to crossing";
const char* TURN_90_CW          = "Turn 90 CW";
const char* TURN_90_CCW         = "Turn 90 CCW";
const char* MOVE_TO_OBJECT      = "Move to object";
const char* TAKE_SOMETHING      = "Take something";
const char* LEAVE_SOMETHING     = "Leave something";
const char* DO_NOTHING          = "Do nothing";

/*****************
 * Motion Params *
 *****************/
float wheels_max_speed  = 130;      // The desired maximum speed in steps per second
float wheels_accel      = 190;      // The desired acceleration in steps per second per second
float arms_max_speed    = 250;      // The desired maximum speed in steps per second
float arms_accel        = 125;      // The desired acceleration in steps per second per second
float syringe_max_speed = 800;      // The desired maximum speed in steps per second
float syringe_accel     = 500;      // The desired acceleration in steps per second per second

// Wheels
const uint8_t WHEEL_MECH_REDUCTION          = 5;
const uint16_t WHEEL_STEPS_PER_REVOLUTION   = 200 * WHEEL_MECH_REDUCTION;
const uint8_t MM_TO_CROSSING_CENTER         = 62;
const uint8_t MM_TO_OBJECT                  = 20;
const uint16_t BLACK_THRESHOLD              = 1500;          // Value to be considered black.


// Arm positions
const uint16_t ARM_LOWER_POSITION     = 120;  // Steps from home to lower position
const uint16_t ARM_L_UPPER_POSITION   = 900;  // Steps from home to upper position
const uint16_t ARM_R_UPPER_POSITION   = 550;  // Steps from home to upper position

// True when the wheels are controlled by speed target, false when position target.
bool wheels_speed_mode              = false;

// Frequency in Hz to check the line follower
const uint8_t LINE_FOLLOWER_FREQ    = 100;  

// Period in ms to check the line follower
const uint8_t LINE_FOLLOWER_PERIOD  = 1000 / LINE_FOLLOWER_FREQ;

/**
 * @brief Enables or disables the wheel steppers 
 * 
 * @param state `ON` or `OFF`
 */
void enable_wheels(bool state){
    digitalWrite(EN_W, state);
}


/**
 * @brief Enables or disables the arm steppers 
 * 
 * @param state `ON` or `OFF`
 */
void enable_arms(bool state){
    digitalWrite(EN_A, state);
}


/**
 * @brief Enables or disables the syringe stepper 
 * 
 * @param state `ON` or `OFF`
 */
void enable_syringe(bool state){
    digitalWrite(EN_SERVO, state);
}


/**
 * @brief Returns `true` when the hall effect endstop is detecting.
 * 
 * @param pin where the endstop is connected.
 */
bool is_endstop_detecting(uint8_t pin) {
    return !digitalRead(pin);
}


/**
 * @brief Move both arms to a position in steps from home and wait to arrive. The task calls `vTaskDelay()` to allow other tasks run.
 * 
 * @param int16_t position in steps from home
 */
void move_arms_to(int16_t position) {
    arm_L.moveTo(position);
    arm_R.moveTo(position);

    while (arm_L.distanceToGo() != 0 && arm_R.distanceToGo() != 0) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void move_arms_up() {
    arm_L.moveTo(ARM_L_UPPER_POSITION);
    arm_R.moveTo(ARM_R_UPPER_POSITION);

    while (arm_L.distanceToGo() != 0 && arm_R.distanceToGo() != 0) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}


/**
 * @brief Move both wheels to a position in steps from home and wait to arrive. The task calls `vTaskDelay()` to allow other tasks run.
 * 
 * @param int16_t position in steps from home
 * @param bool invert one of the wheels to make the robot turn (optional, false by default)
 */
void move_wheels_to(int16_t position, bool invert = false) {
    wheel_L.moveTo(position);
    wheel_R.moveTo(invert ? -position : position);

    while (wheel_L.isRunning() && wheel_R.isRunning()) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void wheels_set_position(long position = 0){
    wheel_L.setCurrentPosition(position);
    wheel_R.setCurrentPosition(position);
}

void wheels_set_speed(float speed, bool rotate = false, bool dir = CW){
    if (rotate) {
        wheel_L.setSpeed(dir == CW ? speed : -speed);            
        wheel_R.setSpeed(dir == CW ? -speed : speed);            
    } else {
        wheel_L.setSpeed(speed);
        wheel_R.setSpeed(speed);
    }
}

/**
 * @brief Steppers setup (acceleration, max speed...) and distance sensor
 */
void motion_setup(){
    // Enable pins
    pinMode(EN_W, OUTPUT);
    pinMode(EN_A, OUTPUT);
    pinMode(EN_SERVO, OUTPUT);
    pinMode(END_L_A, INPUT);
    pinMode(END_R_A, INPUT);
    pinMode(END_SYRINGE, INPUT);

    enable_arms(OFF);
    enable_wheels(OFF);

    // Stepper parameters
    wheel_R.setMaxSpeed(wheels_max_speed);
    wheel_R.setAcceleration(wheels_accel);
    wheel_L.setMaxSpeed(wheels_max_speed);
    wheel_L.setAcceleration(wheels_accel);
    arm_R.setMaxSpeed(arms_max_speed);
    arm_L.setMaxSpeed(arms_max_speed);
    arm_R.setAcceleration(arms_accel);
    arm_L.setAcceleration(arms_accel);
    syringe.setMaxSpeed(syringe_max_speed);
    syringe.setAcceleration(syringe_accel);

    // Distance sensor init
    Wire.begin();
    if (!dist_sensor.begin(0x31, true, &Wire)) {
        Serial.println("Could not find VL53LOX laser distance sensor!");
    } else {
        Serial.println("VL53LOX Laser distance sensor connected.");
    }
}


uint32_t mm_to_steps(uint16_t mm) {
    // Perímetre roda Ø152mm = 2 * PI * 76 = 477.28mm
    // Voltes totals per 1mm = 1 / 477.28 = 0.002 voltes de roda
    return ((mm * WHEEL_STEPS_PER_REVOLUTION * 2) / 1000);
}

/*****************
 * HOMING CYCLES *
 *****************/

/**
 * @brief Arms homing cycle (blocking)
 */
void arms_home(){
    enable_arms(ON);

    arm_L.move(-1250);
    arm_R.move(-1250);

    while (true){
        if (!is_endstop_detecting(END_L_A)) arm_L.run();
        if (!is_endstop_detecting(END_R_A)) arm_R.run();
        if (is_endstop_detecting(END_L_A) && is_endstop_detecting(END_R_A)) break;
        if (arm_L.distanceToGo() == 0 && arm_R.distanceToGo() == 0) break;
        // if (is_endstop_detecting(END_L_A)) break;
    }
    arm_L.setCurrentPosition(0);
    arm_R.setCurrentPosition(0);
    arm_L.move(ARM_L_UPPER_POSITION);
    arm_R.move(ARM_R_UPPER_POSITION);
    
    while (true) {
        if (arm_L.distanceToGo() == 0 && arm_R.distanceToGo() == 0) break;
        arm_L.run();
        arm_R.run();
    }
}


/**
 * @brief Syringe homing cycle (blocking)
 */
void syringe_home(){
    enable_syringe(ON);

    syringe.move(-11000);

    while (!is_endstop_detecting(END_SYRINGE)){
        syringe.run();
    }
    
    syringe.stop();
    syringe.setCurrentPosition(0);

    enable_syringe(OFF);
}


/**
 * @brief Measures the distance (mm) to the next object in front of the robot
 * 
 * @return uint16_t The distance in mm
 */
uint16_t distance_to_object(){    
    VL53L0X_RangingMeasurementData_t measure;
    dist_sensor.rangingTest(&measure, false);
    if (measure.RangeStatus == 0) { 
        Serial.print("Distance (mm): "); Serial.println(measure.RangeMilliMeter);
        return measure.RangeMilliMeter;
    } else {
        // Out of range
        return UINT16_MAX;
    }
}


/**
 * @brief Non-blocking line follower. Must be called regularly.
 * @param speed
 * @return a value indicating if the path is cleared, os a crossing or object is reached when `dir` = true (FW)
 */
line_follower_response_t follow_line_loop(float speed, bool dir = true){
    const uint16_t distance_threshold = 50;          // In mm

    uint16_t line_follower_R = analogRead(LINES_R);
    uint16_t line_follower_L = analogRead(LINES_L);
    
    if (line_follower_L > BLACK_THRESHOLD && line_follower_R > BLACK_THRESHOLD && dir){
        return CROSSING;
    } else if (distance_to_object() < distance_threshold && dir){
        return OBJECT;
    } else {
        const uint16_t p_factor = 5;                      // Proportional factor (dividing)
        int32_t error = line_follower_R - line_follower_L;
        int32_t correction_R = error / p_factor;
        int32_t correction_L = -correction_R;

        wheel_L.setSpeed(dir ? speed + correction_R : -speed + correction_R);
        wheel_R.setSpeed(dir ? speed + correction_L : -speed + correction_L);
        return CLEAR;
    }
}

void run_to_crossing_center(float * speed) {
    wheels_set_position(0 - mm_to_steps(MM_TO_CROSSING_CENTER));

    // Must move some mm's more to center the robot's wheels to the crossing
    while (follow_line_loop(*speed) == CROSSING) {
        wheels_set_speed(*speed);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    *speed = *speed / 2; 
    while (wheel_L.currentPosition() < 0 && wheel_R.currentPosition() < 0) {
        wheels_set_speed(*speed);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    wheels_set_speed(0);
}

/**
 * @brief Computes the new speed during an acceleration. Uses `wheels_accel` to calculate speed.
 * 
 * @param old_speed pointer to the 
 */
void compute_new_speed(float * speed){
    const float accel = wheels_accel / LINE_FOLLOWER_FREQ;
    if (*speed < (wheels_max_speed - accel)){
            *speed += accel;
        } else {
            *speed = wheels_max_speed;
        }
}

void task_move_to(void *expected_target){
    // Line follower response
    line_follower_response_t lf_response;

    // Initial speed to 0 to accelerate
    float speed = 0;
    wheels_speed_mode = true;
    enable_wheels(ON);

    while (true){
        compute_new_speed(&speed);
        lf_response = follow_line_loop(speed);
        if (lf_response != CLEAR) break;
        vTaskDelay(pdMS_TO_TICKS(LINE_FOLLOWER_PERIOD));
    }


    if ((lf_response == CROSSING) && (*(line_follower_response_t*)expected_target = CROSSING)) {
        run_to_crossing_center(&speed);
    } else if ((lf_response == OBJECT) && (*(line_follower_response_t*) expected_target = OBJECT)){
        // Object found
        wheel_L.stop();
        wheel_R.stop();
    } else {
        // What to do?
    }

    wheels_speed_mode = false;

    enable_wheels(OFF);

    // Suspend own task
    vTaskSuspend(NULL);
}


void task_rotate(void *direction){
    // Arch 90º a Ø250mm -> 2 * PI * 125mm / 4 = 196.25mm
    // Perímetre roda Ø152mm = 2 * PI * 76 = 477.28mm
    // Voltes totals per 90º = 196.25 / 477.28 = 0.41 voltes de roda
    const uint32_t ROTATION_STEPS = (WHEEL_STEPS_PER_REVOLUTION * 42) / 100;    // Steps to rotate the robot 90º
    AccelStepper * positive_wheel = (*(bool*) direction == CW ? &wheel_L : &wheel_R);

    // Initial speed to 0 to accelerate
    float speed = 0;
    wheels_speed_mode = true;    
    enable_wheels(ON);

    wheels_set_position(0);

    while (positive_wheel->currentPosition() < (ROTATION_STEPS * 90) / 100) {
        compute_new_speed(&speed);
        wheels_set_speed(speed, true, *(bool*)direction);
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    speed /= 2;
    while (true){
        const uint16_t line_follower_R = analogRead(LINES_R);
        const uint16_t line_follower_L = analogRead(LINES_L);
        const uint8_t ERROR_THRESHOLD = 20;
        uint32_t error = abs(line_follower_L - line_follower_R);
        if (error < ERROR_THRESHOLD || positive_wheel->currentPosition() > (ROTATION_STEPS * 110) / 100) break;
        wheels_set_speed(speed - (speed / error), true, *(bool*)direction);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    wheels_set_speed(0);

    enable_wheels(OFF);
    wheels_speed_mode = false;

    // Suspend own task
    vTaskSuspend(NULL);
}

void task_take_or_leave_something(void* take) {
    bool is_taking = *(bool*) take;                     // True when the robot must take something, false when it must leave something.

    const uint8_t SY_FULL_EXTENDED_REV  = 10;           // Revolutions from home to full extended syringe
    const uint16_t SY_STEPS_REVOLUTION = 200 * 4;      // 200 per revolution * microstepping

    float speed = 0;

    // Steps from home to full extended syringe
    const uint16_t SY_FULL_EXTENDED_STEPS = SY_FULL_EXTENDED_REV * SY_STEPS_REVOLUTION;

    // Line follower response
    line_follower_response_t lf_response;
    wheels_speed_mode = true;
    enable_wheels(ON);

    // Set home to be able to come back
    wheels_set_position(0);

    while (true){
        compute_new_speed(&speed);
        lf_response = follow_line_loop(speed);
        if (lf_response != CLEAR) break;
        vTaskDelay(pdMS_TO_TICKS(LINE_FOLLOWER_PERIOD));
    }

    if (lf_response == OBJECT) {
        // Object found
        uint16_t target_pos_L = wheel_L.currentPosition() + mm_to_steps(MM_TO_OBJECT);
        uint16_t target_pos_R = wheel_R.currentPosition() + mm_to_steps(MM_TO_OBJECT);
        while (wheel_L.currentPosition() < target_pos_L && wheel_R.currentPosition() < target_pos_R) {
            wheels_set_speed(speed/2);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        wheels_set_speed(0);
        enable_wheels(OFF);
        enable_arms(ON);

        // Move the arms down
        move_arms_to(ARM_LOWER_POSITION);

        // Move syringe. If `is_taking` is true, syringe will extend. Otherwise will go to 0 (home)
        enable_syringe(ON);
        syringe.moveTo(is_taking ? SY_FULL_EXTENDED_STEPS : 0);
        while (syringe.distanceToGo() != 0) {
            vTaskDelay(pdMS_TO_TICKS(100));
            if (!is_taking) {
                // Going down
                if (is_endstop_detecting(END_SYRINGE)){
                    syringe.stop();
                    syringe.setCurrentPosition(0);
                }
            }
        }
        enable_syringe(OFF);

        // Move the arms up
        move_arms_up();

        // Move the robot back to the crossing (home)
        enable_wheels(ON);
        wheels_speed_mode = false;
        wheels_set_speed(wheels_max_speed);
        move_wheels_to(0);
    } else if (lf_response == CROSSING) {
        // Found crossing but was expecting an object
        run_to_crossing_center(&speed);
        // Should eject all blocks?
    }

    wheels_speed_mode = false;

    enable_wheels(OFF);

    // Suspend own task
    vTaskSuspend(NULL);
}


void task_idle(void * params){
    vTaskDelay(pdMS_TO_TICKS(500));

    // Suspend own task
    vTaskSuspend(NULL);
}



void task_update_steppers(void *pvParameters){
    (void) pvParameters;
    while (true){
        if (wheels_speed_mode) {
            wheel_R.runSpeed();
            taskYIELD();
            wheel_L.runSpeed();
            taskYIELD();
        } else {
            wheel_R.run();
            taskYIELD();
            wheel_L.run();
            taskYIELD();
        }
        arm_R.run();
        taskYIELD();
        arm_L.run();
        taskYIELD();
        syringe.run();
        taskYIELD();
    }
}