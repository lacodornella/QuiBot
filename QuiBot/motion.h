#ifndef MOTION_H
#define MOTION_H

// Used to define rotation's direction
const bool CW = true;
const bool CCW = false;

// Used to enable / disable the steppers
const bool ON = LOW;
const bool OFF = HIGH;

// Used to tell the take/leave task what to do
const bool TAKE = true;
const bool LEAVE = false;

extern const char* MOVE_TO_CROSSING;
extern const char* TURN_90_CW;
extern const char* TURN_90_CCW;
extern const char* MOVE_TO_OBJECT;
extern const char* TAKE_SOMETHING;
extern const char* LEAVE_SOMETHING;
extern const char* DO_NOTHING;


typedef enum line_follower_response_t{
    CLEAR,      // Path is cleared
    CROSSING,   // A crossing is reached
    OBJECT      // An object is detected
};

void motion_setup();
void arms_home();
void syringe_home();
void task_update_steppers(void *pvParameters);
void task_move_to(void *expect_target);
void task_rotate(void *dir);
void task_take_or_leave_something(void* must_take);
void task_idle(void * params);
uint16_t distance_to_object();

#endif