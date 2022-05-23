#ifndef IO
#define IO

// Expulse blocks Servo pins
#define EN_SERVO      0
#define SERVO_PWM     2

// Syringe Stepper pins
#define STEP_SYRINGE  4
#define DIR_SYRINGE   5

// Led Matrix Data pin
#define LED_MTRX      12

// Right Wheel Stepper pins
#define STEP_R_W      13
#define DIR_R_W       14

// Left Wheel Stepper pins
#define DIR_L_W       15
#define STEP_L_W      16

// Wheel Steppers Enable pin
#define EN_W          17

// Right Arm Stepper pins
#define DIR_R_A       18
#define STEP_R_A      19

// Left Arm Stepper pins
#define DIR_L_A       23
#define STEP_L_A      25

// Arm Steppers Enable pin
#define EN_A          26

// I2C Distance sensor pins
#define SDA_DIST      21
#define SCL_DIST      22

// I2C Color sensor pins
#define SDA_COL       27
#define SCL_COL       32

// Syringe Endstop 
#define END_SYRINGE   33

// Right Arm Endstop
#define END_R_A       34

// Left Arm Endstop
#define END_L_A       35

// Line Follower Sensors
#define LINES_R       36
#define LINES_L       39

#endif