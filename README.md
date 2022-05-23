# QuiBot

QuiBot is a robot to experiment basic sequential programming with children. No computer is needed as it can be programmed by using painted wooden blocks, where each color represents a different action. The robot has a backpack that can contain up to 12 blocks that execute as soon as they're consumed.

## Hardware

The controller used is an ESP32 that controls the following devices:
 - Motion:
    - Wheels: 2x NEMA 17 Steppers with planetary reductor
    - Arms: 2x NEMA 17 Steppers with timing belt reduction
    - Syringe: NEMA 17 Stepper with endless screw
    - Block ejection: 5V Servomotor
 - Sensors:
    - Color sensor: TCS34725
    - Laser distance sensor: VL53L0X
    - Line follower: 2x TCRT5000
    - Arm endstops: 2x Hall effect sensor
    - Syringe endstop: Hall effect sensor
 - Eyes:
    - 2x 8x8 Adressable RGB Led matrix



## Installation

The ESP32 can be programmed using the Arduino IDE. 
