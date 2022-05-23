# QuiBot

QuiBot is a robot to experiment basic sequential programming for children. No computer is needed as it can be programmed by using painted wooden blocks, where each color represents a different action. The robot has a backpack that can contain up to 12 blocks that execute as soon as they're consumed.

[[./static/QuiBot_3D.jpeg|alt=QuiBot 3D]]

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

The ESP32 controller can be programmed using Arduino IDE. The following libraries are required:
 - [FastLed](https://github.com/FastLED/FastLED)
 - [Adafruit_VL53L0X](https://github.com/adafruit/Adafruit_VL53L0X)
 - [TCS34725](https://github.com/hideakitai/TCS34725)
 - [AccelStepper](https://github.com/waspinator/AccelStepper)


## Development

This robot has been conceived and developed by Aleix Badia and Pau Lafoz from La Codornella under the UPC project "Qui-Bot H2O", led by Marta Tarrés and Toni Dorado, and funded by FECYT.