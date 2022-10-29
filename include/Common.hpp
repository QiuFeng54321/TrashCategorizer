#ifndef COMMON_HPP
#define COMMON_HPP
#define DECODE_NEC  // Includes Apple and Onkyo

#include <Arduino.h>
#include <EEPROM.h>
#include <HCSR04.h>
#include <Servo.h>
#include <TaskScheduler.h>

#include <IRremote.hpp>

#include "DFRobot_HuskyLens.h"
#include "arduino-timer.h"
#define IR_INPUT_PIN 8
#include "TinyIRReceiver.hpp"

const int StatusRedPin = 10, StatusYellowPin = 11, StatusGreenPin = 9;
// const int StatusLEDBrightness = 255;

enum class ClassifierType { None = 0, Empty = 1, Left = 2, Right = 3 };
enum class IRButtonCode : uint8_t {
    Power = 0x44,
    Left = 0x07,
    Right = 0x09,
    Down = 0x19,
    OK = 0x15,
    Refresh = 0x43
};

#endif