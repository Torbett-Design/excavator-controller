#include <Arduino.h>
#include <Ps3Controller.h>
#include "audio.h"
#include "beacon.h"
#include "lights.h"
#include "motors.h"
#include "pins.h"

#define PS3_CONTROLLER_MAC "01:02:03:04:05:06"

void ps3_notify() {
    // Controller input handling code
}

void setupController() {
    Ps3.attach(ps3_notify);
    Ps3.begin(PS3_CONTROLLER_MAC);
}

void setupPins() {
    pinMode(Pins::BATT_STAT, INPUT);
    pinMode(Pins::VBATT_ADC_EN, OUTPUT);
    pinMode(Pins::VBATT_ADC, INPUT);
    pinMode(Pins::SERVO_3, OUTPUT);
    pinMode(Pins::SERVO_2, OUTPUT);
    pinMode(Pins::SERVO_1, OUTPUT);
    pinMode(Pins::V5_EN, OUTPUT);

    // Enable 5V by default
    digitalWrite(Pins::V5_EN, HIGH);
}

void setup() {
    Serial.begin(115200);
    setupPins();
    Motors::setup();
    Audio::setup();
    Lights::setup();
    Beacon::setup();
    setupController();
}

void loop() {
    Beacon::update();
    // Main control loop code
}
