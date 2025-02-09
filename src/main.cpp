#include <Arduino.h>
#include <Ps3Controller.h>
#include "audio.h"
#include "beacon.h"
#include "lights.h"
#include "motors.h"
#include "pins.h"

#define PS3_CONTROLLER_MAC "01:02:03:04:05:06"

enum class State {
    OFF,
    IDLE,
    POWER_UP,
    POWER,
    HYDRAULIC,
    POWER_DOWN,
    MOVING,
    REVERSE
};

State state = State::OFF;

static bool startButtonPressed = false;
static unsigned long startButtonTime = 0;
bool tracksWereMoving = false;

void ps3_notify() {
    static bool wasMoving = false;
    bool isMoving = false;

    if (state != State::OFF) {
        // Forward movement using L2/R2 analog values (0-255)
        Motors::setLeftTrack(Ps3.data.analog.button.l2);
        Motors::setRightTrack(Ps3.data.analog.button.r2);

        // Reverse movement using L1/R1 analog values (0-255)
        if (Ps3.data.analog.button.l1) {
            Motors::setLeftTrack(-Ps3.data.analog.button.l1);
        }
        if (Ps3.data.analog.button.r1) {
            Motors::setRightTrack(-Ps3.data.analog.button.r1);
        }

        isMoving = Ps3.data.analog.button.l1 || Ps3.data.analog.button.r1 || 
                   Ps3.data.analog.button.l2 || Ps3.data.analog.button.r2;

        if (isMoving && !wasMoving) {
            Audio::enginePowerUp();
            Audio::tracks();
            state = State::POWER_UP;
        } else if (!isMoving && wasMoving) {
            Audio::stopTrack(Audio::SOUND_TRACK);
        }
    }
    wasMoving = isMoving;

    // Handle start button state
    if (Ps3.data.button.start) {
        if (startButtonTime == 0) {
            startButtonTime = millis();
        }
        startButtonPressed = true;
    } else {
        startButtonPressed = false;
        startButtonTime = 0;
    }
}void setupController() {
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


void handleShutdown() {
    Motors::setBoom(0);
    Motors::setDipper(0);
    Motors::setBucket(0);
    Motors::setThumb(0);
    Motors::setRotator(0);
    Motors::setLeftTrack(0);
    Motors::setRightTrack(0);
    Motors::setPusher(0);
    
    Lights::setBoomLight(false);
    delay(1000);
    Lights::setCabLight(false);
    delay(1000);
    
    Audio::enginePowerDown();
    Audio::stopEngine();
    Beacon::setEnabled(false);
    
    state = State::OFF;
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
    
    // State machine handling
    switch(state) {
        case State::OFF:
            if (startButtonPressed && (millis() - startButtonTime >= 100)) {
                Audio::startEngine();
                Beacon::setEnabled(true);
                state = State::IDLE;
            }
            break;
            
        case State::IDLE:
        case State::POWER:
        case State::HYDRAULIC:
        case State::MOVING:
        case State::REVERSE:
            if (startButtonPressed && (millis() - startButtonTime >= 2000)) {
                handleShutdown();
            }
            break;
            
        case State::POWER_UP:
            if (!Audio::isPlaying(Audio::SOUND_POWERUP)) {
                state = State::MOVING;
                Audio::engineHydraulic();
            }
            break;
    }
    // Rest of control loop code
}

