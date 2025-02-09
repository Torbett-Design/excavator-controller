#include "motors.h"
#include "pins.h"
#include <PCA9635.h>
#include <Wire.h>

namespace Motors {
    // PCA9635 instance with all-call address
    static PCA9635 motorDriver(0x00);

    // Add at the top with other static variables
    static int16_t currentSpeeds[8] = {0}; // Array to store current speeds

    // Motor channel definitions
    enum MotorChannel {
        BOOM = 0,      // Motor 1 (channels 0,1)
        DIPPER = 2,    // Motor 2 (channels 2,3)
        BUCKET = 4,    // Motor 3 (channels 4,5)
        THUMB = 6,     // Motor 4 (channels 6,7)
        ROTATOR = 8,   // Motor 5 (channels 8,9)
        LEFT_TRACK = 10,  // Motor 6 (channels 10,11)
        RIGHT_TRACK = 12, // Motor 7 (channels 12,13)
        PUSHER = 14    // Motor 8 (channels 14,15)
    };

    static void setMotor(MotorChannel motor, int16_t speed) {
        currentSpeeds[motor/2] = speed; // Store speed before applying it
        uint8_t channelA = motor;
        uint8_t channelB = motor + 1;
        
        speed = constrain(speed, -255, 255);
        
        if (speed == 0) {
            motorDriver.write1(channelA, 0);
            motorDriver.write1(channelB, 0);
        }
        else if (speed > 0) {
            motorDriver.write1(channelA, 0);
            motorDriver.write1(channelB, speed);
        }
        else {
            motorDriver.write1(channelA, -speed);
            motorDriver.write1(channelB, 0);
        }
    }

    void setup() {
        Wire.begin(Pins::I2C_SDA, Pins::I2C_SCL);
        
        motorDriver.begin(0x00);
        motorDriver.setMode1(0);
        motorDriver.setMode2(0x04);  // Enable output change on ACK

        // Initialize all motor channels to OFF
        for (int i = 0; i < 16; i++) {
            motorDriver.write1(i, 0);
        }

        // Enable motors
        pinMode(Pins::MOTORS_EN, OUTPUT);
        digitalWrite(Pins::MOTORS_EN, HIGH);
    }

    void setBoom(int16_t speed) {
        setMotor(BOOM, speed);
    }

    void setDipper(int16_t speed) {
        setMotor(DIPPER, speed);
    }

    void setBucket(int16_t speed) {
        setMotor(BUCKET, speed);
    }

    void setThumb(int16_t speed) {
        setMotor(THUMB, speed);
    }

    void setRotator(int16_t speed) {
        setMotor(ROTATOR, speed);
    }

    void setLeftTrack(int16_t speed) {
        setMotor(LEFT_TRACK, speed);
    }

    void setRightTrack(int16_t speed) {
        setMotor(RIGHT_TRACK, speed);
    }

    void setPusher(int16_t speed) {
        setMotor(PUSHER, speed);
    }

    // Add getter implementations
    int16_t getBoom() { return currentSpeeds[BOOM/2]; }
    int16_t getDipper() { return currentSpeeds[DIPPER/2]; }
    int16_t getBucket() { return currentSpeeds[BUCKET/2]; }
    int16_t getThumb() { return currentSpeeds[THUMB/2]; }
    int16_t getRotator() { return currentSpeeds[ROTATOR/2]; }
    int16_t getLeftTrack() { return currentSpeeds[LEFT_TRACK/2]; }
    int16_t getRightTrack() { return currentSpeeds[RIGHT_TRACK/2]; }
    int16_t getPusher() { return currentSpeeds[PUSHER/2]; }
    bool areTracksMoving() {
        return getLeftTrack() != 0 || getRightTrack() != 0;
    }
}