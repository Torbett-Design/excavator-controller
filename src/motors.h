
#pragma once
#include <Arduino.h>

/** @brief Motor control interface */
namespace Motors {
    void setup();
    void setBoom(int16_t speed);
    void setDipper(int16_t speed);
    void setBucket(int16_t speed);
    void setThumb(int16_t speed);
    void setRotator(int16_t speed);
    void setLeftTrack(int16_t speed);
    void setRightTrack(int16_t speed);
    void setPusher(int16_t speed);
    
    // Add getter functions
    int16_t getBoom();
    int16_t getDipper();
    int16_t getBucket();
    int16_t getThumb();
    int16_t getRotator();
    int16_t getLeftTrack();
    int16_t getRightTrack();
    int16_t getPusher();

    bool areTracksMoving();
}