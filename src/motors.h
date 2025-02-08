
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
}
