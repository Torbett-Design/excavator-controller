
#pragma once
#include <Arduino.h>

/** @brief Light control interface */
namespace Lights {
    void setup();
    void setCabLight(uint8_t brightness);
    void setBoomLight(uint8_t brightness);
    void setAuxLight(uint8_t brightness);
    void allLightsOn(uint8_t brightness = 255);
    void allLightsOff();
}
