#pragma once
#include <Arduino.h>

/** @brief Battery monitoring interface */
namespace Battery {
    void setup();
    uint16_t readVoltage();
    bool isLowBattery();
}
