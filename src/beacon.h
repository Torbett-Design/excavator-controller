#pragma once
#include <Arduino.h>

/** @brief Beacon control interface */
namespace Beacon {
    void setup();
    void setEnabled(bool enabled);
    void update();
    void setLowBatteryMode(bool enabled);
    void startTask();
}
