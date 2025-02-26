#pragma once
#include <Arduino.h>

/** @brief Beacon control interface */
namespace Beacon {
    void setup();
    void setEnabled(bool enabled);
    void setLowBatteryMode(bool enabled);
    void update();
    
    // New function to get beacon state
    bool isEnabled();
}
