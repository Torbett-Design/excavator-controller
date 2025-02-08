#pragma once
#include <Arduino.h>

/** @brief Audio system control interface */
namespace Audio {
    void setup();
    void startEngine();
    void enginePowerUp();
    void engineHydraulic();
    void enginePowerDown();
    void stopEngine();
    void playLowBatteryWarning();
    void playWAVAsync(const char* filename, bool loop = false);
    void stopAudio();
}
