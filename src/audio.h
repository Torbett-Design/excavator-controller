#pragma once
#include <Arduino.h>

/** @brief Audio system control interface */
namespace Audio {
    // Sound file paths
    static const char* SOUND_START = "/start.wav";
    static const char* SOUND_IDLE = "/idle.wav";
    static const char* SOUND_POWERUP = "/powerup.wav";
    static const char* SOUND_POWER = "/power.wav";
    static const char* SOUND_HYDRAULIC = "/hydraulic.wav";
    static const char* SOUND_POWERDOWN = "/powerdown.wav";
    static const char* SOUND_STOP = "/stop.wav";
    static const char* SOUND_LOW_BATTERY = "/low-batt.wav";
    static const char* SOUND_TRACK = "/track.wav";
    static const char* SOUND_REVERSE_BEEP = "/reverse.wav";

    enum class PlaybackMode {
        QUEUE,   // Play after current sound finishes
        REPLACE, // Stop current sound and play new sound
        MIX      // Mix with currently playing sound(s)
    };

    // Existing function declarations
    void setup();
    void startEngine();
    void enginePowerUp();
    void engineHydraulic();
    void enginePowerDown();
    void tracks();
    void stopEngine();
    void playLowBatteryWarning();
    void playWAVAsync(const char *filename, bool loop = false, PlaybackMode mode = PlaybackMode::REPLACE, uint8_t volume = 255);
    void stopAudio();
    bool isPlaying(const char* filename);
    void stopTrack(const char* filename);  // Stop specific audio track if playing

    
}
