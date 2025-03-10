#include "beacon.h"
#include <Adafruit_NeoPixel.h>
#include "pins.h"

#define BEACON_LED_COUNT 1

#define BEACON_TASK_STACK_SIZE 2048

// Orange construction beacon color
#define BEACON_COLOR beacon.Color(255, 140, 0)
#define BEACON_FLASH_INTERVAL 80  // Flash rate in ms
#define BATTERY_RED_COLOR beacon.Color(255, 0, 0)
#define BATTERY_LOW_FLASH_INTERVAL 500

namespace Beacon {
    static Adafruit_NeoPixel beacon(BEACON_LED_COUNT, Pins::BEACON, NEO_GRB + NEO_KHZ800);
    static bool beaconEnabled = false;
    static bool beaconState = false;
    static unsigned long lastBeaconUpdate = 0;
    static bool lowBatteryMode = false;
    // Add to beacon.cpp

    static TaskHandle_t beaconTaskHandle = NULL;

    static void beaconTask(void* parameter) {
    while(true) {
        update();
        vTaskDelay(pdMS_TO_TICKS(10)); // 10ms yield
    }
    }

void startTask() {
    xTaskCreate(
        beaconTask,
        "beacon_task",
        BEACON_TASK_STACK_SIZE,
        NULL,
        1,
        &beaconTaskHandle
    );
}

    void setup() {
        beacon.begin();
        beacon.clear();
        beacon.show();
        
        xTaskCreate(
            beaconTask,
            "beacon_task",
            BEACON_TASK_STACK_SIZE,
            NULL,
            1,
            &beaconTaskHandle
        );
    }

    void setEnabled(bool enabled) {
        beaconEnabled = enabled;
        if (!enabled) {
            beacon.clear();
            beacon.show();
        }
    }

    void setLowBatteryMode(bool enabled) {
        lowBatteryMode = enabled;
    }

    void update() {
        if (!beaconEnabled) return;
        
        unsigned long currentMillis = millis();
        
        if (lowBatteryMode) {
            // Slow red flashing for low battery
            if (currentMillis - lastBeaconUpdate >= BATTERY_LOW_FLASH_INTERVAL) {
                beaconState = !beaconState;
                beacon.setPixelColor(0, beaconState ? BATTERY_RED_COLOR : 0);
                beacon.show();
                lastBeaconUpdate = currentMillis;
            }
        } else {
            // Normal orange flashing
            if (currentMillis - lastBeaconUpdate >= BEACON_FLASH_INTERVAL) {
                beaconState = !beaconState;
                beacon.setPixelColor(0, beaconState ? BEACON_COLOR : 0);
                beacon.show();
                lastBeaconUpdate = currentMillis;
            }
        }
    }

    bool isEnabled() {
        return beaconEnabled;
    }
}


