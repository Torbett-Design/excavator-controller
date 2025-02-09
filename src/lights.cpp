#include "lights.h"
#include "pins.h"
#include "driver/ledc.h"


namespace Lights {
    // Light control definitions
    #define PWM_FREQUENCY 5000
    #define PWM_RESOLUTION 8  // 8-bit resolution (0-255)

    // PWM channels for lights
    #define CAB_LIGHT_CHANNEL 0
    #define BOOM_LIGHT_CHANNEL 1
    #define AUX_LIGHT_CHANNEL 2

    static TaskHandle_t lightTaskHandle = NULL;
    static uint8_t targetBrightness[3] = {0, 0, 0}; // Cab, Boom, Aux
    static uint8_t currentBrightness[3] = {0, 0, 0};
    const unsigned long FADE_DURATION = 1000; // 1 second fade
    const unsigned long FADE_INTERVAL = 20; // Update every 20ms (50Hz)
    
    static void lightTask(void* parameter) {
        while(true) {
            for (int i = 0; i < 3; i++) {
                if (currentBrightness[i] != targetBrightness[i]) {
                    int step = (FADE_INTERVAL * 255) / FADE_DURATION;
                    if (currentBrightness[i] < targetBrightness[i]) {
                        currentBrightness[i] = min(currentBrightness[i] + step, targetBrightness[i]);
                    } else {
                        currentBrightness[i] = max(currentBrightness[i] - step, targetBrightness[i]);
                    }
                    
                    switch(i) {
                        case 0: analogWrite(Pins::CAB_LED, currentBrightness[i]); break;
                        case 1: analogWrite(Pins::BOOM_LED, currentBrightness[i]); break;
                        case 2: analogWrite(Pins::AUX_LED, currentBrightness[i]); break;
                    }
                }
            }
            vTaskDelay(pdMS_TO_TICKS(FADE_INTERVAL));
        }
    }

    void setup() {
        xTaskCreate(
            lightTask,
            "light_task",
            2048,  // Stack size
            NULL,
            1,     // Priority
            &lightTaskHandle
        );
    }

    void setCabLight(uint8_t brightness) {
        ledcWrite(CAB_LIGHT_CHANNEL, brightness);
    }

    void setBoomLight(uint8_t brightness) {
        ledcWrite(BOOM_LIGHT_CHANNEL, brightness);
    }

    void setAuxLight(uint8_t brightness) {
        ledcWrite(AUX_LIGHT_CHANNEL, brightness);
    }

    void allLightsOn(uint8_t brightness) {
        setCabLight(brightness);
        setBoomLight(brightness);
        setAuxLight(brightness);
    }

    void allLightsOff() {
        setCabLight(0);
        setBoomLight(0);
        setAuxLight(0);
    }
}
