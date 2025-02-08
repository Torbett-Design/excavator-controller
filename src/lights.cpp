#include "lights.h"
#include "pins.h"

namespace Lights {
    // Light control definitions
    #define PWM_FREQUENCY 5000
    #define PWM_RESOLUTION 8  // 8-bit resolution (0-255)

    // PWM channels for lights
    #define CAB_LIGHT_CHANNEL 0
    #define BOOM_LIGHT_CHANNEL 1
    #define AUX_LIGHT_CHANNEL 2

    void setup() {
        // Configure LED PWM channels
        ledcSetup(CAB_LIGHT_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
        ledcSetup(BOOM_LIGHT_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
        ledcSetup(AUX_LIGHT_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);

        // Attach PWM channels to GPIO pins using Pins namespace
        ledcAttachPin(Pins::CAB_LED, CAB_LIGHT_CHANNEL);
        ledcAttachPin(Pins::BOOM_LED, BOOM_LIGHT_CHANNEL);
        ledcAttachPin(Pins::AUX_LED, AUX_LIGHT_CHANNEL);
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
