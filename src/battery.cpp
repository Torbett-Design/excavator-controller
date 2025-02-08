#include "battery.h"
#include "pins.h"

// Battery monitoring constants
#define BATTERY_CHECK_INTERVAL_MS 10000
#define BATTERY_LOW_THRESHOLD 3500
#define ADC_VOLTAGE_DIVIDER 2.625

namespace Battery {

    volatile bool lowBatteryState = false;

    void setup() {
        pinMode(Pins::VBATT_ADC_EN, OUTPUT);
        pinMode(Pins::VBATT_ADC, INPUT);
        analogReadResolution(12);
    }

    uint16_t readVoltage() {
        digitalWrite(Pins::VBATT_ADC_EN, HIGH);
        delay(10);
        uint16_t adcValue = analogRead(Pins::VBATT_ADC);
        digitalWrite(Pins::VBATT_ADC_EN, LOW);
        
        return (uint16_t)(adcValue * (3300.0 / 4095.0) * ADC_VOLTAGE_DIVIDER);
    }

    bool isLowBattery() {
        return readVoltage() < BATTERY_LOW_THRESHOLD;
    }

}
