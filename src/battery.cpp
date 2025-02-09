#include "battery.h"
#include "pins.h"
#include "audio.h"
#include <ESP32AnalogRead.h>

// Battery monitoring constants
#define BATTERY_CHECK_INTERVAL_MS 10000
#define BATTERY_LOW_THRESHOLD 3500
#define ADC_VOLTAGE_DIVIDER 2.625

namespace Battery {

    volatile bool lowBatteryState = false;
    static unsigned long lastBatteryCheck = 0;
    static ESP32AnalogRead adc;


    #define BATTERY_TASK_STACK_SIZE 2048
    static TaskHandle_t batteryTaskHandle = NULL;

    static void batteryTask(void* parameter) {
        while(true) {
            if (isLowBattery() && !lowBatteryState) {
                lowBatteryState = true;
                Audio::playLowBatteryWarning();
            } else if (!isLowBattery()) {
                lowBatteryState = false;
            }
            vTaskDelay(pdMS_TO_TICKS(BATTERY_CHECK_INTERVAL_MS));
        }
    }


      void setup() {
          pinMode(Pins::VBATT_ADC_EN, OUTPUT);
          adc.attach(Pins::VBATT_ADC);
        
          xTaskCreate(
              batteryTask,
              "battery_task",
              BATTERY_TASK_STACK_SIZE,
              NULL,
              1,
              &batteryTaskHandle
          );
      }

      uint16_t readVoltage() {
          digitalWrite(Pins::VBATT_ADC_EN, HIGH);
          delay(10);
          uint16_t voltage = (uint16_t)(adc.readVoltage() * ADC_VOLTAGE_DIVIDER);
          digitalWrite(Pins::VBATT_ADC_EN, LOW);
        
          return voltage;
      }
    

    bool isLowBattery() {
        return readVoltage() < BATTERY_LOW_THRESHOLD;
    }

}
