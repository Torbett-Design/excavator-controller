  #pragma once
  #include <Arduino.h>

  /** @brief Pin definitions for all hardware interfaces */
  namespace Pins {
      // GPIO assignments
      constexpr uint8_t BATT_STAT = 2;
      constexpr uint8_t BEACON = 5;
      constexpr uint8_t VBATT_ADC_EN = 12;
      constexpr uint8_t VBATT_ADC = 15;
      constexpr uint8_t SERVO_3 = 4;
      constexpr uint8_t SERVO_2 = 16;
      constexpr uint8_t SERVO_1 = 17;
      constexpr uint8_t I2S_BCLK = 14;
      constexpr uint8_t I2S_LRCLK = 27;
      constexpr uint8_t I2S_DOUT = 26;
      constexpr uint8_t I2C_SDA = 33;
      constexpr uint8_t I2C_SCL = 32;
      constexpr uint8_t OE = 35;
      constexpr uint8_t AUX_LED = 21;
      constexpr uint8_t CAB_LED = 19;
      constexpr uint8_t BOOM_LED = 18;
      constexpr uint8_t V5_EN = 22;
      constexpr uint8_t MOTORS_EN = 23;
  }
