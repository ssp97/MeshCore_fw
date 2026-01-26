#pragma once

#include <MeshCore.h>
#include <Arduino.h>

#if defined(ESP_PLATFORM)

#include <helpers/ESP32Board.h>

class sakurapi_namiji_Board : public ESP32Board {
  uint32_t gpio_state = 0;

public:
  void begin() {
    ESP32Board::begin();

  }
  uint32_t getGpio() override {

  }
  void setGpio(uint32_t values) override {

  }

  uint16_t getBattMilliVolts() override {
  #ifdef PIN_VBAT_READ
    analogReadResolution(12);         // ESP32-C3 ADC is 12-bit - 3.3/4096 (ref voltage/max counts)
    uint32_t raw = 0;
    for (int i = 0; i < 3; i++) {
      raw += analogRead(PIN_VBAT_READ);
    }
    raw = raw / 3;

    constexpr float ADC_REF_VOLT = 3.3f;     
    constexpr float ADC_MAX      = 4095.0f;  // 12-bit
    constexpr float R_UP   = 100000.0f;      // for sakurapi_namiji
    constexpr float R_DOWN = 22100.0f;       // for sakurapi_namiji

    float v_adc = raw * ADC_REF_VOLT / ADC_MAX;
    float v_bat = v_adc * (R_UP + R_DOWN) / R_DOWN;

    return v_bat * 1000;
  #else
    return 0;  // not supported
  #endif
  }

  const char* getManufacturerName() const override {
    return "SakuraPi Namiji";
  }
};

#endif
