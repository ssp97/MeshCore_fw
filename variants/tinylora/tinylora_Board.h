#pragma once

#include <MeshCore.h>
#include <Arduino.h>

#if defined(ESP_PLATFORM)

#include <helpers/ESP32Board.h>

class tinylora_Board : public ESP32Board {
  uint32_t gpio_state = 0;

public:
  void begin() {
    ESP32Board::begin();
  }

  uint32_t getGpio() override {
    return gpio_state;
  }

  void setGpio(uint32_t values) override {
    gpio_state = values;
  }

  uint16_t getBattMilliVolts() override {
#ifdef BATTERY_PIN
    analogReadResolution(12);         // ESP32-C3 ADC is 12-bit
    uint32_t raw = 0;
    for (int i = 0; i < 3; i++) {
      raw += analogRead(BATTERY_PIN);
    }
    raw = raw / 3;

    constexpr float ADC_REF_VOLT = 3.0f;     
    constexpr float ADC_MAX      = 4096.0f;  // 12-bit

#ifdef ADC_MULTIPLIER
    float v_adc = raw * ADC_REF_VOLT / ADC_MAX;
    float v_bat = v_adc * ADC_MULTIPLIER;
    return v_bat * 1000;
#else
    float v_adc = raw * ADC_REF_VOLT / ADC_MAX;
    return v_adc * 1000;
#endif

#else
    return 0;  // not supported
#endif
  }

  const char* getManufacturerName() const override {
    return "TinyLora";
  }
};

#endif
