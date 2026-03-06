#pragma once

// Runtime auto-detect for SX1262 / SX1268 / LLCC68 without changing global src/.
// This class owns one instance of each candidate and selects one at boot via std_init().

#include <Arduino.h>
#include <RadioLib.h>

// Use existing global custom radio drivers (do not modify them).
#include <helpers/radiolib/CustomSX1262.h>
#include <helpers/radiolib/CustomSX1268.h>
#include <helpers/radiolib/CustomLLCC68.h>

class AutoSX126xRadio {
public:
  enum class Kind : uint8_t {
    None = 0,
    SX1268,
    LLCC68,
    SX1262,
  };

  // Intentionally NOT explicit: existing targets instantiate radios as
  // `RADIO_CLASS radio = new Module(...);`
  AutoSX126xRadio(Module* mod)
    : _mod(mod),
      _sx1262(mod),
      _sx1268(mod),
      _llcc68(mod),
      _active(nullptr),
      _kind(Kind::None) {
  }

  // NOTE: keep signature compatible with existing target.cpp usage.
  bool std_init(SPIClass* spi = NULL) {
    // Try order: SX1268 -> LLCC68 -> SX1262
    // Rationale: if a board is sold in 433/470MHz and 868/915MHz variants,
    // trying SX1268 first tends to match those builds. LLCC68 and SX1262 are very compatible.
    if (_sx1268.std_init(spi)) {
      _active = &_sx1268;
      _kind = Kind::SX1268;
      return true;
    }
    if (_llcc68.std_init(spi)) {
      _active = &_llcc68;
      _kind = Kind::LLCC68;
      return true;
    }
    if (_sx1262.std_init(spi)) {
      _active = &_sx1262;
      _kind = Kind::SX1262;
      return true;
    }
    _active = nullptr;
    _kind = Kind::None;
    return false;
  }

  Kind kind() const { return _kind; }
  bool ready() const { return _active != nullptr; }

  // Expose selected RadioLib layer for wrappers / RNG.
  PhysicalLayer& phy() {
    // Caller must ensure std_init succeeded.
    return *_active;
  }
  const PhysicalLayer& phy() const {
    return *_active;
  }

  // Helpers used by variant target.cpp
  uint32_t random(uint32_t max) {
    if (!_active) return 0;
    return _active->random(max);
  }

  int16_t setFrequency(float freq) {
    if (!_active) return RADIOLIB_ERR_CHIP_NOT_FOUND;
    return _active->setFrequency(freq);
  }

  int16_t setSpreadingFactor(uint8_t sf) {
    if (!_active) return RADIOLIB_ERR_CHIP_NOT_FOUND;
    // SX1262/SX1268/LLCC68 all inherit SX126x.
    return ((SX126x*)_active)->setSpreadingFactor(sf);
  }

  int16_t setBandwidth(float bw) {
    if (!_active) return RADIOLIB_ERR_CHIP_NOT_FOUND;
    return ((SX126x*)_active)->setBandwidth(bw);
  }

  int16_t setCodingRate(uint8_t cr) {
    if (!_active) return RADIOLIB_ERR_CHIP_NOT_FOUND;
    return ((SX126x*)_active)->setCodingRate(cr);
  }

  int16_t setOutputPower(int8_t dbm) {
    if (!_active) return RADIOLIB_ERR_CHIP_NOT_FOUND;
    return _active->setOutputPower(dbm);
  }

  // Used by wrapper for better packetScore and LBT.
  uint8_t getSpreadingFactor() const {
    if (!_active) return 10;
    return ((SX126x*)_active)->spreadingFactor;
  }

  // RSSI helpers
  // NOTE: For SX126x, PhysicalLayer::getRSSI() returns the *last packet* RSSI
  // (it calls SX126x::getRSSI(true)). For noise-floor / LBT we need the
  // *instantaneous* RSSI, which is SX126x::getRSSI(false).
  float getRSSIInst() const {
    if (!_active) return -200.0f;
    return ((SX126x*)_active)->getRSSI(false);
  }

  float getRSSIPacket() const {
    if (!_active) return -200.0f;
    return _active->getRSSI();
  }

  bool isReceivingPacket() const {
    if (!_active) return false;
    switch (_kind) {
      case Kind::SX1268:
        return ((CustomSX1268*)_active)->isReceiving();
      case Kind::LLCC68:
        return ((CustomLLCC68*)_active)->isReceiving();
      case Kind::SX1262:
        return ((CustomSX1262*)_active)->isReceiving();
      default:
        return false;
    }
  }

private:
  Module* _mod;
  CustomSX1262 _sx1262;
  CustomSX1268 _sx1268;
  CustomLLCC68 _llcc68;

  PhysicalLayer* _active;
  Kind _kind;
};
