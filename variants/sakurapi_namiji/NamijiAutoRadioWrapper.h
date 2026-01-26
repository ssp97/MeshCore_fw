#pragma once

// Local RadioLib wrapper for sakurapi_namiji variant.
// Copied and adapted from src/helpers/radiolib/RadioLibWrappers.* to avoid modifying global src/.

#include <Arduino.h>
#include <Mesh.h>
#include <RadioLib.h>

#include "AutoSX126xRadio.h"

class NamijiAutoRadioWrapper : public mesh::Radio {
public:
  NamijiAutoRadioWrapper(AutoSX126xRadio& autoRadio, mesh::MainBoard& board)
    : _auto(&autoRadio), _radio(nullptr), _board(&board) {
    n_recv = n_sent = 0;
    _noise_floor = 0;
    _threshold = 0;
    _num_floor_samples = 0;
    _floor_sample_sum = 0;
  }

  void begin() override;

  // Stats helpers (expected by some example code)
  uint32_t getPacketsRecv() const { return n_recv; }
  uint32_t getPacketsSent() const { return n_sent; }
  void resetStats() { n_recv = n_sent = 0; }

  int recvRaw(uint8_t* bytes, int sz) override;
  uint32_t getEstAirtimeFor(int len_bytes) override;
  float packetScore(float snr, int packet_len) override;

  bool startSendRaw(const uint8_t* bytes, int len) override;
  bool isSendComplete() override;
  void onSendFinished() override;

  bool isInRecvMode() const override;
  bool isReceiving() override;

  int getNoiseFloor() const override { return _noise_floor; }
  void triggerNoiseFloorCalibrate(int threshold) override;
  void resetAGC() override;
  void loop() override;

  float getLastRSSI() const override;
  float getLastSNR() const override;

private:
  // Internal state flags (mirrors original RadioLibWrapper logic)
  static volatile uint8_t s_state;
public:
  // ISR hook (must be public so C-style ISR shim can call it)
  static void onDioAction();

private:

  void idle();
  void startRecv();
  bool isChannelActive();
  float getCurrentRSSI();
  float packetScoreInt(float snr, int sf, int packet_len);

private:
  AutoSX126xRadio* _auto;
  PhysicalLayer* _radio;
  mesh::MainBoard* _board;

  uint32_t n_recv, n_sent;
  int16_t _noise_floor, _threshold;
  uint16_t _num_floor_samples;
  int32_t _floor_sample_sum;
};

