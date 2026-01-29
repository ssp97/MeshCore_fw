
#define RADIOLIB_STATIC_ONLY 1

#include "NamijiAutoRadioWrapper.h"

// State machine values (same as original RadioLibWrapper)
#define STATE_IDLE       0
#define STATE_RX         1
#define STATE_TX_WAIT    3
#define STATE_INT_READY 16

#define NUM_NOISE_FLOOR_SAMPLES  64
#define SAMPLING_THRESHOLD       14

volatile uint8_t NamijiAutoRadioWrapper::s_state = STATE_IDLE;

static
#if defined(ESP8266) || defined(ESP32)
  ICACHE_RAM_ATTR
#endif
void namijiSetFlag(void) {
  NamijiAutoRadioWrapper::onDioAction();
}

void NamijiAutoRadioWrapper::onDioAction() {
  s_state |= STATE_INT_READY;
}

void NamijiAutoRadioWrapper::begin() {
  // Resolve the active radio selected by AutoSX126xRadio::std_init().
  if (!_auto || !_auto->ready()) {
    // Not initialized yet; leave in idle.
    _radio = nullptr;
    s_state = STATE_IDLE;
    return;
  }

  _radio = &_auto->phy();

  // This ISR is used for both RX complete and TX complete by RadioLib.
  _radio->setPacketReceivedAction(namijiSetFlag);
  s_state = STATE_IDLE;

  // Handle wake-from-sleep RX case.
  if (_board && _board->getStartupReason() == BD_STARTUP_RX_PACKET) {
    onDioAction();
  }

  _noise_floor = 0;
  _threshold = 0;
  _num_floor_samples = 0;
  _floor_sample_sum = 0;
}

void NamijiAutoRadioWrapper::idle() {
  if (_radio) {
    _radio->standby();
  }
  s_state = STATE_IDLE;
}

void NamijiAutoRadioWrapper::triggerNoiseFloorCalibrate(int threshold) {
  _threshold = threshold;
  if (_num_floor_samples >= NUM_NOISE_FLOOR_SAMPLES) {
    _num_floor_samples = 0;
    _floor_sample_sum = 0;
  }
}

void NamijiAutoRadioWrapper::resetAGC() {
  // don't reset while packet event pending or while mid-receive
  if ((s_state & STATE_INT_READY) != 0) return;
  if (_auto && _auto->isReceivingPacket()) return;
  s_state = STATE_IDLE;  // trigger startReceive()
}

void NamijiAutoRadioWrapper::loop() {
  if (!_radio) return;

  if (s_state == STATE_RX && _num_floor_samples < NUM_NOISE_FLOOR_SAMPLES) {
    if (!(_auto && _auto->isReceivingPacket())) {
      int rssi = (int)getCurrentRSSI();
      if (rssi < _noise_floor + SAMPLING_THRESHOLD) {
        _num_floor_samples++;
        _floor_sample_sum += rssi;
      }
    }
  } else if (_num_floor_samples >= NUM_NOISE_FLOOR_SAMPLES && _floor_sample_sum != 0) {
    _noise_floor = _floor_sample_sum / NUM_NOISE_FLOOR_SAMPLES;
    if (_noise_floor < -120) {
      _noise_floor = -120;
    }
    _floor_sample_sum = 0;

    #if MESH_DEBUG && ARDUINO
      Serial.printf("Radio: noise_floor = %d\n", (int)_noise_floor);
    #endif
  }
}

void NamijiAutoRadioWrapper::startRecv() {
  if (!_radio) return;
  int err = _radio->startReceive();
  if (err == RADIOLIB_ERR_NONE) {
    s_state = STATE_RX;
  } else {
    #if MESH_DEBUG && ARDUINO
      Serial.printf("Radio: error: startReceive(%d)\n", err);
    #endif
  }
}

bool NamijiAutoRadioWrapper::isInRecvMode() const {
  return (s_state & ~STATE_INT_READY) == STATE_RX;
}

int NamijiAutoRadioWrapper::recvRaw(uint8_t* bytes, int sz) {
  if (!_radio) return 0;

  int len = 0;
  if (s_state & STATE_INT_READY) {
    len = _radio->getPacketLength();
    if (len > 0) {
      if (len > sz) len = sz;
      int err = _radio->readData(bytes, len);
      if (err != RADIOLIB_ERR_NONE) {
        #if MESH_DEBUG && ARDUINO
          Serial.printf("Radio: error: readData(%d)\n", err);
        #endif
        len = 0;
      } else {
        n_recv++;
      }
    }
    s_state = STATE_IDLE;
  }

  if (s_state != STATE_RX) {
    startRecv();
  }
  return len;
}

uint32_t NamijiAutoRadioWrapper::getEstAirtimeFor(int len_bytes) {
  if (!_radio) return 0;
  return _radio->getTimeOnAir(len_bytes) / 1000;
}

bool NamijiAutoRadioWrapper::startSendRaw(const uint8_t* bytes, int len) {
  if (!_radio) return false;
  if (_board) _board->onBeforeTransmit();

  int err = _radio->startTransmit((uint8_t*)bytes, len);
  if (err == RADIOLIB_ERR_NONE) {
    s_state = STATE_TX_WAIT;
    return true;
  }

  #if MESH_DEBUG && ARDUINO
    Serial.printf("Radio: error: startTransmit(%d)\n", err);
  #endif
  idle();
  if (_board) _board->onAfterTransmit();
  return false;
}

bool NamijiAutoRadioWrapper::isSendComplete() {
  if (s_state & STATE_INT_READY) {
    s_state = STATE_IDLE;
    n_sent++;
    return true;
  }
  return false;
}

void NamijiAutoRadioWrapper::onSendFinished() {
  if (!_radio) return;
  _radio->finishTransmit();
  if (_board) _board->onAfterTransmit();
  s_state = STATE_IDLE;
}

float NamijiAutoRadioWrapper::getLastRSSI() const {
  // Packet RSSI (last received packet). This matches existing stats usage.
  return (_auto && _auto->ready()) ? _auto->getRSSIPacket() : (_radio ? _radio->getRSSI() : 0);
}

float NamijiAutoRadioWrapper::getLastSNR() const {
  return _radio ? _radio->getSNR() : 0;
}

float NamijiAutoRadioWrapper::getCurrentRSSI() {
  if (!_radio) return -200;

  // IMPORTANT: For SX126x, getRSSI() with no args returns *last packet RSSI*
  // (SX126x::getRSSI(true)). For noise-floor calibration / LBT we need the
  // *instantaneous* RSSI (SX126x::getRSSI(false)). Using packet RSSI here can
  // yield bogus values like ~0 dBm when no packet has been received.
  if (_auto && _auto->ready()) {
    return _auto->getRSSIInst();
  }

  // Fallback (should not happen for this variant).
  return _radio->getRSSI();
}

bool NamijiAutoRadioWrapper::isChannelActive() {
  if (_threshold == 0) return false;
  return (int)getCurrentRSSI() > _noise_floor + _threshold;
}

bool NamijiAutoRadioWrapper::isReceiving() {
  if (_auto && _auto->isReceivingPacket()) return true;
  return isChannelActive();
}

// Approximate SNR threshold per SF for successful reception (based on Semtech datasheets)
static float snr_threshold[] = {
  -7.5,  // SF7
  -10,   // SF8
  -12.5, // SF9
  -15,   // SF10
  -17.5, // SF11
  -20    // SF12
};

float NamijiAutoRadioWrapper::packetScoreInt(float snr, int sf, int packet_len) {
  if (sf < 7) return 0.0f;
  if (sf > 12) sf = 12;

  if (snr < snr_threshold[sf - 7]) return 0.0f;

  float success_rate_based_on_snr = (snr - snr_threshold[sf - 7]) / 10.0f;
  float collision_penalty = 1.0f - (packet_len / 256.0f);

  float score = success_rate_based_on_snr * collision_penalty;
  if (score < 0.0f) score = 0.0f;
  if (score > 1.0f) score = 1.0f;
  return score;
}

float NamijiAutoRadioWrapper::packetScore(float snr, int packet_len) {
  int sf = 10;
  if (_auto && _auto->ready()) {
    sf = _auto->getSpreadingFactor();
  }
  return packetScoreInt(snr, sf, packet_len);
}

