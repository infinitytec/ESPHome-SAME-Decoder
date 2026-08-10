// my_components/same_decoder/same_decoder.cpp
#include "same_decoder.h"
#include "esphome/core/log.h"
#include <cmath>
#include <cstring>

namespace esphome {
namespace same_decoder {

static const char *const TAG = "same_decoder";

// SAME physical-layer constants (EIA/NWS SAME spec)
static const float BAUD = 520.833f;
static const float MARK_HZ = 2083.3f;   // logical 1
static const float SPACE_HZ = 1562.5f;  // logical 0

void SAMEDecoder::setup() {
  ESP_LOGCONFIG(TAG, "Setting up SAME decoder @ %d Hz, DIN GPIO%d", this->sample_rate_, this->din_pin_);
  if (!this->start_i2s_()) {
    ESP_LOGE(TAG, "I2S init failed; decoder disabled.");
    this->mark_failed();
    return;
  }
  // Idle state for entities
  if (this->alert_active_) this->alert_active_->publish_state(false);
}

void SAMEDecoder::loop() {
  // -----------------------------------------------------------------------
  // High-level pipeline (runs continuously):
  //   1. read a chunk of PCM from I2S
  //   2. demodulate bits, hunt for preamble + "ZCZC"
  //   3. assemble the 3 redundant bursts, majority-vote them
  //   4. parse the header; if any area matches my FIPS -> publish + fire event
  //
  // NOTE: The read/demod inner loop is intentionally a skeleton. Locking AFSK
  // on live audio is the hard, iterative part and needs tuning against real
  // captured NWR samples (see project notes). Structure is complete; the
  // per-bit tone discrimination and clock recovery need real-world hardening.
  // -----------------------------------------------------------------------
  static int16_t buf;
  int n = this->read_samples_(buf, 512);
  if (n <= 0) return;

  if (!this->find_preamble_()) return;

  std::string burst;
  if (!this->assemble_burst_(burst)) return;

  SAMEMessage msg;
  if (!this->parse_header_(burst, msg)) {
    ESP_LOGW(TAG, "Header parse failed: %s", burst.c_str());
    return;
  }

  bool mine = this->matches_my_fips_(msg);
  this->publish_(msg, mine);

  if (mine) {
    for (auto *t : this->alert_triggers_) t->trigger();
  }
}

// ---- I2S bring-up -----------------------------------------------------------
bool SAMEDecoder::start_i2s_() {
  // TODO: configure the shared i2s_audio parent for RX at sample_rate_, mono,
  // 16-bit, using din_pin_. On esp-idf this installs an I2S RX channel that
  // reads the ES8388's ADC output. Left as an integration point because the
  // exact API depends on your installed ESPHome version's i2s_audio internals.
  ESP_LOGCONFIG(TAG, "I2S RX configured (mono/16-bit/%dHz).", this->sample_rate_);
  return true;
}

int SAMEDecoder::read_samples_(int16_t *buf, int max) {
  // TODO: i2s_channel_read(...) into buf; return sample count.
  (void) buf; (void) max;
  return 0;  // skeleton
}

// ---- DSP --------------------------------------------------------------------
bool SAMEDecoder::demodulate_bit_(const int16_t *s, int n, bool &bit) {
  // Goertzel energy at MARK vs SPACE over one bit period (sample_rate_/BAUD samples).
  auto goertzel = [&](float freq) -> float {
    float w = 2.0f * M_PI * freq / this->sample_rate_;
    float c = 2.0f * cosf(w), q0, q1 = 0, q2 = 0;
    for (int i = 0; i < n; i++) { q0 = c * q1 - q2 + s[i]; q2 = q1; q1 = q0; }
    return q1 * q1 + q2 * q2 - q1 * q2 * c;
  };
  bit = goertzel(MARK_HZ) > goertzel(SPACE_HZ);
  return true;
}

// ---- Framing (skeletons) ----------------------------------------------------
bool SAMEDecoder::find_preamble_() { return false; }               // TODO
bool SAMEDecoder::assemble_burst_(std::string &out) { (void)out; return false; }  // TODO
bool SAMEDecoder::parse_header_(const std::string &b, SAMEMessage &m) {
  // Expected: ZCZC-ORG-EEE-PSSCCC-PSSCCC...+TTTT-JJJHHMM-LLLLLLLL-
  if (b.rfind("ZCZC-", 0) != 0) return false;
  m.raw = b;
  // TODO: tokenize on '-' and '+' into originator/event/areas/purge/issued/sender.
  return true;
}

// ---- Matching + CAP mapping -------------------------------------------------
bool SAMEDecoder::matches_my_fips_(const SAMEMessage &msg) {
  if (this->fips_source_ == nullptr) return false;
  std::string mine = this->fips_source_->state;   // "020091,020173"
  for (const auto &area : msg.areas) {
    // SAME area is PSSCCC; the leading P is a subdivision digit. Match on the
    // trailing 5 (SSCCC) or the full 6 depending on how the operator enters them.
    if (mine.find(area) != std::string::npos) return true;
    if (area.size() == 6 && mine.find(area.substr(1)) != std::string::npos) return true;
  }
  return false;
}

std::string SAMEDecoder::severity_for_(const std::string &e) {
  // Map SAME event codes to cap_alerts' canonical CAP tiers.
  if (e == "TOR" || e == "TSW" || e == "EWW" || e == "EQW") return "extreme";
  if (e == "SVR" || e == "FFW" || e == "HUW" || e == "TOA") return "severe";
  if (e == "SVA" || e == "FFA" || e == "FLW" || e == "WSW") return "moderate";
  if (e == "RWT" || e == "RMT" || e == "SPS")               return "minor";
  return "unknown";
}

std::string SAMEDecoder::describe_(const std::string &e) {
  if (e == "TOR") return "Tornado Warning";
  if (e == "SVR") return "Severe Thunderstorm Warning";
  if (e == "FFW") return "Flash Flood Warning";
  if (e == "RWT") return "Required Weekly Test";
  if (e == "RMT") return "Required Monthly Test";
  return e;  // fall back to the raw code
}

void SAMEDecoder::publish_(const SAMEMessage &msg, bool mine) {
  std::string areas;
  for (size_t i = 0; i < msg.areas.size(); i++) {
    if (i) areas += ",";
    areas += msg.areas[i];
  }
  if (this->t_originator_) this->t_originator_->publish_state(msg.originator);
  if (this->t_event_code_) this->t_event_code_->publish_state(msg.event_code);
  if (this->t_event_)      this->t_event_->publish_state(this->describe_(msg.event_code));
  if (this->t_severity_)   this->t_severity_->publish_state(this->severity_for_(msg.event_code));
  if (this->t_areas_)      this->t_areas_->publish_state(areas);
  if (this->t_sender_)     this->t_sender_->publish_state(msg.sender);
  if (this->t_raw_)        this->t_raw_->publish_state(msg.raw);
  if (this->alert_active_) this->alert_active_->publish_state(mine);
  // TODO: convert msg.purge (+TTTT) to minutes and publish s_expires_.
  ESP_LOGI(TAG, "SAME %s (%s) mine=%d areas=%s",
           msg.event_code.c_str(), this->severity_for_(msg.event_code).c_str(),
           mine, areas.c_str());
}

}  // namespace same_decoder
}  // namespace esphome
