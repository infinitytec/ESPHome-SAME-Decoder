// components/same_decoder/same_decoder.cpp
#include "same_decoder.h"
#include "same_event_codes.h"
#include "esphome/core/log.h"

#include <functional>   // std::hash
#include <sstream>
#include <iomanip>
#include <algorithm>    // std::max

namespace esphome {
namespace same_decoder {

static const char *const TAG = "same_decoder";

// ================= DSP: dual-Goertzel SAME AFSK demodulator =================
// SAME: 520.833 bps AFSK. Mark(1)=2083.33Hz, Space(0)=1562.5Hz.
// 48kHz -> 92.16 samples/bit.
// Framing: [0xAB x16 preamble] 'ZCZC' <header> (x3) 'NNNN'. Bytes are LSB-first.

static constexpr float SAMPLES_PER_BIT = 92.16f;      // 48000 / 520.8333
static constexpr float COEFF_MARK      = 1.926090f;   // 2*cos(2*pi*2083.33/48000)
static constexpr float COEFF_SPACE     = 1.958313f;   // 2*cos(2*pi*1562.5/48000)
static constexpr int   BIT_WINDOW      = 92;          // integer samples per Goertzel block

// Goertzel energy for one coefficient over `n` samples.
static float goertzel_energy(const int16_t *x, int n, float coeff) {
  float s0 = 0.f, s1 = 0.f, s2 = 0.f;
  for (int i = 0; i < n; i++) {
    s0 = (float) x[i] + coeff * s1 - s2;
    s2 = s1;
    s1 = s0;
  }
  return s1 * s1 + s2 * s2 - coeff * s1 * s2;
}

// Decide one bit from a window: mark energy vs space energy.
static bool decode_bit(const int16_t *win, int n) {
  float em = goertzel_energy(win, n, COEFF_MARK);
  float es = goertzel_energy(win, n, COEFF_SPACE);
  return em > es;   // true = 1 (mark), false = 0 (space)
}

// =============================== Lifecycle ===============================

void SAMEDecoder::setup() {
  ESP_LOGCONFIG(TAG, "Setting up SAME decoder (din=GPIO%u, %u Hz)...",
                this->din_pin_, this->sample_rate_);
  this->i2s_ready_ = this->start_i2s_();
  if (!this->i2s_ready_) {
    ESP_LOGW(TAG, "I2S RX not configured yet. Decoder idle.");
  }
}

void SAMEDecoder::dump_config() {
  ESP_LOGCONFIG(TAG, "SAME Decoder:");
  ESP_LOGCONFIG(TAG, "  DIN pin: GPIO%u", this->din_pin_);
  ESP_LOGCONFIG(TAG, "  Sample rate: %u Hz", this->sample_rate_);
  ESP_LOGCONFIG(TAG, "  Samples/bit: %.2f  (window=%d)", SAMPLES_PER_BIT, BIT_WINDOW);
  ESP_LOGCONFIG(TAG, "  Goertzel coeffs: mark=%.6f space=%.6f", COEFF_MARK, COEFF_SPACE);
  ESP_LOGCONFIG(TAG, "  Alert triggers: %u", (unsigned) this->alert_triggers_.size());
  ESP_LOGCONFIG(TAG, "  NOTE: read_samples_() must be wired to I2S RX to decode.");
}

void SAMEDecoder::loop() {
  if (!this->i2s_ready_)
    return;

  if (!this->find_preamble_())
    return;

  std::string header;
  if (!this->assemble_burst_(header))
    return;

  SameAlert alert;
  if (!this->parse_header_(header, alert))
    return;

  this->publish_alert_(alert);
}

// =============================== DSP stages ===============================

bool SAMEDecoder::start_i2s_() {
  ESP_LOGCONFIG(TAG, "DSP: dual-Goertzel demod armed (%.2f samples/bit).", SAMPLES_PER_BIT);
  return true;
}

// Pull PCM from the I2S RX channel into buf. Returns #samples read.
// >>> This is the SINGLE remaining hardware-binding point. <<<
// Wire this to your i2s_audio parent's read API (or a microphone callback)
// for your ESPHome version. Until then it returns 0 and the demod sees silence.
size_t SAMEDecoder::read_samples_(int16_t *buf, size_t max_samples) {
  // TODO(port): example shape (API varies by version):
  //   size_t bytes = this->i2s_parent_->read(buf, max_samples * sizeof(int16_t));
  //   return bytes / sizeof(int16_t);
  (void) buf; (void) max_samples;
  return 0;
}

// Hunt for the preamble (alternating mark/space = 0xAB pattern), one window at a time.
bool SAMEDecoder::find_preamble_() {
  int16_t win[BIT_WINDOW];
  size_t got = this->read_samples_(win, BIT_WINDOW);
  if (got < (size_t) BIT_WINDOW)
    return false;

  bool bit = decode_bit(win, BIT_WINDOW);
  // Track alternation as a proxy for the 0xAB preamble tone pattern.
  if (bit != this->last_preamble_bit_) {
    this->preamble_run_++;
  } else {
    this->preamble_run_ = 0;
  }
  this->last_preamble_bit_ = bit;

  if (this->preamble_run_ >= PREAMBLE_MIN_ALT) {
    ESP_LOGD(TAG, "Preamble lock: %d alternating bits.", this->preamble_run_);
    this->preamble_run_ = 0;
    return true;
  }
  return false;
}

// After preamble, read bytes (LSB-first) for up to 3 bursts, then majority-vote.
bool SAMEDecoder::assemble_burst_(std::string &header_out) {
  std::string bursts[3];
  int captured = 0;

  for (int b = 0; b < 3; b++) {
    std::string hdr;
    uint8_t cur = 0; int nbits = 0;
    int guard = 0;

    while (guard++ < MAX_HEADER_BYTES * 8) {
      int16_t win[BIT_WINDOW];
      size_t got = this->read_samples_(win, BIT_WINDOW);
      if (got < (size_t) BIT_WINDOW)
        break;

      bool bit = decode_bit(win, BIT_WINDOW);
      // LSB-first assembly.
      cur >>= 1;
      if (bit) cur |= 0x80;
      nbits++;

      if (nbits == 8) {
        char c = (char) cur;
        hdr += c;
        cur = 0; nbits = 0;
        // End of message terminator?
        if (hdr.size() >= 4 && hdr.compare(hdr.size() - 4, 4, "NNNN") == 0)
          break;
        if (hdr.size() >= (size_t) MAX_HEADER_BYTES)
          break;
      }
    }

    ESP_LOGD(TAG, "Burst %d raw: '%s'", b, hdr.c_str());
    if (!hdr.empty()) { bursts[b] = hdr; captured++; }
  }

  if (captured == 0) {
    ESP_LOGW(TAG, "No bursts captured.");
    return false;
  }

  // Byte-by-byte 2-of-3 majority vote across whichever bursts we captured.
  size_t maxlen = 0;
  for (int i = 0; i < 3; i++) maxlen = std::max(maxlen, bursts[i].size());
  std::string voted;
  voted.reserve(maxlen);
  for (size_t i = 0; i < maxlen; i++) {
    char best = 0; int bestcount = 0;
    for (int a = 0; a < 3; a++) {
      if (i >= bursts[a].size()) continue;
      char ca = bursts[a][i]; int cnt = 0;
      for (int c = 0; c < 3; c++)
        if (i < bursts[c].size() && bursts[c][i] == ca) cnt++;
      if (cnt > bestcount) { bestcount = cnt; best = ca; }
    }
    if (best) voted += best;
  }

  ESP_LOGD(TAG, "Voted header: '%s'", voted.c_str());
  size_t z = voted.find("ZCZC");
  if (z == std::string::npos) {
    ESP_LOGW(TAG, "Voted header missing ZCZC.");
    return false;
  }
  header_out = voted.substr(z);
  return true;
}

// =========================== Parsing / mapping ===========================

std::string SAMEDecoder::describe_(const std::string &code) {
  auto it = SAME_EVENT_CODES.find(code);
  if (it != SAME_EVENT_CODES.end())
    return it->second.name;
  return code;  // unknown code: fall back to the raw 3-letter code
}

std::string SAMEDecoder::severity_for_(const std::string &code) {
  auto it = SAME_EVENT_CODES.find(code);
  if (it != SAME_EVENT_CODES.end())
    return it->second.severity;
  return "Unknown";
}

std::string SAMEDecoder::make_id_(const SameAlert &a) {
  // Stable across re-broadcasts of the SAME alert: event+areas+onset.
  std::string key = a.event_code + "|" + a.areas_csv + "|" + a.onset_iso;
  size_t h = std::hash<std::string>{}(key);
  std::ostringstream os;
  os << std::hex << h;
  return os.str();
}

bool SAMEDecoder::parse_header_(const std::string &header, SameAlert &out) {
  // Expected: ZCZC-ORG-EEE-PSSCCC-PSSCCC...+TTTT-JJJHHMM-LLLLLLLL-
  // NOTE: Timestamp math (JJJHHMM/purge -> ISO) is left as a TODO to keep this
  // honest; wire it when validating against real captures.
  out.raw_header = header;
  if (header.rfind("ZCZC", 0) != 0)
    return false;

  std::vector<std::string> parts;
  std::string cur;
  for (char c : header) {
    if (c == '-') { parts.push_back(cur); cur.clear(); }
    else { cur += c; }
  }
  if (!cur.empty()) parts.push_back(cur);
  if (parts.size() < 4)
    return false;

  out.originator = parts[1];                               // REAL
  out.event_code = parts[2];                               // REAL
  out.event_name = this->describe_(out.event_code);        // DERIVED
  out.severity   = this->severity_for_(out.event_code);    // DERIVED

  // Areas: everything after event code until the token containing '+'.
  std::string areas;
  for (size_t i = 3; i < parts.size(); i++) {
    if (parts[i].find('+') != std::string::npos) break;
    if (!areas.empty()) areas += ",";
    areas += parts[i];
  }
  out.areas_csv = areas;                                   // REAL

  // Status: RWT/RMT/DMO/NPT => Test, else Actual (simple heuristic).
  out.status = (out.event_code == "RWT" || out.event_code == "RMT" ||
                out.event_code == "DMO" || out.event_code == "NPT")
                   ? "Test" : "Actual";                    // DERIVED

  // TODO: parse purge (+TTTT) and issue (JJJHHMM) into onset_iso/expires_iso.
  out.onset_iso = "";                                      // DERIVED (TODO)
  out.expires_iso = "";                                    // DERIVED (TODO)
  out.sender = parts.empty() ? "" : parts.back();          // REAL (LLLLLLLL)

  out.id = this->make_id_(out);                            // DERIVED stable hash
  return true;
}

void SAMEDecoder::publish_alert_(const SameAlert &a) {
  this->last_ = a;
  this->decode_count_++;

  if (this->decode_count_sensor_ != nullptr)
    this->decode_count_sensor_->publish_state((float) this->decode_count_);
  if (this->last_raw_sensor_ != nullptr)
    this->last_raw_sensor_->publish_state(a.raw_header);

  ESP_LOGI(TAG, "Decoded SAME: %s (%s) areas=%s",
           a.event_name.c_str(), a.event_code.c_str(), a.areas_csv.c_str());

  // Fire the on_alert automation(s); lambdas read the accessors off `this`.
  for (auto *t : this->alert_triggers_)
    t->trigger();
}

}  // namespace same_decoder
}  // namespace esphome
