// components/same_decoder/same_decoder.cpp
#include "same_decoder.h"
#include "same_event_codes.h"
#include "esphome/core/log.h"

#include <functional>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <cinttypes>   // PRIu32

namespace esphome {
namespace same_decoder {

static const char *const TAG = "same_decoder";

static constexpr float COEFF_MARK  = 1.926090f;
static constexpr float COEFF_SPACE = 1.958313f;

static float goertzel_energy(const int16_t *x, int n, float coeff) {
  float s1 = 0.f, s2 = 0.f;
  for (int i = 0; i < n; i++) {
    float s0 = (float) x[i] + coeff * s1 - s2;
    s2 = s1;
    s1 = s0;
  }
  return s1 * s1 + s2 * s2 - coeff * s1 * s2;
}

static bool decode_bit(const int16_t *win, int n) {
  float em = goertzel_energy(win, n, COEFF_MARK);
  float es = goertzel_energy(win, n, COEFF_SPACE);
  return em > es;
}

void SAMEDecoder::setup() {
  ESP_LOGCONFIG(TAG, "SAME decoder ready (%.2f samples/bit, non-blocking state machine).",
                SAMPLES_PER_BIT);
  this->reset_capture_();
}

void SAMEDecoder::dump_config() {
  ESP_LOGCONFIG(TAG, "SAME Decoder:");
  ESP_LOGCONFIG(TAG, "  Sample rate: %" PRIu32 " Hz", this->sample_rate_);
  ESP_LOGCONFIG(TAG, "  Bit window: %d samples (%.2f/bit)", BIT_WINDOW, SAMPLES_PER_BIT);
  ESP_LOGCONFIG(TAG, "  Goertzel coeffs: mark=%.6f space=%.6f", COEFF_MARK, COEFF_SPACE);
  ESP_LOGCONFIG(TAG, "  Alert triggers: %u", (unsigned) this->alert_triggers_.size());
  ESP_LOGCONFIG(TAG, "  Audio via microphone:on_data -> feed_bytes().");
}

void SAMEDecoder::feed_bytes(const std::vector<uint8_t> &data) {
  const size_t n = data.size() / 2;
  const int16_t *samples = reinterpret_cast<const int16_t *>(data.data());
  for (size_t i = 0; i < n; i++)
    this->feed_sample_(samples[i]);
}

void SAMEDecoder::feed_sample_(int16_t s) {
  this->win_[this->win_fill_++] = s;
  if (this->win_fill_ < BIT_WINDOW)
    return;

  bool bit = decode_bit(this->win_, BIT_WINDOW);
  this->win_fill_ = 0;

  this->bit_phase_ += (SAMPLES_PER_BIT - BIT_WINDOW);
  if (this->bit_phase_ >= 1.0f) {
    this->bit_phase_ -= 1.0f;
  }

  this->process_bit_(bit);
}

void SAMEDecoder::reset_capture_() {
  this->phase_ = HUNT_PREAMBLE;
  this->preamble_run_ = 0;
  this->cur_byte_ = 0;
  this->cur_nbits_ = 0;
  this->cur_burst_.clear();
  this->burst_idx_ = 0;
  for (int i = 0; i < 3; i++) this->bursts_[i].clear();
}

void SAMEDecoder::process_bit_(bool bit) {
  if (this->phase_ == HUNT_PREAMBLE) {
    if (bit != this->last_bit_) {
      this->preamble_run_++;
    } else {
      this->preamble_run_ = 0;
    }
    this->last_bit_ = bit;

    if (this->preamble_run_ >= PREAMBLE_MIN_ALT) {
      ESP_LOGD(TAG, "Preamble lock: %d alternating bits.", this->preamble_run_);
      this->phase_ = CAPTURE;
      this->cur_byte_ = 0;
      this->cur_nbits_ = 0;
      this->cur_burst_.clear();
    }
    return;
  }

  this->cur_byte_ >>= 1;
  if (bit) this->cur_byte_ |= 0x80;
  this->cur_nbits_++;

  if (this->cur_nbits_ == 8) {
    char c = (char) this->cur_byte_;
    this->cur_burst_ += c;
    this->cur_byte_ = 0;
    this->cur_nbits_ = 0;

    bool eom = (this->cur_burst_.size() >= 4 &&
                this->cur_burst_.compare(this->cur_burst_.size() - 4, 4, "NNNN") == 0);
    if (eom || this->cur_burst_.size() >= (size_t) MAX_HEADER_BYTES) {
      ESP_LOGD(TAG, "Burst %d raw: '%s'", this->burst_idx_, this->cur_burst_.c_str());
      this->bursts_[this->burst_idx_] = this->cur_burst_;
      this->burst_idx_++;
      this->cur_burst_.clear();

      if (this->burst_idx_ >= 3) {
        this->vote_and_emit_();
        this->reset_capture_();
      } else {
        this->cur_byte_ = 0;
        this->cur_nbits_ = 0;
      }
    }
  }
}

void SAMEDecoder::vote_and_emit_() {
  size_t maxlen = 0;
  for (int i = 0; i < 3; i++) maxlen = std::max(maxlen, this->bursts_[i].size());
  std::string voted;
  voted.reserve(maxlen);
  for (size_t i = 0; i < maxlen; i++) {
    char best = 0; int bestcount = 0;
    for (int a = 0; a < 3; a++) {
      if (i >= this->bursts_[a].size()) continue;
      char ca = this->bursts_[a][i]; int cnt = 0;
      for (int c = 0; c < 3; c++)
        if (i < this->bursts_[c].size() && this->bursts_[c][i] == ca) cnt++;
      if (cnt > bestcount) { bestcount = cnt; best = ca; }
    }
    if (best) voted += best;
  }

  ESP_LOGD(TAG, "Voted header: '%s'", voted.c_str());
  size_t z = voted.find("ZCZC");
  if (z == std::string::npos) {
    ESP_LOGW(TAG, "Voted header missing ZCZC; discarding.");
    return;
  }

  SameAlert alert;
  if (this->parse_header_(voted.substr(z), alert))
    this->publish_alert_(alert);
}

std::string SAMEDecoder::describe_(const std::string &code) {
  auto it = SAME_EVENT_CODES.find(code);
  if (it != SAME_EVENT_CODES.end())
    return it->second.name;
  return code;
}

std::string SAMEDecoder::severity_for_(const std::string &code) {
  auto it = SAME_EVENT_CODES.find(code);
  if (it != SAME_EVENT_CODES.end())
    return it->second.severity;
  return "Unknown";
}

std::string SAMEDecoder::make_id_(const SameAlert &a) {
  std::string key = a.event_code + "|" + a.areas_csv + "|" + a.onset_iso;
  size_t h = std::hash<std::string>{}(key);
  std::ostringstream os;
  os << std::hex << h;
  return os.str();
}

bool SAMEDecoder::parse_header_(const std::string &header, SameAlert &out) {
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

  out.originator = parts[$1];
  out.event_code = parts;
  out.event_name = this->describe_(out.event_code);
  out.severity   = this->severity_for_(out.event_code);

  std::string areas;
  for (size_t i = 3; i < parts.size(); i++) {
    if (parts[i].find('+') != std::string::npos) break;
    if (!areas.empty()) areas += ",";
    areas += parts[i];
  }
  out.areas_csv = areas;

  out.status = (out.event_code == "RWT" || out.event_code == "RMT" ||
                out.event_code == "DMO" || out.event_code == "NPT")
                   ? "Test" : "Actual";

  out.onset_iso = "";
  out.expires_iso = "";
  out.sender = parts.empty() ? "" : parts.back();

  out.id = this->make_id_(out);
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

  for (auto *t : this->alert_triggers_)
    t->trigger();
}

}  // namespace same_decoder
}  // namespace esphome
