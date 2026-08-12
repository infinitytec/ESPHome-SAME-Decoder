// components/same_decoder/same_decoder.cpp
#include "same_decoder.h"
#include "same_event_codes.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

#include <functional>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <cinttypes>
#include <cmath>

namespace esphome {
namespace same_decoder {

static const char *const TAG = "same_decoder";

// Sanitize to printable 7-bit ASCII so the ESPHome API never receives invalid UTF-8.
static std::string sanitize_ascii(const std::string &in) {
  std::string out;
  out.reserve(in.size());
  for (unsigned char c : in) {
    if (c >= 0x20 && c <= 0x7E)
      out += (char) c;
    else
      out += '?';
  }
  return out;
}

void SAMEDecoder::setup() {
  this->samples_per_bit_ = (float) this->sample_rate_ / 520.83f;
  this->phase_inc_ = 1.0f / this->samples_per_bit_;
  this->update_goertzel_coeffs_();
  ESP_LOGCONFIG(TAG, "SAME decoder ready (multi-bin AFC, soft decisions, burst timeout).");
  this->reset_capture_();
}

void SAMEDecoder::loop() {
  // If we have at least one burst and the group has gone quiet longer than the
  // timeout, emit whatever we have and return to hunt. Prevents a missed third
  // repeat from blocking the next alert.
  if (this->burst_idx_ > 0 && this->group_start_ms_ != 0) {
    const uint32_t now = millis();
    const uint32_t idle = now - this->last_activity_ms_;
    const uint32_t age = now - this->group_start_ms_;
    if (idle >= this->burst_timeout_ms_ || age >= (this->burst_timeout_ms_ + 5000)) {
      ESP_LOGD(TAG, "Burst group timeout (%u bursts, idle=%u ms); emitting partial.",
               (unsigned) this->burst_idx_, (unsigned) idle);
      this->vote_and_emit_();
      this->reset_capture_();
    }
  }
}

void SAMEDecoder::dump_config() {
  ESP_LOGCONFIG(TAG, "SAME Decoder:");
  ESP_LOGCONFIG(TAG, "  Sample rate: %" PRIu32 " Hz", this->sample_rate_);
  ESP_LOGCONFIG(TAG, "  Software gain: %.1f", this->gain_);
  ESP_LOGCONFIG(TAG, "  Samples/bit: %.2f, Goertzel window: %d", this->samples_per_bit_, GWIN);
  ESP_LOGCONFIG(TAG, "  Frequency bins: %d  step: %.0f Hz", NUM_BINS, BIN_STEP_HZ);
  ESP_LOGCONFIG(TAG, "  Max sync Hamming distance: %d", MAX_SYNC_HAMMING);
  ESP_LOGCONFIG(TAG, "  Burst group timeout: %" PRIu32 " ms", this->burst_timeout_ms_);
  ESP_LOGCONFIG(TAG, "  Alert triggers: %u", (unsigned) this->alert_triggers_.size());
}

void SAMEDecoder::update_goertzel_coeffs_() {
  for (int b = 0; b < NUM_BINS; b++) {
    float offset = (b - (NUM_BINS / 2)) * BIN_STEP_HZ + this->freq_offset_hz_;
    float fm = NOM_MARK_HZ + offset;
    float fs = NOM_SPACE_HZ + offset;
    this->coeff_mark_[b] =
        2.0f * std::cos(2.0f * (float) M_PI * fm / (float) this->sample_rate_);
    this->coeff_space_[b] =
        2.0f * std::cos(2.0f * (float) M_PI * fs / (float) this->sample_rate_);
  }
}

float SAMEDecoder::goertzel_ring_(int start, int n, float coeff) const {
  float s1 = 0.f, s2 = 0.f;
  for (int i = 0; i < n; i++) {
    int idx = (start + i) % RING_LEN;
    float s0 = (float) this->ring_[idx] + coeff * s1 - s2;
    s2 = s1;
    s1 = s0;
  }
  return s1 * s1 + s2 * s2 - coeff * s1 * s2;
}

int SAMEDecoder::hamming32_(uint32_t a, uint32_t b) const {
  uint32_t x = a ^ b;
  x = x - ((x >> 1) & 0x55555555u);
  x = (x & 0x33333333u) + ((x >> 2) & 0x33333333u);
  return (int) ((((x + (x >> 4)) & 0x0F0F0F0Fu) * 0x01010101u) >> 24);
}

void SAMEDecoder::feed_bytes(const std::vector<uint8_t> &data) {
  const size_t n = data.size() / 2;
  const int16_t *samples = reinterpret_cast<const int16_t *>(data.data());
  for (size_t i = 0; i < n; i++) {
    float v = (float) samples[i] * this->gain_;
    if (v > 32767.0f)
      v = 32767.0f;
    if (v < -32768.0f)
      v = -32768.0f;
    this->feed_sample_((int16_t) v);
  }
}

void SAMEDecoder::feed_sample_(int16_t s) {
  this->ring_[this->ring_pos_] = s;
  this->ring_pos_ = (this->ring_pos_ + 1) % RING_LEN;

  this->phase_ += this->phase_inc_;
  if (this->phase_ < 1.0f)
    return;
  this->phase_ -= 1.0f;

  int center_start = (this->ring_pos_ - GWIN + RING_LEN) % RING_LEN;

  // Evaluate all frequency bins; keep highest mark/space contrast.
  float best_contrast = -1.0f;
  float best_em = 0.f, best_es = 0.f;
  int best_b = this->best_bin_;

  for (int b = 0; b < NUM_BINS; b++) {
    float em = this->goertzel_ring_(center_start, GWIN, this->coeff_mark_[b]);
    float es = this->goertzel_ring_(center_start, GWIN, this->coeff_space_[b]);
    float contrast = std::fabs(em - es);
    if (contrast > best_contrast) {
      best_contrast = contrast;
      best_em = em;
      best_es = es;
      best_b = b;
    }
  }
  this->best_bin_ = best_b;

  float soft = (best_em - best_es) / (this->agc_level_ + 1.0f);

  // Light AGC on decision magnitude.
  float mag = std::fabs(best_em) + std::fabs(best_es);
  this->agc_level_ = (1.0f - AGC_ALPHA) * this->agc_level_ + AGC_ALPHA * mag;

  // Running signal vs residual energy for an SNR-style metric.
  float sig = std::max(best_em, best_es);
  float res = std::min(best_em, best_es);
  this->energy_signal_ = (1.0f - SNR_ALPHA) * this->energy_signal_ + SNR_ALPHA * sig;
  this->energy_noise_ = (1.0f - SNR_ALPHA) * this->energy_noise_ + SNR_ALPHA * res;
  if (this->energy_noise_ > 1.0f) {
    this->last_snr_db_ = 10.0f * std::log10(this->energy_signal_ / this->energy_noise_);
  }

  // Early / late samples for timing recovery.
  int early_off = (int) (EARLY_FRAC * this->samples_per_bit_);
  int late_off = (int) (LATE_FRAC * this->samples_per_bit_);
  int early_start = (center_start - early_off + RING_LEN) % RING_LEN;
  int late_start = (center_start + late_off) % RING_LEN;

  float e_early = this->goertzel_ring_(early_start, GWIN, this->coeff_mark_[best_b]) -
                  this->goertzel_ring_(early_start, GWIN, this->coeff_space_[best_b]);
  float e_late = this->goertzel_ring_(late_start, GWIN, this->coeff_mark_[best_b]) -
                 this->goertzel_ring_(late_start, GWIN, this->coeff_space_[best_b]);

  float timing_err = (std::fabs(e_late) - std::fabs(e_early));
  this->phase_ += 0.04f * (timing_err / (std::fabs(soft) + 1.0f));

  // Simple AFC: walk residual offset toward the winning bin.
  if (best_b != (NUM_BINS / 2)) {
    float desired = (best_b - (NUM_BINS / 2)) * BIN_STEP_HZ;
    this->freq_offset_hz_ =
        (1.0f - this->afc_alpha_) * this->freq_offset_hz_ + this->afc_alpha_ * desired;
    static int afc_counter = 0;
    if (++afc_counter >= 32) {
      afc_counter = 0;
      this->update_goertzel_coeffs_();
    }
  }

  bool hard = soft > 0.0f;
  this->emit_bit_(hard, soft);
}

void SAMEDecoder::reset_capture_() {
  this->phase_state_ = HUNT_SYNC;
  this->sync_shift_ = 0;
  this->cur_byte_ = 0;
  this->cur_nbits_ = 0;
  this->cur_burst_.clear();
  this->burst_idx_ = 0;
  for (int i = 0; i < 3; i++) {
    this->bursts_[i].clear();
    this->burst_quality_[i] = 0.0f;
  }
  this->soft_accum_ = 0.0f;
  this->soft_count_ = 0;
  this->group_start_ms_ = 0;
  this->last_activity_ms_ = 0;
}

void SAMEDecoder::emit_bit_(bool hard_bit, float soft) {
  this->last_activity_ms_ = millis();

  if (this->phase_state_ == CAPTURE) {
    this->soft_accum_ += std::fabs(soft);
    this->soft_count_++;
  }

  if (this->phase_state_ == HUNT_SYNC) {
    this->sync_shift_ = (this->sync_shift_ >> 1) | ((uint32_t) (hard_bit ? 1u : 0u) << 31);

    int dist = this->hamming32_(this->sync_shift_, SYNC_ZCZC);
    if (dist <= MAX_SYNC_HAMMING) {
      ESP_LOGD(TAG, "ZCZC sync found (Hamming=%d). Byte-aligned capture starting.", dist);
      this->phase_state_ = CAPTURE;
      this->cur_burst_ = "ZCZC";
      this->cur_byte_ = 0;
      this->cur_nbits_ = 0;
      this->soft_accum_ = 0.0f;
      this->soft_count_ = 0;
      if (this->burst_idx_ == 0)
        this->group_start_ms_ = millis();
    }
    return;
  }

  // CAPTURE — assemble LSB-first bytes.
  this->cur_byte_ >>= 1;
  if (hard_bit)
    this->cur_byte_ |= 0x80;
  this->cur_nbits_++;

  if (this->cur_nbits_ == 8) {
    char c = (char) this->cur_byte_;
    this->cur_burst_ += c;
    this->cur_byte_ = 0;
    this->cur_nbits_ = 0;

    bool eom = (this->cur_burst_.size() >= 4 &&
                this->cur_burst_.compare(this->cur_burst_.size() - 4, 4, "NNNN") == 0);
    // Tolerate a single-character error in the EOM marker.
    if (!eom && this->cur_burst_.size() >= 4) {
      const char *tail = this->cur_burst_.c_str() + this->cur_burst_.size() - 4;
      int errs = 0;
      for (int i = 0; i < 4; i++)
        if (tail[i] != 'N')
          errs++;
      if (errs <= 1)
        eom = true;
    }

    if (eom || this->cur_burst_.size() >= (size_t) MAX_HEADER_BYTES) {
      if (eom) {
        size_t nn = this->cur_burst_.rfind("NNNN");
        if (nn != std::string::npos) {
          this->cur_burst_.erase(nn + 4);
        } else {
          // Near-match: trim to last plausible end.
          for (size_t i = this->cur_burst_.size(); i-- > 3;) {
            if (this->cur_burst_[i] == 'N') {
              this->cur_burst_.erase(i + 1);
              break;
            }
          }
        }
      }

      std::string hex;
      char tmp[4];
      for (size_t i = 0; i < this->cur_burst_.size() && i < 32; i++) {
        snprintf(tmp, sizeof(tmp), "%02X ", (uint8_t) this->cur_burst_[i]);
        hex += tmp;
      }
      float q = (this->soft_count_ > 0) ? (this->soft_accum_ / this->soft_count_) : 0.0f;
      ESP_LOGD(TAG, "Burst %d ascii: '%s'  quality=%.1f", this->burst_idx_,
               sanitize_ascii(this->cur_burst_).c_str(), q);
      ESP_LOGD(TAG, "Burst %d hex : %s", this->burst_idx_, hex.c_str());

      this->bursts_[this->burst_idx_] = this->cur_burst_;
      this->burst_quality_[this->burst_idx_] = q;
      this->burst_idx_++;
      this->cur_burst_.clear();
      this->soft_accum_ = 0.0f;
      this->soft_count_ = 0;

      if (this->burst_idx_ >= 3) {
        this->vote_and_emit_();
        this->reset_capture_();
      } else {
        // Re-hunt for the next repeat (robust to gaps between bursts).
        this->phase_state_ = HUNT_SYNC;
        this->sync_shift_ = 0;
        this->cur_byte_ = 0;
        this->cur_nbits_ = 0;
      }
    }
  }
}

void SAMEDecoder::vote_and_emit_() {
  if (this->burst_idx_ < 1)
    return;

  size_t maxlen = 0;
  for (int i = 0; i < this->burst_idx_; i++)
    maxlen = std::max(maxlen, this->bursts_[i].size());

  std::string voted;
  voted.reserve(maxlen);

  for (size_t i = 0; i < maxlen; i++) {
    // Quality-weighted majority over the bursts we actually have.
    char best = 0;
    float best_score = -1.0f;
    for (int a = 0; a < this->burst_idx_; a++) {
      if (i >= this->bursts_[a].size())
        continue;
      char ca = this->bursts_[a][i];
      float score = 0.0f;
      for (int c = 0; c < this->burst_idx_; c++) {
        if (i < this->bursts_[c].size() && this->bursts_[c][i] == ca)
          score += this->burst_quality_[c] + 0.1f;
      }
      if (score > best_score) {
        best_score = score;
        best = ca;
      }
    }
    if (best)
      voted += best;
  }

  float total_q = 0.0f;
  int used = 0;
  for (int i = 0; i < this->burst_idx_; i++) {
    if (!this->bursts_[i].empty()) {
      total_q += this->burst_quality_[i];
      used++;
    }
  }
  this->last_quality_ = (used > 0) ? (total_q / used) : 0.0f;

  ESP_LOGD(TAG, "Voted header (%d bursts): '%s'  quality=%.1f  snr=%.1f dB  offset=%.1f Hz",
           this->burst_idx_, sanitize_ascii(voted).c_str(), this->last_quality_,
           this->last_snr_db_, this->freq_offset_hz_);

  if (this->quality_sensor_ != nullptr)
    this->quality_sensor_->publish_state(this->last_quality_);
  if (this->snr_sensor_ != nullptr)
    this->snr_sensor_->publish_state(this->last_snr_db_);
  if (this->freq_offset_sensor_ != nullptr)
    this->freq_offset_sensor_->publish_state(this->freq_offset_hz_);

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
    if (c == '-') {
      parts.push_back(cur);
      cur.clear();
    } else {
      cur += c;
    }
  }
  if (!cur.empty())
    parts.push_back(cur);
  if (parts.size() < 4)
    return false;

  out.originator = parts[1];
  out.event_code = parts[2];
  out.event_name = this->describe_(out.event_code);
  out.severity = this->severity_for_(out.event_code);

  std::string areas;
  for (size_t i = 3; i < parts.size(); i++) {
    if (parts[i].find('+') != std::string::npos)
      break;
    if (!areas.empty())
      areas += ",";
    areas += parts[i];
  }
  out.areas_csv = areas;

  out.status = (out.event_code == "RWT" || out.event_code == "RMT" ||
                out.event_code == "DMO" || out.event_code == "NPT")
                   ? "Test"
                   : "Actual";

  out.onset_iso = "";
  out.expires_iso = "";
  out.sender = parts.empty() ? "" : parts.back();
  out.id = this->make_id_(out);
  return true;
}

void SAMEDecoder::publish_alert_(const SameAlert &a) {
  this->last_ = a;
  this->last_.raw_header = sanitize_ascii(a.raw_header);
  this->decode_count_++;

  if (this->decode_count_sensor_ != nullptr)
    this->decode_count_sensor_->publish_state((float) this->decode_count_);
  if (this->last_raw_sensor_ != nullptr)
    this->last_raw_sensor_->publish_state(this->last_.raw_header);

  ESP_LOGI(TAG, "Decoded SAME: %s (%s) areas=%s  quality=%.1f  snr=%.1f dB  offset=%.1f Hz",
           a.event_name.c_str(), a.event_code.c_str(), a.areas_csv.c_str(),
           this->last_quality_, this->last_snr_db_, this->freq_offset_hz_);

  for (auto *t : this->alert_triggers_)
    t->trigger();
}

}  // namespace same_decoder
}  // namespace esphome
