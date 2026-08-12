// components/same_decoder/same_decoder.cpp
#include "same_decoder.h"
#include "same_event_codes.h"
#include "esphome/core/log.h"

#include <functional>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <cinttypes>
#include <cmath>

namespace esphome {
namespace same_decoder {

static const char *const TAG = "same_decoder";

static constexpr float COEFF_MARK  = 1.926090f;   // 2*cos(2*pi*2083.33/48000)
static constexpr float COEFF_SPACE = 1.958313f;   // 2*cos(2*pi*1562.5/48000)

// Replace any byte that is not printable 7-bit ASCII (0x20..0x7E) with '?'.
// SAME headers are pure ASCII by spec; anything else is capture noise. This
// guarantees the string we hand to the ESPHome API (a protobuf UTF-8 string)
// is always valid UTF-8 and cannot crash the connection.
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

// Goertzel energy over `n` samples pulled from the ring buffer. The window
// ENDS at ring index `endpos` (inclusive) and extends backwards `n` samples.
static float goertzel_ring_end(const int16_t *ring, int ringlen, int endpos, int n, float coeff) {
  float s1 = 0.f, s2 = 0.f;
  int startpos = ((endpos - (n - 1)) % ringlen + ringlen) % ringlen;
  for (int i = 0; i < n; i++) {
    int idx = (startpos + i) % ringlen;
    float s0 = (float) ring[idx] + coeff * s1 - s2;
    s2 = s1;
    s1 = s0;
  }
  return s1 * s1 + s2 * s2 - coeff * s1 * s2;
}

void SAMEDecoder::setup() {
  ESP_LOGCONFIG(TAG, "SAME decoder ready (timing-recovery, gain=%.1f).", this->gain_);
  this->reset_capture_();
}

void SAMEDecoder::dump_config() {
  ESP_LOGCONFIG(TAG, "SAME Decoder:");
  ESP_LOGCONFIG(TAG, "  Sample rate: %" PRIu32 " Hz", this->sample_rate_);
  ESP_LOGCONFIG(TAG, "  Software gain: %.1f", this->gain_);
  ESP_LOGCONFIG(TAG, "  Samples/bit: %.2f, Goertzel window: %d, ring: %d", SAMPLES_PER_BIT, GWIN, RINGLEN);
  ESP_LOGCONFIG(TAG, "  Goertzel coeffs: mark=%.6f space=%.6f", COEFF_MARK, COEFF_SPACE);
  ESP_LOGCONFIG(TAG, "  Timing loop: Kp=%.3f Ki=%.4f delta=%d conf_min=%.2f", TR_KP, TR_KI, TR_DELTA, TR_CONF_MIN);
  ESP_LOGCONFIG(TAG, "  Alert triggers: %u", (unsigned) this->alert_triggers_.size());
}

void SAMEDecoder::feed_bytes(const std::vector<uint8_t> &data) {
  const size_t n = data.size() / 2;
  const int16_t *samples = reinterpret_cast<const int16_t *>(data.data());
  for (size_t i = 0; i < n; i++) {
    float v = (float) samples[i] * this->gain_;
    if (v >  32767.0f) v =  32767.0f;
    if (v < -32768.0f) v = -32768.0f;
    this->feed_sample_((int16_t) v);
  }
}

// Push each sample into the ring; advance the bit-clock phase (with rate
// correction). When phase crosses 1.0, make a bit decision using a Goertzel
// window CENTERED on the bit, and run an early/late timing update that nudges
// phase and bit-rate toward the true bit center.
void SAMEDecoder::feed_sample_(int16_t s) {
  this->ring_[this->ring_pos_] = s;
  this->ring_pos_ = (this->ring_pos_ + 1) % RINGLEN;
  if (this->samples_seen_ < 0x7fffffff) this->samples_seen_++;

  this->phase_ += (PHASE_INC + this->w_off_);
  if (this->phase_ < 1.0f)
    return;
  this->phase_ -= 1.0f;

  // Index of the most-recently written sample.
  int s_now = (this->ring_pos_ - 1 + RINGLEN) % RINGLEN;

  // Window END indices for center / early / late. Centering the window on the
  // bit (CENTER_LAG back from "now") removes the half-window bias the old code
  // had from ending the window at the newest sample.
  int t_center = (s_now - CENTER_LAG + RINGLEN) % RINGLEN;
  int t_early  = (t_center - TR_DELTA + RINGLEN) % RINGLEN;
  int t_late   = (t_center + TR_DELTA) % RINGLEN;

  // Center energies + bit decision.
  float em_c = goertzel_ring_end(this->ring_, RINGLEN, t_center, GWIN, COEFF_MARK);
  float es_c = goertzel_ring_end(this->ring_, RINGLEN, t_center, GWIN, COEFF_SPACE);
  bool bit = em_c > es_c;

  // Only adapt timing once the ring is primed with enough history for the
  // furthest-back (late+window) read, so early bits don't chase stale data.
  bool primed = this->samples_seen_ >= (uint32_t) (CENTER_LAG + TR_DELTA + GWIN + 2);
  if (primed) {
    float em_e = goertzel_ring_end(this->ring_, RINGLEN, t_early, GWIN, COEFF_MARK);
    float es_e = goertzel_ring_end(this->ring_, RINGLEN, t_early, GWIN, COEFF_SPACE);
    float em_l = goertzel_ring_end(this->ring_, RINGLEN, t_late,  GWIN, COEFF_MARK);
    float es_l = goertzel_ring_end(this->ring_, RINGLEN, t_late,  GWIN, COEFF_SPACE);

    float d_early = std::fabs(em_e - es_e);
    float d_late  = std::fabs(em_l - es_l);
    float d_center = std::fabs(em_c - es_c);
    float e_center = em_c + es_c;

    const float eps = 1e-9f;
    float e_raw = (d_late - d_early) / (d_late + d_early + eps);
    float conf  = d_center / (e_center + eps);
    float e = (conf >= TR_CONF_MIN) ? e_raw : 0.0f;

    // Proportional phase correction (clamped).
    float dphi = TR_KP * e;
    if (dphi >  TR_DPHI_CLAMP) dphi =  TR_DPHI_CLAMP;
    if (dphi < -TR_DPHI_CLAMP) dphi = -TR_DPHI_CLAMP;
    this->phase_ += dphi;

    // Integral bit-rate correction (clamped) - removes steady drift.
    this->w_off_ += TR_KI * e;
    if (this->w_off_ >  TR_WOFF_CLAMP) this->w_off_ =  TR_WOFF_CLAMP;
    if (this->w_off_ < -TR_WOFF_CLAMP) this->w_off_ = -TR_WOFF_CLAMP;
  }

  this->emit_bit_(bit);
}

void SAMEDecoder::reset_capture_() {
  this->phase_state_ = HUNT_SYNC;
  this->sync_shift_ = 0;
  this->cur_byte_ = 0;
  this->cur_nbits_ = 0;
  this->cur_burst_.clear();
  this->burst_idx_ = 0;
  for (int i = 0; i < 3; i++) this->bursts_[i].clear();
}

void SAMEDecoder::emit_bit_(bool bit) {
  if (this->phase_state_ == HUNT_SYNC) {
    // Shift new bit into MSB side; first-received bit ends up toward LSB as we
    // shift right. Build a rolling 32-bit window and compare against ZCZC.
    this->sync_shift_ = (this->sync_shift_ >> 1) | ((uint32_t) (bit ? 1u : 0u) << 31);

    // 'ZCZC' LSB-first bitstream: Z=0x5A->01011010, C=0x43->11000010 (LSB-first
    // per-byte), concatenated = 01011010 11000010 01011010 11000010.
    // Assembled into a 32-bit word as received (first bit -> LSB after 32 shifts):
    // We match SYNC_ZCZC computed to equal that received pattern.
    if (this->sync_shift_ == SYNC_ZCZC) {
      ESP_LOGD(TAG, "ZCZC sync found. Byte-aligned capture starting.");
      this->phase_state_ = CAPTURE;
      // Seed the burst with the known "ZCZC" we just matched.
      this->cur_burst_ = "ZCZC";
      this->cur_byte_ = 0;
      this->cur_nbits_ = 0;
    }
    return;
  }

  // CAPTURE -- assemble LSB-first bytes.
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
      // On a clean end-of-message, trim anything captured past the "NNNN" so
      // trailing noise bytes never reach the parser or the text sensor. (On a
      // length-limited stop there is no NNNN to trim to; keep what we have.)
      if (eom) {
        size_t nn = this->cur_burst_.rfind("NNNN");
        if (nn != std::string::npos)
          this->cur_burst_.erase(nn + 4);
      }

      // Hex+ASCII diagnostic of the assembled burst.
      std::string hex;
      char tmp[4];
      for (size_t i = 0; i < this->cur_burst_.size() && i < 32; i++) {
        snprintf(tmp, sizeof(tmp), "%02X ", (uint8_t) this->cur_burst_[i]);
        hex += tmp;
      }
      ESP_LOGD(TAG, "Burst %d ascii: '%s'", this->burst_idx_, sanitize_ascii(this->cur_burst_).c_str());
      ESP_LOGD(TAG, "Burst %d hex : %s", this->burst_idx_, hex.c_str());

      this->bursts_[this->burst_idx_] = this->cur_burst_;
      this->burst_idx_++;
      this->cur_burst_.clear();

      if (this->burst_idx_ >= 3) {
        this->vote_and_emit_();
        this->reset_capture_();
      } else {
        // Next repeat: hunt for its ZCZC again (robust to gaps between bursts).
        this->phase_state_ = HUNT_SYNC;
        this->sync_shift_ = 0;
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

  ESP_LOGD(TAG, "Voted header: '%s'", sanitize_ascii(voted).c_str());
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

  out.originator = parts[1];
  out.event_code = parts[2];
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
  // Ensure the raw header we retain and publish is always valid 7-bit ASCII so
  // the ESPHome API (protobuf UTF-8 string) can never be poisoned by capture
  // noise. SAME headers are ASCII by spec, so this is lossless for valid data.
  this->last_.raw_header = sanitize_ascii(a.raw_header);
  this->decode_count_++;

  if (this->decode_count_sensor_ != nullptr)
    this->decode_count_sensor_->publish_state((float) this->decode_count_);
  if (this->last_raw_sensor_ != nullptr)
    this->last_raw_sensor_->publish_state(this->last_.raw_header);

  ESP_LOGI(TAG, "Decoded SAME: %s (%s) areas=%s",
           a.event_name.c_str(), a.event_code.c_str(), a.areas_csv.c_str());

  for (auto *t : this->alert_triggers_)
    t->trigger();
}

}  // namespace same_decoder
}  // namespace esphome
