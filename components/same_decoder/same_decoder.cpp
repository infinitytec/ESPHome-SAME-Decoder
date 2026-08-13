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

static constexpr float COEFF_MARK  = 1.926090f;
static constexpr float COEFF_SPACE = 1.958313f;

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

static bool is_valid_same_char(char c) {
  return (c >= '0' && c <= '9') ||
         (c >= 'A' && c <= 'Z') ||
         c == '-' || c == '+' || c == '/';
}

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

float SAMEDecoder::soft_clip_(float v) {
  float a = std::fabs(v);
  if (a <= SOFT_KNEE)
    return v;
  float sign = (v < 0.0f) ? -1.0f : 1.0f;
  float over = (a - SOFT_KNEE) / (SAMPLE_MAX - SOFT_KNEE);
  float compressed = SOFT_KNEE + (SAMPLE_MAX - SOFT_KNEE) * std::tanh(over);
  return sign * compressed;
}

void SAMEDecoder::setup() {
  ESP_LOGCONFIG(TAG, "SAME decoder ready (timing-recovery, gain=%.1f).", this->gain_);
  this->reset_capture_();
}

void SAMEDecoder::set_api_connected(bool connected) {
  if (connected != this->api_connected_) {
    this->api_connected_ = connected;
    this->api_connected_changed_.store(true, std::memory_order_release);
  }
}

void SAMEDecoder::loop() {
  if (this->api_connected_changed_.exchange(false, std::memory_order_acq_rel)) {
    if (this->api_connected_)
      this->flush_pending_();
  }

  uint32_t pending = this->sync_pending_.exchange(0, std::memory_order_acq_rel);
  while (pending > 0) {
    for (auto *t : this->sync_triggers_)
      t->trigger();
    pending--;
  }

  uint32_t head = this->q_head_.load(std::memory_order_relaxed);
  while (head != this->q_tail_.load(std::memory_order_acquire)) {
    SameAlert a = this->alert_queue_[head];
    head = (head + 1) % ALERT_Q_LEN;
    this->q_head_.store(head, std::memory_order_release);
    this->dispatch_alert_(a);
  }
}

void SAMEDecoder::dump_config() {
  ESP_LOGCONFIG(TAG, "SAME Decoder:");
  ESP_LOGCONFIG(TAG, "  Sample rate: %" PRIu32 " Hz", this->sample_rate_);
  ESP_LOGCONFIG(TAG, "  Software gain: %.1f", this->gain_);
  ESP_LOGCONFIG(TAG, "  Soft-clip knee: %.0f (rail %.0f)", SOFT_KNEE, SAMPLE_MAX);
  ESP_LOGCONFIG(TAG, "  Samples/bit: %.2f, Goertzel window: %d, ring: %d", SAMPLES_PER_BIT, GWIN, RINGLEN);
  ESP_LOGCONFIG(TAG, "  Timing loop: Kp=%.3f Ki=%.4f delta=%d conf_min=%.2f", TR_KP, TR_KI, TR_DELTA, TR_CONF_MIN);
  ESP_LOGCONFIG(TAG, "  Sync fuzzy Hamming tolerance: %d bit(s)", SYNC_MAX_HAMMING);
  ESP_LOGCONFIG(TAG, "  Early-emit: >=%d agreeing bursts (max mismatch %.0f%%)", MIN_BURSTS_TO_EMIT, BURST_MAX_MISMATCH * 100.0f);
  ESP_LOGCONFIG(TAG, "  Single-burst emit: only if strictly valid, after >= %" PRIu32 " ms", SINGLE_BURST_MIN_MS);
  ESP_LOGCONFIG(TAG, "  Burst timeout (>=2 bursts): %" PRIu32 " ms", this->timeout_ms_.load(std::memory_order_relaxed));
  ESP_LOGCONFIG(TAG, "  Alert queue depth: %d, pending buffer max: %u", ALERT_Q_LEN, (unsigned) PENDING_MAX);
  ESP_LOGCONFIG(TAG, "  Alert triggers: %u, sync triggers: %u",
                (unsigned) this->alert_triggers_.size(), (unsigned) this->sync_triggers_.size());
}

void SAMEDecoder::feed_bytes(const std::vector<uint8_t> &data) {
  const size_t n = data.size() / 2;
  const int16_t *samples = reinterpret_cast<const int16_t *>(data.data());
  for (size_t i = 0; i < n; i++) {
    float v = (float) samples[i] * this->gain_;
    v = soft_clip_(v);
    if (v >  32767.0f) v =  32767.0f;
    if (v < -32768.0f) v = -32768.0f;
    this->feed_sample_((int16_t) v);
  }
}

// Effective timeout depends on how many bursts we have: a lone burst waits the
// longer SINGLE_BURST_MIN_MS floor so real repeats can arrive first; >=2 bursts
// use the normal (slider) timeout.
static inline uint32_t effective_timeout(int burst_idx, uint32_t slider_ms, uint32_t single_min_ms) {
  if (burst_idx <= 1)
    return slider_ms > single_min_ms ? slider_ms : single_min_ms;
  return slider_ms;
}

void SAMEDecoder::feed_sample_(int16_t s) {
  this->ring_[this->ring_pos_] = s;
  this->ring_pos_ = (this->ring_pos_ + 1) % RINGLEN;
  if (this->samples_seen_ < 0x7fffffff) this->samples_seen_++;

  this->phase_ += (PHASE_INC + this->w_off_);
  if (this->phase_ < 1.0f) {
    if (this->burst_idx_ >= 1 && this->burst_idx_ < 3 && this->last_burst_ms_ != 0) {
      uint32_t to = effective_timeout(this->burst_idx_, this->timeout_ms_.load(std::memory_order_relaxed), SINGLE_BURST_MIN_MS);
      if ((uint32_t) (millis() - this->last_burst_ms_) >= to) {
        ESP_LOGD(TAG, "Burst timeout (%d burst(s), %" PRIu32 " ms): flushing.", this->burst_idx_, to);
        this->vote_and_emit_(true);
        this->reset_capture_();
      }
    }
    return;
  }
  this->phase_ -= 1.0f;

  int s_now = (this->ring_pos_ - 1 + RINGLEN) % RINGLEN;

  int t_center = (s_now - CENTER_LAG + RINGLEN) % RINGLEN;
  int t_early  = (t_center - TR_DELTA + RINGLEN) % RINGLEN;
  int t_late   = (t_center + TR_DELTA) % RINGLEN;

  float em_c = goertzel_ring_end(this->ring_, RINGLEN, t_center, GWIN, COEFF_MARK);
  float es_c = goertzel_ring_end(this->ring_, RINGLEN, t_center, GWIN, COEFF_SPACE);
  bool bit = em_c > es_c;

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

    float dphi = TR_KP * e;
    if (dphi >  TR_DPHI_CLAMP) dphi =  TR_DPHI_CLAMP;
    if (dphi < -TR_DPHI_CLAMP) dphi = -TR_DPHI_CLAMP;
    this->phase_ += dphi;

    this->w_off_ += TR_KI * e;
    if (this->w_off_ >  TR_WOFF_CLAMP) this->w_off_ =  TR_WOFF_CLAMP;
    if (this->w_off_ < -TR_WOFF_CLAMP) this->w_off_ = -TR_WOFF_CLAMP;
  }

  this->emit_bit_(bit);

  if (this->burst_idx_ >= 1 && this->burst_idx_ < 3 && this->last_burst_ms_ != 0) {
    uint32_t to = effective_timeout(this->burst_idx_, this->timeout_ms_.load(std::memory_order_relaxed), SINGLE_BURST_MIN_MS);
    if ((uint32_t) (millis() - this->last_burst_ms_) >= to) {
      ESP_LOGD(TAG, "Burst timeout (%d burst(s), %" PRIu32 " ms): flushing.", this->burst_idx_, to);
      this->vote_and_emit_(true);
      this->reset_capture_();
    }
  }
}

void SAMEDecoder::reset_capture_() {
  this->phase_state_ = HUNT_SYNC;
  this->sync_shift_ = 0;
  this->cur_byte_ = 0;
  this->cur_nbits_ = 0;
  this->cur_burst_.clear();
  this->burst_idx_ = 0;
  this->plus_seen_ = false;
  this->tail_count_ = 0;
  this->last_burst_ms_ = 0;
  for (int i = 0; i < 3; i++) this->bursts_[i].clear();
}

bool SAMEDecoder::bursts_agree_(int count) {
  if (count < 2) return false;
  const std::string &a = this->bursts_[count - 1];
  const std::string &b = this->bursts_[count - 2];
  size_t overlap = std::min(a.size(), b.size());
  if (overlap == 0) return false;
  size_t diff = 0;
  for (size_t i = 0; i < overlap; i++)
    if (a[i] != b[i]) diff++;
  size_t longer = std::max(a.size(), b.size());
  diff += (longer - overlap);
  float frac = (float) diff / (float) longer;
  return frac <= BURST_MAX_MISMATCH;
}

void SAMEDecoder::finish_burst_() {
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
  this->plus_seen_ = false;
  this->tail_count_ = 0;
  this->last_burst_ms_ = millis();

  if (this->burst_idx_ >= 3) {
    this->vote_and_emit_(false);
    this->reset_capture_();
  } else if (this->burst_idx_ >= MIN_BURSTS_TO_EMIT && this->bursts_agree_(this->burst_idx_)) {
    ESP_LOGD(TAG, "Early emit: %d agreeing bursts.", this->burst_idx_);
    this->vote_and_emit_(false);
    this->reset_capture_();
  } else {
    this->phase_state_ = HUNT_SYNC;
    this->sync_shift_ = 0;
    this->cur_byte_ = 0;
    this->cur_nbits_ = 0;
  }
}

void SAMEDecoder::emit_bit_(bool bit) {
  if (this->phase_state_ == HUNT_SYNC) {
    this->sync_shift_ = (this->sync_shift_ >> 1) | ((uint32_t) (bit ? 1u : 0u) << 31);

    uint32_t diffbits = this->sync_shift_ ^ SYNC_ZCZC;
    int hamming = __builtin_popcount(diffbits);
    if (hamming <= SYNC_MAX_HAMMING) {
      ESP_LOGD(TAG, "ZCZC sync found (hamming=%d). Byte-aligned capture starting.", hamming);
      uint32_t p = this->sync_pending_.load(std::memory_order_relaxed);
      if (p < 8) this->sync_pending_.store(p + 1, std::memory_order_release);
      this->phase_state_ = CAPTURE;
      this->cur_burst_ = "ZCZC";
      this->cur_byte_ = 0;
      this->cur_nbits_ = 0;
      this->plus_seen_ = false;
      this->tail_count_ = 0;
    }
    return;
  }

  this->cur_byte_ >>= 1;
  if (bit) this->cur_byte_ |= 0x80;
  this->cur_nbits_++;

  if (this->cur_nbits_ != 8)
    return;

  char c = (char) this->cur_byte_;
  this->cur_byte_ = 0;
  this->cur_nbits_ = 0;

  if (!is_valid_same_char(c)) {
    if (this->cur_burst_.size() >= 4) {
      this->finish_burst_();
    } else {
      this->phase_state_ = HUNT_SYNC;
      this->sync_shift_ = 0;
      this->cur_byte_ = 0;
      this->cur_nbits_ = 0;
      this->plus_seen_ = false;
      this->tail_count_ = 0;
    }
    return;
  }

  this->cur_burst_ += c;

  if (!this->plus_seen_) {
    if (c == '+') {
      this->plus_seen_ = true;
      this->tail_count_ = 0;
    }
  } else {
    this->tail_count_++;
    if (c == '-' && this->tail_count_ >= TAIL_MIN) {
      this->finish_burst_();
      return;
    } else if (this->tail_count_ >= TAIL_COMPLETE) {
      ESP_LOGD(TAG, "Header end fallback: fixed tail complete without closing dash.");
      this->finish_burst_();
      return;
    }
  }

  if (this->cur_burst_.size() >= (size_t) MAX_HEADER_BYTES) {
    this->finish_burst_();
  }
}

// Strict structural + known-code validity check for a single header string.
// Used to gate single-burst emits (hybrid option c): a lone burst may only be
// emitted if it is a complete, well-formed header AND its event code is known.
// Per option (x), ANY known code qualifies (tests included).
bool SAMEDecoder::header_is_strictly_valid_(const std::string &header) {
  if (header.rfind("ZCZC", 0) != 0) return false;
  if (header.find('+') == std::string::npos) return false;   // must have the '+' tail marker
  SameAlert probe;
  if (!this->parse_header_(header, probe)) return false;
  if (probe.event_code.size() != 3) return false;
  if (!this->is_known_code_(probe.event_code)) return false; // known code required
  if (probe.originator.size() != 3) return false;            // ORG must be 3 chars
  return true;
}

void SAMEDecoder::vote_and_emit_(bool from_timeout) {
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
  std::string header = voted.substr(z);

  // Count how many bursts actually contributed to this vote.
  int collected = 0;
  for (int i = 0; i < 3; i++) if (!this->bursts_[i].empty()) collected++;

  // Hybrid (c): if this emit is driven by a timeout with only ONE burst, apply
  // the strict validity gate. A single burst is only trustworthy enough to emit
  // if it is a complete, well-formed header with a known event code (option x:
  // any known code, tests included). Multi-burst emits (voting/agreement) keep
  // their existing behavior.
  if (from_timeout && collected < 2) {
    if (!this->header_is_strictly_valid_(header)) {
      ESP_LOGW(TAG, "Single-burst timeout emit rejected (failed strict validity): '%s'",
               sanitize_ascii(header).c_str());
      return;
    }
    ESP_LOGI(TAG, "Single-burst emit accepted (strictly valid).");
  }

  SameAlert alert;
  if (this->parse_header_(header, alert))
    this->publish_alert_(alert);
}

bool SAMEDecoder::is_known_code_(const std::string &code) {
  return SAME_EVENT_CODES.find(code) != SAME_EVENT_CODES.end();
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
    size_t plus = parts[i].find('+');
    if (plus != std::string::npos) {
      std::string last_area = parts[i].substr(0, plus);
      if (!last_area.empty()) {
        if (!areas.empty()) areas += ",";
        areas += last_area;
      }
      break;
    }
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
  SameAlert clean = a;
  clean.raw_header = sanitize_ascii(a.raw_header);

  ESP_LOGI(TAG, "Decoded SAME: %s (%s) areas=%s",
           clean.event_name.c_str(), clean.event_code.c_str(), clean.areas_csv.c_str());

  uint32_t tail = this->q_tail_.load(std::memory_order_relaxed);
  uint32_t next = (tail + 1) % ALERT_Q_LEN;
  if (next == this->q_head_.load(std::memory_order_acquire)) {
    ESP_LOGW(TAG, "Alert queue full; dropping alert (loop starved?).");
    return;
  }
  this->alert_queue_[tail] = clean;
  this->q_tail_.store(next, std::memory_order_release);
}

void SAMEDecoder::dispatch_alert_(const SameAlert &a) {
  this->last_ = a;
  this->decode_count_++;

  if (this->decode_count_sensor_ != nullptr)
    this->decode_count_sensor_->publish_state((float) this->decode_count_);
  if (this->last_raw_sensor_ != nullptr)
    this->last_raw_sensor_->publish_state(this->last_.raw_header);

  this->deliver_or_buffer_(a);
}

void SAMEDecoder::deliver_or_buffer_(const SameAlert &a) {
  if (this->api_connected_) {
    this->last_ = a;
    for (auto *t : this->alert_triggers_)
      t->trigger();
    return;
  }

  if (this->pending_.size() >= PENDING_MAX) {
    ESP_LOGW(TAG, "Pending alert buffer full (%u); dropping oldest.", (unsigned) PENDING_MAX);
    this->pending_.erase(this->pending_.begin());
  }
  this->pending_.push_back(a);
  ESP_LOGW(TAG, "API offline; buffered alert %s (%u pending).",
           a.event_code.c_str(), (unsigned) this->pending_.size());
}

void SAMEDecoder::flush_pending_() {
  if (this->pending_.empty())
    return;
  ESP_LOGI(TAG, "API reconnected; flushing %u buffered alert(s).", (unsigned) this->pending_.size());
  for (auto &a : this->pending_) {
    this->last_ = a;
    for (auto *t : this->alert_triggers_)
      t->trigger();
  }
  this->pending_.clear();
}

}  // namespace same_decoder
}  // namespace esphome
