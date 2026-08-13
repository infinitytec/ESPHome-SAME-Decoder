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
  ESP_LOGCONFIG(TAG, "  Timing loop: Kp=%.3f Ki=%.4f delta=%d conf_min=%.2f", TR_KP, TR_KI, TR_DELTA, TR_CONF_MIN);
  ESP_LOGCONFIG(TAG, "  Sync: ZCZC fuzzy Hamming <= %d; fallback ZC- exact (Hamming 0)", SYNC_MAX_HAMMING);
  ESP_LOGCONFIG(TAG, "  Collect all 3 bursts; early-emit at %d agreeing", MIN_BURSTS_TO_EMIT);
  ESP_LOGCONFIG(TAG, "  Same-message match: ORG/EEE fuzzy, area+timing EXACT");
  ESP_LOGCONFIG(TAG, "  Dedup: session-based; immediate-dup guard %" PRIu32 " ms", IMMEDIATE_DUP_MS);
  ESP_LOGCONFIG(TAG, "  Re-arm sync cleanly after each burst (catch tight repeats).");
  ESP_LOGCONFIG(TAG, "  Weak-evidence gate: structural only (UNKNOWN codes fire).");
  ESP_LOGCONFIG(TAG, "  Single-burst emit: structurally valid only, after >= %" PRIu32 " ms", SINGLE_BURST_MIN_MS);
  ESP_LOGCONFIG(TAG, "  Burst timeout (>=2 bursts): %" PRIu32 " ms", this->timeout_ms_.load(std::memory_order_relaxed));
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
        this->vote_and_emit_(true, this->fallback_sync_used_);
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
      this->vote_and_emit_(true, this->fallback_sync_used_);
      this->reset_capture_();
    }
  }
}

// Re-arm the sync hunter to a CLEAN state so a tightly-spaced repeat burst
// (~1 s after the previous one) is reliably re-acquired. This clears the byte
// assembler AND neutralizes the timing-recovery bias (w_off_) and phase that the
// just-finished burst left locked - stale lock was causing the 2nd/3rd repeats
// to be missed (every message decoded on burst 0 only, then timed out).
void SAMEDecoder::rearm_sync_() {
  this->phase_state_ = HUNT_SYNC;
  this->sync_shift_ = 0;
  this->cur_byte_ = 0;
  this->cur_nbits_ = 0;
  this->cur_burst_.clear();
  this->plus_seen_ = false;
  this->tail_count_ = 0;
  this->w_off_ = 0.0f;
  this->phase_ = 0.0f;
}

void SAMEDecoder::reset_capture_() {
  this->rearm_sync_();
  this->burst_idx_ = 0;
  this->last_burst_ms_ = 0;
  this->early_emitted_ = false;
  this->fallback_sync_used_ = false;
  this->session_emitted_header_.clear();
  for (int i = 0; i < 3; i++) this->bursts_[i].clear();
}

void SAMEDecoder::begin_new_capture_(bool fallback) {
  this->phase_state_ = CAPTURE;
  this->cur_burst_ = "ZCZC";
  this->cur_byte_ = 0;
  this->cur_nbits_ = 0;
  this->plus_seen_ = false;
  this->tail_count_ = 0;
  if (fallback)
    this->cur_burst_ += '-';
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

bool SAMEDecoder::fuzzy_equal_(const std::string &a, const std::string &b) {
  size_t overlap = std::min(a.size(), b.size());
  size_t longer = std::max(a.size(), b.size());
  if (longer == 0) return true;
  size_t diff = 0;
  for (size_t i = 0; i < overlap; i++)
    if (a[i] != b[i]) diff++;
  diff += (longer - overlap);
  return ((float) diff / (float) longer) <= FIELD_FUZZ;
}

bool SAMEDecoder::same_message_as_current_(const std::string &new_burst) {
  if (this->burst_idx_ == 0) return true;
  const std::string &ref = this->bursts_[0];

  SameAlert ra, rb;
  bool pa = this->parse_header_(ref, ra);
  bool pb = this->parse_header_(new_burst, rb);

  if (!pa || !pb) {
    size_t cmp = std::min({(size_t) 12, ref.size(), new_burst.size()});
    if (cmp < 8) return true;
    size_t diff = 0;
    for (size_t i = 0; i < cmp; i++)
      if (ref[i] != new_burst[i]) diff++;
    return ((float) diff / (float) cmp) <= FIELD_FUZZ;
  }

  if (!this->fuzzy_equal_(ra.originator, rb.originator)) return false;
  if (!this->fuzzy_equal_(ra.event_code, rb.event_code)) return false;
  if (ra.areas_csv != rb.areas_csv) return false;
  if (ra.timing != rb.timing) return false;
  return true;
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

  std::string this_burst = this->cur_burst_;

  if (this->burst_idx_ >= 1 && !this->same_message_as_current_(this_burst)) {
    ESP_LOGD(TAG, "New differing burst mid-collection: finalizing old session, starting new.");
    this->reset_capture_();
    this->bursts_[0] = this_burst;
    this->burst_idx_ = 1;
    this->last_burst_ms_ = millis();
    this->rearm_sync_();
    return;
  }

  if (this->burst_idx_ < 3)
    this->bursts_[this->burst_idx_] = this_burst;
  this->burst_idx_++;
  this->last_burst_ms_ = millis();

  if (this->burst_idx_ >= 3) {
    this->vote_and_emit_(false, this->fallback_sync_used_);
    this->reset_capture_();
    return;
  }

  if (this->burst_idx_ >= MIN_BURSTS_TO_EMIT && !this->early_emitted_ && this->bursts_agree_(this->burst_idx_)) {
    ESP_LOGD(TAG, "Early emit: %d agreeing bursts (still collecting burst 3).", this->burst_idx_);
    this->vote_and_emit_(false, this->fallback_sync_used_);
    this->early_emitted_ = true;
  }

  // Re-arm cleanly for the NEXT tightly-spaced repeat burst.
  this->rearm_sync_();
}

void SAMEDecoder::emit_bit_(bool bit) {
  if (this->phase_state_ == HUNT_SYNC) {
    this->sync_shift_ = (this->sync_shift_ >> 1) | ((uint32_t) (bit ? 1u : 0u) << 31);

    uint32_t diffbits = this->sync_shift_ ^ SYNC_ZCZC;
    int hamming = __builtin_popcount(diffbits);
    bool primary = (hamming <= SYNC_MAX_HAMMING);

    bool fallback = false;
    if (!primary) {
      fallback = ((this->sync_shift_ & MASK24) == SYNC_ZC_DASH_24);
    }

    if (primary || fallback) {
      bool fb = fallback && !primary;
      if (this->burst_idx_ == 0)
        this->fallback_sync_used_ = fb;
      ESP_LOGD(TAG, "%s sync found%s. Byte-aligned capture starting.",
               fb ? "Fallback ZC-" : "ZCZC",
               fb ? " (first ZC assumed lost)" : "");
      uint32_t p = this->sync_pending_.load(std::memory_order_relaxed);
      if (p < 8) this->sync_pending_.store(p + 1, std::memory_order_release);
      this->begin_new_capture_(fb);
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
      this->rearm_sync_();
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

// Weak-evidence gate: structural checks only. The "must be a known event code"
// requirement has been REMOVED so an UNKNOWN-but-structurally-valid header
// (e.g. a rare/new code, or a national code not yet in the table) still fires -
// it decodes with the raw code as its name and "Unknown" severity. Structural
// checks are retained so garbled noise cannot fire as a bogus alert.
bool SAMEDecoder::header_is_strictly_valid_(const std::string &header) {
  if (header.rfind("ZCZC", 0) != 0) return false;
  if (header.find('+') == std::string::npos) return false;
  SameAlert probe;
  if (!this->parse_header_(header, probe)) return false;
  if (probe.event_code.size() != 3) return false;
  if (probe.originator.size() != 3) return false;
  return true;
}

void SAMEDecoder::vote_and_emit_(bool from_timeout, bool fallback_synced) {
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

  int collected = 0;
  for (int i = 0; i < 3; i++) if (!this->bursts_[i].empty()) collected++;

  bool weak = (from_timeout && collected < 2) || fallback_synced;
  if (weak) {
    if (!this->header_is_strictly_valid_(header)) {
      ESP_LOGW(TAG, "Weak-evidence emit rejected (failed structural validity): '%s'",
               sanitize_ascii(header).c_str());
      return;
    }
    ESP_LOGI(TAG, "Weak-evidence emit accepted (structurally valid).");
  }

  SameAlert alert;
  if (!this->parse_header_(header, alert))
    return;

  if (!this->is_known_code_(alert.event_code)) {
    ESP_LOGW(TAG, "Unknown event code '%s' - firing as Unknown severity.", alert.event_code.c_str());
  }

  uint32_t now = millis();

  if (!this->session_emitted_header_.empty()) {
    if (this->session_emitted_header_ == header) {
      ESP_LOGD(TAG, "Same-session duplicate header suppressed (%s).", alert.event_code.c_str());
      return;
    }
    ESP_LOGI(TAG, "Same-session consensus changed; reissuing corrected alert.");
  } else {
    bool immediate_same = (!this->last_global_header_.empty()) &&
                          (this->last_global_header_ == header) &&
                          ((uint32_t) (now - this->last_global_ms_) <= IMMEDIATE_DUP_MS);
    if (immediate_same) {
      ESP_LOGD(TAG, "Immediate cross-session duplicate suppressed (%s within %" PRIu32 " ms).",
               alert.event_code.c_str(), IMMEDIATE_DUP_MS);
      this->session_emitted_header_ = header;
      this->last_global_header_ = header;
      this->last_global_ms_ = now;
      return;
    }
  }

  this->session_emitted_header_ = header;
  this->last_global_header_ = header;
  this->last_global_ms_ = now;
  this->publish_alert_(alert);
}

bool SAMEDecoder::is_known_code_(const std::string &code) {
  return SAME_EVENT_CODES.find(code) != SAME_EVENT_CODES.end();
}

std::string SAMEDecoder::describe_(const std::string &code) {
  auto it = SAME_EVENT_CODES.find(code);
  if (it != SAME_EVENT_CODES.end())
    return it->second.name;
  return code;   // unknown code -> raw code as the name
}

std::string SAMEDecoder::severity_for_(const std::string &code) {
  auto it = SAME_EVENT_CODES.find(code);
  if (it != SAME_EVENT_CODES.end())
    return it->second.severity;
  return "Unknown";
}

std::string SAMEDecoder::make_id_(const SameAlert &a) {
  std::string key = a.event_code + "|" + a.areas_csv + "|" + a.timing;
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
  std::string timing;
  for (size_t i = 3; i < parts.size(); i++) {
    size_t plus = parts[i].find('+');
    if (plus != std::string::npos) {
      std::string last_area = parts[i].substr(0, plus);
      if (!last_area.empty()) {
        if (!areas.empty()) areas += ",";
        areas += last_area;
      }
      timing = parts[i].substr(plus);
      if (i + 1 < parts.size())
        timing += "-" + parts[i + 1];
      break;
    }
    if (!areas.empty()) areas += ",";
    areas += parts[i];
  }
  out.areas_csv = areas;
  out.timing = timing;

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
