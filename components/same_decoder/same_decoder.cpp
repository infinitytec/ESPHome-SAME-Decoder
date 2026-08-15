// components/same_decoder/same_decoder.cpp
// T0 preamble-lock (PRIMARY) + ALWAYS-WARM tiers + strong post-emit recovery
// + T0 acquisition instrumentation (DEBUG diagnostics only)
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

void SAMEDecoder::compute_coeffs_() {
  const float sr = (float) this->sample_rate_;
  const float mark_hz  = 2083.333333f + this->freq_offset_hz_;
  const float space_hz = 1562.5f      + this->freq_offset_hz_;
  this->coeff_mark_  = 2.0f * std::cos(2.0f * (float) M_PI * mark_hz  / sr);
  this->coeff_space_ = 2.0f * std::cos(2.0f * (float) M_PI * space_hz / sr);

  const float baud = 520.0f + 5.0f / 6.0f;
  this->samples_per_bit_ = sr / baud;
  this->phase_inc_ = 1.0f / this->samples_per_bit_;
  this->tr_woff_clamp_ = 0.002f * this->phase_inc_;

  ESP_LOGCONFIG(TAG, "Coeffs: sr=%" PRIu32 " mark=%.6f space=%.6f spb=%.3f",
                this->sample_rate_, this->coeff_mark_, this->coeff_space_, this->samples_per_bit_);
}

void SAMEDecoder::set_sample_rate(uint32_t rate) {
  this->sample_rate_ = rate;
  this->compute_coeffs_();
}

void SAMEDecoder::update_agc_(float mag) {
  if (!this->agc_enable_)
    return;
  if (mag > this->agc_peak_)
    this->agc_peak_ += AGC_ATTACK * (mag - this->agc_peak_);
  else
    this->agc_peak_ += AGC_RELEASE * (mag - this->agc_peak_);
  if (this->agc_peak_ < 1.0f)
    this->agc_peak_ = 1.0f;
  float desired = this->agc_target_ / this->agc_peak_;
  if (desired < this->agc_min_gain_) desired = this->agc_min_gain_;
  if (desired > this->agc_max_gain_) desired = this->agc_max_gain_;
  this->agc_gain_ += 0.001f * (desired - this->agc_gain_);
}

void SAMEDecoder::setup() {
  this->compute_coeffs_();
  ESP_LOGCONFIG(TAG, "SAME decoder ready (T0 preamble-lock=%s + ALWAYS-WARM tiers, gain=%.1f, agc=%s, ab_req=%s).",
                this->preamble_lock_ ? "on" : "off",
                this->gain_,
                this->agc_enable_ ? "on" : "off",
                this->ab_required_ ? "on" : "off");
  this->reset_capture_();
  this->reset_preamble_detector_();
  this->last_feed_ms_.store(millis(), std::memory_order_relaxed);
}

void SAMEDecoder::set_api_connected(bool connected) {
  if (connected != this->api_connected_) {
    this->api_connected_ = connected;
    this->api_connected_changed_.store(true, std::memory_order_release);
  }
}

uint32_t SAMEDecoder::ms_since_last_feed() const {
  return (uint32_t) (millis() - this->last_feed_ms_.load(std::memory_order_relaxed));
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

  uint32_t eom = this->eom_pending_.exchange(0, std::memory_order_acq_rel);
  while (eom > 0) {
    for (auto *t : this->eom_triggers_)
      t->trigger();
    eom--;
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
  ESP_LOGCONFIG(TAG, "SAME Decoder (T0 preamble-lock + ALWAYS-WARM tiers):");
  ESP_LOGCONFIG(TAG, "  Sample rate: %" PRIu32 " Hz", this->sample_rate_);
  ESP_LOGCONFIG(TAG, "  Base gain: %.1f  AGC: %s", this->gain_, this->agc_enable_ ? "on" : "off");
  ESP_LOGCONFIG(TAG, "  Freq offset: %.1f Hz", this->freq_offset_hz_);
  ESP_LOGCONFIG(TAG, "  Samples/bit: %.2f  Goertzel window: %d", this->samples_per_bit_, GWIN);
  ESP_LOGCONFIG(TAG, "  Coeffs mark=%.6f space=%.6f", this->coeff_mark_, this->coeff_space_);
  ESP_LOGCONFIG(TAG, "  T0 preamble lock: %s (min density %.2f, min bits %" PRIu32 ", acq gain %.1f)",
                this->preamble_lock_ ? "on" : "off",
                this->preamble_min_density_.load(std::memory_order_relaxed),
                this->preamble_min_bits_.load(std::memory_order_relaxed),
                this->preamble_acq_gain_.load(std::memory_order_relaxed));
  ESP_LOGCONFIG(TAG, "  AB preamble required: %s (min match %d)", this->ab_required_ ? "yes" : "no", AB_MIN_MATCH);
  ESP_LOGCONFIG(TAG, "  Timeout (>=2 bursts): %" PRIu32 " ms", this->timeout_ms_.load(std::memory_order_relaxed));
  ESP_LOGCONFIG(TAG, "  Single-burst min: %" PRIu32 " ms", this->single_burst_min_ms_);
  ESP_LOGCONFIG(TAG, "  Post-emit dead-time: %" PRIu32 " ms", this->post_emit_dead_ms_);
  ESP_LOGCONFIG(TAG, "  Alert triggers: %u  sync triggers: %u  eom triggers: %u",
                (unsigned) this->alert_triggers_.size(), (unsigned) this->sync_triggers_.size(),
                (unsigned) this->eom_triggers_.size());
}

void SAMEDecoder::reprime_detector_() {
  ESP_LOGD(TAG, "Re-priming detector (explicit).");
  this->reset_capture_();
  for (int i = 0; i < RINGLEN; i++) this->ring_[i] = 0;
  this->ring_pos_ = 0;
  this->phase_ = 0.0f;
  this->w_off_ = 0.0f;
  this->samples_seen_ = 0;
  this->ab_byte_ = 0;
  this->ab_nbits_ = 0;
  this->ab_match_count_ = 0;
  this->eom_n_count_ = 0;
  this->reset_preamble_detector_();
}

// ---- T0 preamble lock ---------------------------------------------------

void SAMEDecoder::reset_preamble_detector_() {
  for (int i = 0; i < PRE_WIN; i++) this->pre_hist_[i] = 0;
  this->pre_hist_pos_ = 0;
  this->pre_trans_count_ = 0;
  this->pre_fill_ = 0;
  this->pre_have_last_ = false;
  this->pre_last_bit_ = false;
  this->pre_run_bits_ = 0;
  this->preamble_locked_ = false;
}

// Phase-independent preamble detector. Called for every demodulated bit while
// hunting. Measures transition density over a sliding window; a 0xAB tone is
// near-alternating, so density -> ~1.0. Tolerates dropped/mistimed bytes because
// it never assumes byte alignment.
void SAMEDecoder::update_preamble_detector_(bool bit) {
  if (!this->preamble_lock_)
    return;

  uint8_t trans = 0;
  if (this->pre_have_last_)
    trans = (bit != this->pre_last_bit_) ? 1 : 0;
  this->pre_last_bit_ = bit;
  this->pre_have_last_ = true;

  // Slide the window: subtract the leaving flag, add the entering flag.
  this->pre_trans_count_ -= this->pre_hist_[this->pre_hist_pos_];
  this->pre_hist_[this->pre_hist_pos_] = trans;
  this->pre_trans_count_ += trans;
  this->pre_hist_pos_ = (this->pre_hist_pos_ + 1) % PRE_WIN;
  if (this->pre_fill_ < PRE_WIN) this->pre_fill_++;

  // Need a reasonably full window before trusting density.
  if (this->pre_fill_ < 16)
    return;

  float density = (float) this->pre_trans_count_ / (float) this->pre_fill_;
  float min_density = this->preamble_min_density_.load(std::memory_order_relaxed);
  uint32_t min_bits = this->preamble_min_bits_.load(std::memory_order_relaxed);

  // --- INSTRUMENTATION: throttled density ramp (max ~5/sec) ---
  uint32_t now_dbg = millis();
  if (density >= 0.5f && (now_dbg - this->pre_dbg_last_ms_) >= 200) {
    this->pre_dbg_last_ms_ = now_dbg;
    ESP_LOGD(TAG, "T0-ramp density=%.2f run_bits=%" PRIu32 " fill=%d (need dens>=%.2f bits>=%" PRIu32 ")",
             density, this->pre_run_bits_, this->pre_fill_, min_density, min_bits);
  }

  if (density >= min_density) {
    if (this->pre_run_bits_ < 0x7fffffff) this->pre_run_bits_++;
    if (!this->preamble_locked_ && this->pre_run_bits_ >= min_bits) {
      this->enter_preamble_lock_();
    }
  } else {
    // A brief dropout shouldn't instantly kill the run: allow a little slack so
    // a few missed/mistimed 0xAB bytes don't break the lock.
    if (this->pre_run_bits_ >= 4)
      this->pre_run_bits_ -= 4;
    else
      this->pre_run_bits_ = 0;
  }
}

// T0 fires: we've positively identified a preamble tone. Reset the PLL cleanly
// and enter fast-acquire so the bit clock converges on the AB run *before* ZCZC.
void SAMEDecoder::enter_preamble_lock_() {
  this->preamble_locked_ = true;
  ESP_LOGD(TAG, "T0 preamble lock (density>=%.2f over %" PRIu32 " bits). Priming clock.",
           this->preamble_min_density_.load(std::memory_order_relaxed),
           this->pre_run_bits_);

  // Clean PLL start so the alternating tone drives convergence, not stale noise.
  this->phase_ = 0.0f;
  this->w_off_ = 0.0f;

  // Enter fast-acquire: temporarily boost timing-loop authority for a short bit
  // budget, then fall back to normal tracking gains.
  this->fast_acquire_ = true;
  this->fast_acquire_bits_left_ = FAST_ACQUIRE_BITS;

  // --- INSTRUMENTATION: start lock->sync latency measurement ---
  this->t0_lock_bit_marker_ = this->hunt_bit_counter_;
  this->t0_lock_pending_ = true;

  // Earliest possible user feedback: light the decode LED at the preamble.
  this->fire_sync_once_();
}

// Debounced sync fire: one on_sync per acquisition, not once per preamble byte.
void SAMEDecoder::fire_sync_once_() {
  uint32_t p = this->sync_pending_.load(std::memory_order_relaxed);
  if (p < 8) this->sync_pending_.store(p + 1, std::memory_order_release);
}

void SAMEDecoder::fire_eom_() {
  uint32_t p = this->eom_pending_.load(std::memory_order_relaxed);
  if (p < 8) this->eom_pending_.store(p + 1, std::memory_order_release);
}

// -------------------------------------------------------------------------

void SAMEDecoder::feed_bytes(const std::vector<uint8_t> &data) {
  uint32_t now = millis();
  uint32_t prev = this->last_feed_ms_.exchange(now, std::memory_order_relaxed);
  if (prev != 0 && (uint32_t) (now - prev) >= FEED_GAP_MS) {
    this->w_off_ = 0.0f;
    this->phase_ = 0.0f;
    this->ab_byte_ = 0;
    this->ab_nbits_ = 0;
    this->ab_match_count_ = 0;
  }

  const size_t n = data.size() / 2;
  const int16_t *samples = reinterpret_cast<const int16_t *>(data.data());
  for (size_t i = 0; i < n; i++) {
    float v = (float) samples[i] * this->gain_;
    if (this->agc_enable_)
      v *= this->agc_gain_;
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
  float mag = std::fabs((float) s);
  this->update_agc_(mag);

  this->env_slow_ += ENV_ALPHA * (mag - this->env_slow_);
  if (this->was_idle_) {
    if (mag > ENV_SILENCE * ENV_RISE_MULT) {
      this->was_idle_ = false;
      this->idle_edge_samples_ = 48000 / 2;   // ~0.5 s window for Tier-4
    }
  } else {
    if (this->env_slow_ < ENV_SILENCE)
      this->was_idle_ = true;
  }
  if (this->idle_edge_samples_ > 0)
    this->idle_edge_samples_--;

  if (this->last_emit_ms_ != 0 &&
      (uint32_t) (millis() - this->last_emit_ms_) < this->post_emit_dead_ms_) {
    // --- INSTRUMENTATION: flag when real audio is being swallowed by dead-time ---
    if (mag > ENV_SILENCE * ENV_RISE_MULT) {
      uint32_t now_dbg = millis();
      if ((now_dbg - this->pre_dbg_last_ms_) >= 200) {
        this->pre_dbg_last_ms_ = now_dbg;
        uint32_t remain = this->post_emit_dead_ms_ - (uint32_t)(now_dbg - this->last_emit_ms_);
        ESP_LOGD(TAG, "Post-emit dead-time DROPPING active audio (mag=%.0f, %" PRIu32 " ms left).",
                 mag, remain);
      }
    }
    return;
  }

  this->ring_[this->ring_pos_] = s;
  this->ring_pos_ = (this->ring_pos_ + 1) % RINGLEN;
  if (this->samples_seen_ < 0x7fffffff) this->samples_seen_++;

  // Gentle continuous decay of timing offset while hunting
  if (this->phase_state_ == HUNT_SYNC) {
    this->w_off_ *= 0.9997f;
    if (std::fabs(this->w_off_) < 1e-8f)
      this->w_off_ = 0.0f;
  }

  this->phase_ += (this->phase_inc_ + this->w_off_);
  if (this->phase_ < 1.0f) {
    if (this->burst_idx_ >= 1 && this->burst_idx_ < 3 && this->last_burst_ms_ != 0) {
      uint32_t to = effective_timeout(this->burst_idx_,
                                      this->timeout_ms_.load(std::memory_order_relaxed),
                                      this->single_burst_min_ms_);
      if ((uint32_t) (millis() - this->last_burst_ms_) >= to) {
        ESP_LOGD(TAG, "Burst timeout (%d): flushing.", this->burst_idx_);
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

  float em_c = goertzel_ring_end(this->ring_, RINGLEN, t_center, GWIN, this->coeff_mark_);
  float es_c = goertzel_ring_end(this->ring_, RINGLEN, t_center, GWIN, this->coeff_space_);
  bool bit = em_c > es_c;

  bool primed = this->samples_seen_ >= (uint32_t) (CENTER_LAG + TR_DELTA + GWIN + 2);
  if (primed) {
    float em_e = goertzel_ring_end(this->ring_, RINGLEN, t_early, GWIN, this->coeff_mark_);
    float es_e = goertzel_ring_end(this->ring_, RINGLEN, t_early, GWIN, this->coeff_space_);
    float em_l = goertzel_ring_end(this->ring_, RINGLEN, t_late,  GWIN, this->coeff_mark_);
    float es_l = goertzel_ring_end(this->ring_, RINGLEN, t_late,  GWIN, this->coeff_space_);

    float d_early = std::fabs(em_e - es_e);
    float d_late  = std::fabs(em_l - es_l);
    float d_center = std::fabs(em_c - es_c);
    float e_center = em_c + es_c;
    const float eps = 1e-9f;
    float e_raw = (d_late - d_early) / (d_late + d_early + eps);
    float conf  = d_center / (e_center + eps);
    float e = (conf >= TR_CONF_MIN) ? e_raw : 0.0f;

    // ---- T0 fast-acquire: boost loop authority briefly after preamble lock ----
    float kp = TR_KP;
    float ki = TR_KI;
    float woff_clamp = this->tr_woff_clamp_;
    if (this->fast_acquire_) {
      float g = this->preamble_acq_gain_.load(std::memory_order_relaxed);
      kp *= g;
      ki *= g;
      woff_clamp *= g;   // allow a wider pull-in range while acquiring
      if (this->fast_acquire_bits_left_ > 0)
        this->fast_acquire_bits_left_--;
      if (this->fast_acquire_bits_left_ == 0)
        this->fast_acquire_ = false;
    }

    float dphi = kp * e;
    if (dphi >  TR_DPHI_CLAMP) dphi =  TR_DPHI_CLAMP;
    if (dphi < -TR_DPHI_CLAMP) dphi = -TR_DPHI_CLAMP;
    this->phase_ += dphi;

    this->w_off_ += ki * e;
    if (this->w_off_ >  woff_clamp) this->w_off_ =  woff_clamp;
    if (this->w_off_ < -woff_clamp) this->w_off_ = -woff_clamp;
  }

  this->emit_bit_(bit);

  if (this->burst_idx_ >= 1 && this->burst_idx_ < 3 && this->last_burst_ms_ != 0) {
    uint32_t to = effective_timeout(this->burst_idx_,
                                    this->timeout_ms_.load(std::memory_order_relaxed),
                                    this->single_burst_min_ms_);
    if ((uint32_t) (millis() - this->last_burst_ms_) >= to) {
      ESP_LOGD(TAG, "Burst timeout (%d): flushing.", this->burst_idx_);
      this->vote_and_emit_(true, this->fallback_sync_used_);
      this->reset_capture_();
    }
  }
}

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
  this->eom_n_count_ = 0;
}

void SAMEDecoder::reset_capture_() {
  this->rearm_sync_();
  this->burst_idx_ = 0;
  this->last_burst_ms_ = 0;
  this->early_emitted_ = false;
  this->fallback_sync_used_ = false;
  this->session_emitted_header_.clear();
  for (int i = 0; i < 3; i++) this->bursts_[i].clear();
  this->ab_byte_ = 0;
  this->ab_nbits_ = 0;
  this->ab_match_count_ = 0;
  this->eom_n_count_ = 0;
}

bool SAMEDecoder::ab_preamble_ok_() const {
  return this->ab_match_count_ >= AB_MIN_MATCH;
}

void SAMEDecoder::begin_new_capture_(int preamble_len, bool fallback) {
  this->phase_state_ = CAPTURE;
  this->reset_preamble_detector_();   // T0: clear tone tracker on capture start
  if (preamble_len >= 4)
    this->cur_burst_ = "ZCZC";
  else if (preamble_len == 3)
    this->cur_burst_ = "ZCZ";
  else
    this->cur_burst_ = "ZC";
  this->cur_byte_ = 0;
  this->cur_nbits_ = 0;
  this->plus_seen_ = false;
  this->tail_count_ = 0;
  this->eom_n_count_ = 0;
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
  return ((float) diff / (float) longer) <= BURST_MAX_MISMATCH;
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
  ESP_LOGD(TAG, "Burst %d ascii: '%s'", this->burst_idx_, sanitize_ascii(this->cur_burst_).c_str());

  std::string this_burst = this->cur_burst_;

  if (this->burst_idx_ >= 1 && !this->same_message_as_current_(this_burst)) {
    ESP_LOGD(TAG, "Differing burst mid-collection: finalising old, starting new.");
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
    ESP_LOGD(TAG, "Early emit on %d agreeing bursts.", this->burst_idx_);
    this->vote_and_emit_(false, this->fallback_sync_used_);
    this->early_emitted_ = true;
  }

  this->rearm_sync_();
}

void SAMEDecoder::emit_bit_(bool bit) {
  if (this->phase_state_ == HUNT_SYNC) {
    // ---- T0: feed the phase-independent preamble detector every hunt bit ----
    this->update_preamble_detector_(bit);
    this->hunt_bit_counter_++;   // INSTRUMENTATION: monotonic hunt-bit count

    this->sync_shift_ = (this->sync_shift_ >> 1) | ((uint32_t) (bit ? 1u : 0u) << 31);

    this->ab_byte_ = (this->ab_byte_ >> 1) | (bit ? 0x80 : 0);
    this->ab_nbits_++;
    if (this->ab_nbits_ >= 8) {
      this->ab_nbits_ = 0;
      if (this->ab_byte_ == 0xAB) {
        if (this->ab_match_count_ < 32) this->ab_match_count_++;
      } else if (this->ab_match_count_ > 0) {
        this->ab_match_count_--;
      }
    }

    if (this->ab_required_ && !this->ab_preamble_ok_())
      return;

    // Strong tiers always armed
    uint32_t diffbits = this->sync_shift_ ^ SYNC_ZCZC;
    int hamming = __builtin_popcount(diffbits);
    bool t1 = (hamming <= SYNC_MAX_HAMMING);

    bool t1b = false;
    if (!t1) {
      uint8_t b1 = (uint8_t) ((this->sync_shift_ >> 8) & 0xFF);
      uint8_t b3 = (uint8_t) ((this->sync_shift_ >> 24) & 0xFF);
      if (b1 == (uint8_t) 'C' && b3 == (uint8_t) 'C' && hamming <= 3)
        t1b = true;
    }

    bool t2 = ((this->sync_shift_ & MASK24) == SYNC_ZC_DASH_24);
    bool t3 = ((this->sync_shift_ & MASK24) == SYNC_CZC_24) ||
              ((this->sync_shift_ & MASK24) == SYNC_ZCZ_24);

    // Tier-4 still gated (prevents false-start flood)
    bool t4 = (this->idle_edge_samples_ > 0) &&
              ((this->sync_shift_ & MASK16) == SYNC_ZC_16);

    if (t1 || t1b || t2 || t3 || t4) {
      int preamble_len;
      bool fb = false;
      const char *tier;
      if (t1 || t1b) { preamble_len = 4; tier = t1 ? "ZCZC" : "ZCZC(misread)"; }
      else if (t2)   { preamble_len = 2; fb = true; tier = "ZC- fallback"; }
      else if (t3)   { preamble_len = 3; tier = "CZC/ZCZ clip"; }
      else           { preamble_len = 2; tier = "ZC idle-edge"; }

      if (this->burst_idx_ == 0)
        this->fallback_sync_used_ = fb || t3 || t4;

      ESP_LOGD(TAG, "Sync via %s (AB=%d, T0=%s). Capture start.",
               tier, this->ab_match_count_, this->preamble_locked_ ? "locked" : "no");

      // --- INSTRUMENTATION: how many bits from T0 lock to this sync? ---
      if (this->t0_lock_pending_) {
        uint32_t elapsed = this->hunt_bit_counter_ - this->t0_lock_bit_marker_;
        ESP_LOGD(TAG, "T0->sync latency = %" PRIu32 " bits (~%.1f preamble-bytes).",
                 elapsed, elapsed / 8.0f);
        this->t0_lock_pending_ = false;
      }

      // If T0 already lit the LED for this acquisition, don't double-fire on_sync.
      if (!this->preamble_locked_)
        this->fire_sync_once_();

      this->begin_new_capture_(preamble_len, fb);
    }
    return;
  }

  // CAPTURE
  this->cur_byte_ >>= 1;
  if (bit) this->cur_byte_ |= 0x80;
  this->cur_nbits_++;

  if (this->cur_nbits_ != 8)
    return;

  char c = (char) this->cur_byte_;
  this->cur_byte_ = 0;
  this->cur_nbits_ = 0;

  if (c == 'N') {
    this->eom_n_count_++;
    if (this->eom_n_count_ >= 4) {
      ESP_LOGD(TAG, "EOM (NNNN) - closing session.");
      if (this->burst_idx_ >= 1)
        this->vote_and_emit_(true, this->fallback_sync_used_);
      this->fire_eom_();          // LED off immediately: message is over
      this->reset_capture_();
      return;
    }
  } else {
    this->eom_n_count_ = 0;
  }

  if (!is_valid_same_char(c)) {
    if (this->cur_burst_.size() >= 4)
      this->finish_burst_();
    else
      this->rearm_sync_();
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
      this->finish_burst_();
      return;
    }
  }

  if (this->cur_burst_.size() >= (size_t) MAX_HEADER_BYTES)
    this->finish_burst_();
}

bool SAMEDecoder::header_is_strictly_valid_(const std::string &header) {
  if (header.rfind("ZCZC", 0) != 0) return false;
  if (header.find('+') == std::string::npos) return false;
  SameAlert probe;
  if (!this->parse_header_(header, probe)) return false;
  if (probe.event_code.size() != 3) return false;
  if (probe.originator.size() != 3) return false;
  return true;
}

bool SAMEDecoder::header_passes_semantic_(const SameAlert &a) const {
  if (a.originator.size() != 3 || a.event_code.size() != 3)
    return false;
  return true;
}

std::string SAMEDecoder::canonicalize_front_(const std::string &voted) {
  if (voted.rfind("ZCZC-", 0) == 0)
    return voted;
  size_t dash = voted.find('-');
  if (dash == std::string::npos) {
    size_t i = 0;
    while (i < voted.size() && (voted[i] == 'Z' || voted[i] == 'C')) i++;
    return std::string("ZCZC-") + voted.substr(i);
  }
  return std::string("ZCZC-") + voted.substr(dash + 1);
}

void SAMEDecoder::note_emit_() {
  this->last_emit_ms_ = millis();

  // Strong post-emit recovery:
  // 1. Clear timing and correlator state
  this->w_off_ = 0.0f;
  this->phase_ = 0.0f;
  this->ab_byte_ = 0;
  this->ab_nbits_ = 0;
  this->ab_match_count_ = 0;
  this->eom_n_count_ = 0;

  // 1b. Clear T0 preamble detector so the next preamble (e.g. EOM) locks clean.
  this->reset_preamble_detector_();
  this->fast_acquire_ = false;
  this->fast_acquire_bits_left_ = 0;

  // 2. Zero the entire sample ring so residual audio from the previous
  //    message cannot contaminate the next preamble.
  for (int i = 0; i < RINGLEN; i++)
    this->ring_[i] = 0;
  this->ring_pos_ = 0;
  this->samples_seen_ = 0;

  // 3. Re-open a short Tier-4 window so a slightly weak follow-up header
  //    still has the extra acquisition path available.
  this->idle_edge_samples_ = 48000 / 2;   // ~0.5 s

  // Strong tiers remain fully armed (always-warm).
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

  std::string header = this->canonicalize_front_(voted);
  ESP_LOGD(TAG, "Voted -> '%s'", sanitize_ascii(header).c_str());

  if (header.rfind("ZCZC", 0) != 0) {
    ESP_LOGW(TAG, "Missing ZCZC; discard.");
    return;
  }

  int collected = 0;
  for (int i = 0; i < 3; i++) if (!this->bursts_[i].empty()) collected++;

  bool weak = (from_timeout && collected < 2) || fallback_synced;
  if (weak) {
    if (!this->header_is_strictly_valid_(header)) {
      ESP_LOGW(TAG, "Weak evidence failed structural check: '%s'", sanitize_ascii(header).c_str());
      return;
    }
  }

  SameAlert alert;
  if (!this->parse_header_(header, alert))
    return;

  if (!this->header_passes_semantic_(alert)) {
    ESP_LOGW(TAG, "Semantic check soft-fail (still emitting): '%s'", sanitize_ascii(header).c_str());
  }

  if (!this->is_known_code_(alert.event_code)) {
    ESP_LOGW(TAG, "Unknown event code '%s'.", alert.event_code.c_str());
  }

  uint32_t now = millis();

  if (!this->session_emitted_header_.empty()) {
    if (this->session_emitted_header_ == header) {
      ESP_LOGD(TAG, "Session duplicate suppressed.");
      return;
    }
    ESP_LOGI(TAG, "Session consensus changed; reissuing.");
  } else {
    bool immediate_same = (!this->last_global_header_.empty()) &&
                          (this->last_global_header_ == header) &&
                          ((uint32_t) (now - this->last_global_ms_) <= IMMEDIATE_DUP_MS);
    if (immediate_same) {
      ESP_LOGD(TAG, "Immediate duplicate suppressed.");
      this->session_emitted_header_ = header;
      this->last_global_header_ = header;
      this->last_global_ms_ = now;
      return;
    }
  }

  this->session_emitted_header_ = header;
  this->last_global_header_ = header;
  this->last_global_ms_ = now;
  this->note_emit_();
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
    ESP_LOGW(TAG, "Alert queue full; drop.");
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
    ESP_LOGW(TAG, "Pending full; drop oldest.");
    this->pending_.erase(this->pending_.begin());
  }
  this->pending_.push_back(a);
  ESP_LOGW(TAG, "API offline; buffered %s (%u).", a.event_code.c_str(), (unsigned) this->pending_.size());
}

void SAMEDecoder::flush_pending_() {
  if (this->pending_.empty())
    return;
  ESP_LOGI(TAG, "API reconnected; flushing %u.", (unsigned) this->pending_.size());
  for (auto &a : this->pending_) {
    this->last_ = a;
    for (auto *t : this->alert_triggers_)
      t->trigger();
  }
  this->pending_.clear();
}

}  // namespace same_decoder
}  // namespace esphome
