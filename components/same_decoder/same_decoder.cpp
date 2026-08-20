// components/same_decoder/same_decoder.cpp
// Preamble-locked fixed-phase SAME decoder + automatic closed-loop fallback.
//
// v5 (preamble qualifier fix): the bit-centered one-bit Goertzel makes each
// clean preamble bit a SINGLE tone, so the old per-bit mark/space "balance"
// gate (balanced < preamble_balance_max_) never qualified and preamble lock
// never fired. The qualifier is now tone-present + high per-bit confidence +
// alternation. Everything else is unchanged: decision-directed early-late TED,
// integral gated on lock/CAPTURE with anti-windup + off-signal drain, phase-snap
// at lock, bumpless w_off_ across the inter-burst gap.

#include "same_decoder.h"
#include "same_soft.h"
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

// Debug summary interval (bits). Power of two so we can mask instead of modulo.
// Reduced to 512 (was 64) now that the core path is validated.
static constexpr uint32_t DBG_SUMMARY_EVERY = 512;

static inline int imod(int a, int m) {
  a %= m;
  if (a < 0) a += m;
  return a;
}

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

// SAME chars: digits, UPPER and lower letters, '-', '+', '/'. Lowercase is
// accepted so non-standard sender/callsign fields (and test files) don't shred.
static bool is_valid_same_char(char c) {
  return (c >= '0' && c <= '9') ||
         (c >= 'A' && c <= 'Z') ||
         (c >= 'a' && c <= 'z') ||
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

  const float baud = 520.0f + 5.0f / 6.0f;   // 520 5/6 bps, per 47 CFR 11.31
  this->samples_per_bit_ = sr / baud;
  this->phase_inc_ = 1.0f / this->samples_per_bit_;
  this->tr_woff_clamp_ = (this->residual_drift_ppm_ * 1e-6f) * this->phase_inc_;

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
  ESP_LOGCONFIG(TAG,
      "SAME decoder ready (phase-snap preamble-lock + centered detection + always-on tracked timing, gain=%.1f, agc=%s).",
      this->gain_, this->agc_enable_ ? "on" : "off");
  this->reset_capture_();
  this->last_feed_ms_.store(millis(), std::memory_order_relaxed);
  this->fire_eom_();
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

  uint32_t pre = this->preamble_pending_.exchange(0, std::memory_order_acq_rel);
  while (pre > 0) {
    for (auto *t : this->preamble_triggers_)
      t->trigger();
    pre--;
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
  ESP_LOGCONFIG(TAG, "SAME Decoder (phase-snap preamble-lock + centered detection + always-on tracked timing):");
  ESP_LOGCONFIG(TAG, "  Sample rate: %" PRIu32 " Hz", this->sample_rate_);
  ESP_LOGCONFIG(TAG, "  Base gain: %.1f  AGC: %s", this->gain_, this->agc_enable_ ? "on" : "off");
  ESP_LOGCONFIG(TAG, "  Freq offset: %.1f Hz", this->freq_offset_hz_);
  ESP_LOGCONFIG(TAG, "  Samples/bit: %.2f  Goertzel window: %d (centered, dec-delay %d)",
                this->samples_per_bit_, GWIN, DEC_DELAY);
  ESP_LOGCONFIG(TAG, "  Coeffs mark=%.6f space=%.6f", this->coeff_mark_, this->coeff_space_);
  ESP_LOGCONFIG(TAG, "  Preamble lock (v5 qualifier: tone + conf>=%.2f + alternation): >= %d good bit-periods, mean|LLR|>=%.2f, energy mult %.1f",
                PRE_CONF_MIN, this->preamble_lock_bits_, this->lock_confidence_min_,
                this->preamble_energy_mult_.load(std::memory_order_relaxed));
  ESP_LOGCONFIG(TAG, "    (preamble_balance_max=%.2f retained for config compat; no longer gated)",
                this->preamble_balance_max_);
  ESP_LOGCONFIG(TAG, "    Phase-snap: one-shot sub-bit align at lock (+/-%d samples, step %d, %d centers); w_off_ untouched.",
                SNAP_THETA_MAX, SNAP_THETA_STEP, SNAP_USE);
  ESP_LOGCONFIG(TAG, "    Lock carried across inter-burst gap for up to %" PRIu32 " ms (time-aged).", (uint32_t) LOCK_HOLD_MS);
  ESP_LOGCONFIG(TAG, "    Preamble-lock diagnostic ACTIVE at DEBUG (PRE diag).");
  ESP_LOGCONFIG(TAG, "  Residual-drift trim: %.0f ppm of bit period", this->residual_drift_ppm_);
  ESP_LOGCONFIG(TAG, "  Timing gate: engage conf>=%.2f, release conf<%.2f, dwell %d (confidence-weighted, KI floor %.2f, leak %.0e)",
                TR_CONF_ENGAGE, TR_CONF_RELEASE, TR_GATE_DWELL, TR_CONF_FLOOR, (double) TR_WOFF_LEAK);
  ESP_LOGCONFIG(TAG, "    Integral gated on lock/CAPTURE; off-signal drain leak %.0e, clamp frac %.2f, anti-windup beta %.2f",
                (double) TR_WOFF_LEAK_UNLOCKED, TR_WOFF_CLAMP_UNLOCKED_FRAC, TR_ANTI_WINDUP_BETA);
  ESP_LOGCONFIG(TAG, "  Timing loop: decision-directed early-late; acq Kp=%.4f Ki=%.5f -> track Kp=%.4f Ki=%.5f after %d bits",
                ACQ_KP, ACQ_KI, this->fallback_kp_, this->fallback_ki_, N_GOOD_TO_TRACK);
  ESP_LOGCONFIG(TAG, "    TL summary every %" PRIu32 " bits at DEBUG.", (uint32_t) DBG_SUMMARY_EVERY);
  ESP_LOGCONFIG(TAG, "    ZCZC hamming: strict<=%d (sampled at preamble-locked timing)", SYNC_MAX_HAMMING);
  ESP_LOGCONFIG(TAG, "    Unlocked ZCZC starts allowed but LOW-TRUST (strict-valid + 2-of-3 only).");
  ESP_LOGCONFIG(TAG, "  EOM requires context: %s (%" PRIu32 " ms)",
                this->eom_require_context_ ? "yes" : "no", this->eom_context_ms_);
  ESP_LOGCONFIG(TAG, "  Decode watchdog: %" PRIu32 " ms",
                this->decode_watchdog_ms_.load(std::memory_order_relaxed));
  ESP_LOGCONFIG(TAG, "  Timeout (>=2 bursts): %" PRIu32 " ms", this->timeout_ms_.load(std::memory_order_relaxed));
  ESP_LOGCONFIG(TAG, "  Single-burst min: %" PRIu32 " ms", this->single_burst_min_ms_);
  ESP_LOGCONFIG(TAG, "  Post-emit dead-time: %" PRIu32 " ms", this->post_emit_dead_ms_);
  ESP_LOGCONFIG(TAG, "  Alert triggers: %u  sync: %u  eom: %u  preamble: %u",
                (unsigned) this->alert_triggers_.size(), (unsigned) this->sync_triggers_.size(),
                (unsigned) this->eom_triggers_.size(), (unsigned) this->preamble_triggers_.size());
}

// Full teardown: clears timing state AND last_lock_ms_ (message boundaries only).
// This is the ONLY place (besides feed-gap) that zeros w_off_ -- preserving the
// converged timing correction across the lock event and inter-burst gap.
void SAMEDecoder::reset_preamble_lock_() {
  this->pre_run_ = 0;
  this->pre_run_conf_sum_ = 0.0;
  this->preamble_locked_ = false;
  this->was_preamble_locked_ = false;
  this->byte_phase_ = 0;
  this->fallback_active_ = false;
  this->w_off_ = 0.0f;
  this->last_lock_ms_ = 0;
  this->tr_gate_open_ = false;
  this->tr_gate_run_ = 0;
  // v5 qualifier state.
  this->pre_alt_score_ = 0;
  this->pre_last_bit_ = -1;
  // Phase-snap + acquisition state are session-scoped: tear them down here.
  this->snap_pending_ = false;
  this->snap_delta_phase_ = 0.0f;
  this->center_hist_count_ = 0;
  this->center_hist_pos_ = 0;
  this->good_bits_since_lock_ = 0;
  // Debug accumulators are session-scoped too (keeps summaries per-session).
  this->dbg_conf_accum_ = 0.0f;
  this->dbg_conf_n_ = 0;
  // Preamble-lock diagnostic throttle state.
  this->pre_diag_bits_ = 0;
  this->pre_diag_last_qual_ = false;
  this->pre_diag_last_tone_ = false;
  this->pre_diag_last_alt_ = false;
}

// Per-burst run-accumulator clear ONLY. Preserves preamble_locked_, w_off_,
// last_lock_ms_, and the timing-gate state so bursts 2/3 stay locked AND keep
// their converged timing across the ~1s inter-burst gap.
void SAMEDecoder::clear_preamble_run_() {
  this->pre_run_ = 0;
  this->pre_run_conf_sum_ = 0.0;
  // Keep alternation score/last-bit warm across the gap so the next burst's
  // preamble re-qualifies quickly (they are cheap running estimators).
}

bool SAMEDecoder::lock_is_effective_() const {
  if (this->preamble_locked_)
    return true;
  return (this->last_lock_ms_ != 0) &&
         ((uint32_t) (millis() - this->last_lock_ms_) <= LOCK_HOLD_MS);
}

// Search recent preamble bit-centers over a sub-bit offset grid and arm a
// one-shot phase_ adjustment that snaps the sampler onto bit-center. This reads
// energies already sitting in the ring (the preamble bits just decided) and
// picks the offset that maximizes mean |mark-space|/(mark+space) confidence.
// It adjusts phase_ ONLY (never w_off_): frequency stays bumpless.
void SAMEDecoder::request_phase_snap_() {
  if (this->center_hist_count_ < SNAP_USE)
    return;   // not enough recent centers yet; snap deferred until we have them

  const int s_now = (this->ring_pos_ - 1 + RINGLEN) % RINGLEN;
  const uint32_t s_now_abs = this->samples_seen_ - 1;
  const float eps = 1e-6f;

  // Gather the most recent SNAP_USE center absolute indices from the ring log.
  uint32_t centers[SNAP_USE];
  int gathered = 0;
  for (int k = 1; k <= this->center_hist_count_ && gathered < SNAP_USE; k++) {
    int idx = imod(this->center_hist_pos_ - k, SNAP_CENTERS);
    centers[gathered++] = this->center_hist_[idx];
  }
  if (gathered < SNAP_USE)
    return;

  float best_score = -1.0f;
  int best_theta = 0;
  for (int theta = -SNAP_THETA_MAX; theta <= SNAP_THETA_MAX; theta += SNAP_THETA_STEP) {
    float sum_conf = 0.0f;
    int valid = 0;
    for (int c = 0; c < gathered; c++) {
      long end_abs = (long) centers[c] + theta + HALF_AFTER;
      if (end_abs + TR_DELTA > (long) s_now_abs)
        continue;
      if (end_abs < 0)
        continue;
      int back = (int) ((long) s_now_abs - end_abs);
      if (back >= RINGLEN - GWIN)
        continue;
      int end_ring = imod(s_now - back, RINGLEN);
      float em = goertzel_ring_end(this->ring_, RINGLEN, end_ring, GWIN, this->coeff_mark_);
      float es = goertzel_ring_end(this->ring_, RINGLEN, end_ring, GWIN, this->coeff_space_);
      float d = (em - es) / (em + es + eps);
      sum_conf += std::fabs(d);
      valid++;
    }
    if (valid == 0)
      continue;
    float score = sum_conf / (float) valid;
    if (score > best_score) {
      best_score = score;
      best_theta = theta;
    }
  }

  if (best_score < 0.0f)
    return;

  this->snap_delta_phase_ = -(float) best_theta * this->phase_inc_;
  this->snap_pending_ = true;
  ESP_LOGD(TAG, "Phase-snap armed: theta=%d samples (score=%.2f, dphase=%.4f).",
           best_theta, best_score, this->snap_delta_phase_);
}

// Preamble lock (v5 qualifier). Declares/refreshes lock and fires the trigger.
// Qualifier: tone-present (energy over adaptive floor) AND high per-bit
// confidence (clean single tone) AND alternation (running score). On the rising
// edge into lock it ARMS a one-shot phase snap (phase_ only); it never modifies
// w_off_. A throttled PRE diag line (read-only) is emitted while HUNTING, BEFORE
// the CAPTURE early-return so it is reliably reached.
void SAMEDecoder::update_preamble_lock_(float mark_e, float space_e, bool bit, float llr) {
  float tone_energy = mark_e + space_e;

  float f = this->pre_noise_floor_;
  float target = std::min(tone_energy, f);
  f += PRE_FLOOR_ALPHA * (target - f);
  if (f < 1.0f) f = 1.0f;
  this->pre_noise_floor_ = f;

  float mult = this->preamble_energy_mult_.load(std::memory_order_relaxed);

  // Per-bit confidence: for a clean single-tone preamble bit this is ~0.9-1.0.
  float conf = std::fabs(mark_e - space_e) / (mark_e + space_e + 1e-9f);

  bool tone_present = (tone_energy > mult * f);
  bool clean_tone = (conf >= PRE_CONF_MIN);

  // Alternation score: rises on a bit toggle, decays on a repeat. Tolerant of
  // 0xAB's occasional double-bit because a single repeat only nudges it down.
  int this_bit = bit ? 1 : 0;
  if (this->pre_last_bit_ >= 0) {
    if (this_bit != this->pre_last_bit_)
      this->pre_alt_score_ += PRE_ALT_UP;
    else
      this->pre_alt_score_ -= PRE_ALT_DOWN;
    if (this->pre_alt_score_ < 0) this->pre_alt_score_ = 0;
    if (this->pre_alt_score_ > PRE_ALT_MAX) this->pre_alt_score_ = PRE_ALT_MAX;
  }
  this->pre_last_bit_ = this_bit;
  bool alternating = (this->pre_alt_score_ >= PRE_ALT_OK);

  bool qualifies = tone_present && clean_tone && alternating;

  // Freeze preamble lock accounting during CAPTURE so nothing about the
  // preamble path can perturb the payload. Metrics remain read-only. The PRE
  // diag is emitted only while HUNTING (below), so it is skipped here.
  if (this->phase_state_ == CAPTURE) {
    this->was_preamble_locked_ = this->preamble_locked_;
    if (this->preamble_locked_)
      this->last_lock_ms_ = millis();   // keep recency warm; no timing change
    return;
  }

  // ---- Preamble-lock diagnostic (read-only, throttled). Emitted while HUNTING,
  // BEFORE the run/lock update so its throttle counter reliably advances. Shows
  // the three qualifier conditions and the current run. ----
  this->pre_diag_bits_++;
  {
    bool reason_change = (qualifies != this->pre_diag_last_qual_) ||
                         (tone_present != this->pre_diag_last_tone_) ||
                         (alternating != this->pre_diag_last_alt_);
    if (reason_change || this->pre_diag_bits_ >= PRE_DIAG_EVERY) {
      ESP_LOGD(TAG,
               "PRE diag: tone=%c conf=%.2f clean=%c alt=%d/%d(%c) qual=%c run=%d/%d nf=%.1f",
               tone_present ? 'Y' : 'N', conf, clean_tone ? 'Y' : 'N',
               this->pre_alt_score_, PRE_ALT_OK, alternating ? 'Y' : 'N',
               qualifies ? 'Y' : 'N', this->pre_run_, this->preamble_lock_bits_, f);
      this->pre_diag_bits_ = 0;
      this->pre_diag_last_qual_ = qualifies;
      this->pre_diag_last_tone_ = tone_present;
      this->pre_diag_last_alt_ = alternating;
    }
  }

  if (qualifies) {
    this->pre_run_++;
    this->pre_run_conf_sum_ += std::fabs(llr);

    if (!this->preamble_locked_ && this->pre_run_ >= this->preamble_lock_bits_) {
      float mean_conf = (float) (this->pre_run_conf_sum_ / (double) this->pre_run_);
      if (mean_conf >= this->lock_confidence_min_) {
        // PHASE-SNAP LOCK: mark the state + recency, then arm a ONE-SHOT sub-bit
        // phase align onto bit-center. w_off_ and byte_phase_ are NOT touched;
        // frequency stays bumpless and the tracked sampler keeps its converged
        // rate correction.
        this->preamble_locked_ = true;
        this->last_lock_ms_ = millis();
        this->good_bits_since_lock_ = 0;
        this->request_phase_snap_();
      }
    }
  } else {
    if (this->pre_run_ > 0) this->pre_run_--;
    if (this->pre_run_ == 0) {
      this->pre_run_conf_sum_ = 0.0;
    }
  }

  if (this->preamble_locked_)
    this->last_lock_ms_ = millis();

  if (this->preamble_locked_ && !this->was_preamble_locked_) {
    ESP_LOGI(TAG, "Preamble LOCK (run=%d, mean_conf=%.2f, alt=%d) [phase-snap].", this->pre_run_,
             this->pre_run_ > 0 ? (float) (this->pre_run_conf_sum_ / (double) this->pre_run_) : 0.0f,
             this->pre_alt_score_);
    this->fire_preamble_once_();
  }
  this->was_preamble_locked_ = this->preamble_locked_;
}

void SAMEDecoder::fire_sync_once_() {
  uint32_t p = this->sync_pending_.load(std::memory_order_relaxed);
  if (p < 8) this->sync_pending_.store(p + 1, std::memory_order_release);
}

void SAMEDecoder::fire_eom_() {
  uint32_t p = this->eom_pending_.load(std::memory_order_relaxed);
  if (p < 8) this->eom_pending_.store(p + 1, std::memory_order_release);
}

void SAMEDecoder::fire_preamble_once_() {
  uint32_t p = this->preamble_pending_.load(std::memory_order_relaxed);
  if (p < 8) this->preamble_pending_.store(p + 1, std::memory_order_release);
}

void SAMEDecoder::check_decode_watchdog_() {
  if (!this->decode_active_)
    return;
  uint32_t wd = this->decode_watchdog_ms_.load(std::memory_order_relaxed);
  if ((uint32_t) (millis() - this->decode_active_since_) < wd)
    return;

  ESP_LOGW(TAG, "Decode watchdog fired (%" PRIu32 " ms): abandoning stuck session.", wd);
  if (this->burst_idx_ >= 1)
    this->vote_and_emit_(true, this->fallback_sync_used_);
  this->reset_capture_();
  this->reset_preamble_lock_();   // true session boundary: full teardown
  this->decode_active_ = false;
  this->fire_eom_();
}

void SAMEDecoder::feed_bytes(const std::vector<uint8_t> &data) {
  uint32_t now = millis();
  uint32_t prev = this->last_feed_ms_.exchange(now, std::memory_order_relaxed);
  if (prev != 0 && (uint32_t) (now - prev) >= FEED_GAP_MS) {
    this->phase_ = 0.0f;
    this->reset_preamble_lock_();   // genuine audio discontinuity: full teardown
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

  this->check_decode_watchdog_();
}

static inline uint32_t effective_timeout(int burst_idx, uint32_t slider_ms, uint32_t single_min_ms) {
  if (burst_idx <= 1)
    return slider_ms > single_min_ms ? slider_ms : single_min_ms;
  return slider_ms;
}

void SAMEDecoder::feed_sample_(int16_t s) {
  float mag = std::fabs((float) s);
  this->update_agc_(mag);

  // Time-age the lock so a strong lock can never bridge a full message boundary.
  if (this->preamble_locked_ && this->last_lock_ms_ != 0 &&
      (uint32_t) (millis() - this->last_lock_ms_) > LOCK_HOLD_MS) {
    this->preamble_locked_ = false;
    this->was_preamble_locked_ = false;
  }

  this->env_slow_ += ENV_ALPHA * (mag - this->env_slow_);
  if (this->was_idle_) {
    if (mag > ENV_SILENCE * ENV_RISE_MULT) {
      this->was_idle_ = false;
      this->idle_edge_samples_ = 48000 / 2;
    }
  } else {
    if (this->env_slow_ < ENV_SILENCE)
      this->was_idle_ = true;
  }
  if (this->idle_edge_samples_ > 0)
    this->idle_edge_samples_--;

  if (this->last_emit_ms_ != 0 &&
      (uint32_t) (millis() - this->last_emit_ms_) < this->post_emit_dead_ms_) {
    return;
  }

  this->ring_[this->ring_pos_] = s;
  this->ring_pos_ = (this->ring_pos_ + 1) % RINGLEN;
  if (this->samples_seen_ < 0x7fffffff) this->samples_seen_++;

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

  // Apply a one-shot phase snap (armed at preamble lock) right after the wrap,
  // while phase_ is small, so it cannot cause an accidental immediate re-fire.
  // Adjusts phase_ ONLY; w_off_ is left untouched (bumpless frequency).
  if (this->snap_pending_) {
    float delta = this->snap_delta_phase_;
    if (delta >  0.45f) delta =  0.45f;
    if (delta < -0.45f) delta = -0.45f;
    float p = this->phase_ + delta;
    if (p < 0.0f) p = 0.0f;
    if (p > 0.99f) p = 0.99f;
    this->phase_ = p;
    this->snap_pending_ = false;
    this->snap_delta_phase_ = 0.0f;
  }

  int s_now = (this->ring_pos_ - 1 + RINGLEN) % RINGLEN;

  // Record this bit-center (absolute sample index) for the phase-snap search.
  this->center_hist_[this->center_hist_pos_] = this->samples_seen_ - 1;
  this->center_hist_pos_ = (this->center_hist_pos_ + 1) % SNAP_CENTERS;
  if (this->center_hist_count_ < SNAP_CENTERS) this->center_hist_count_++;

  // Centered detection: the window of length GWIN is centered on the bit instant
  // that occurred DEC_DELAY samples ago, so its END sits at s_now-DEC_DELAY+
  // HALF_AFTER. Early/late are the same window shifted by +/-TR_DELTA; DEC_DELAY
  // guarantees the LATE window end is still in the past (<= s_now).
  int t_center = imod(s_now - DEC_DELAY + HALF_AFTER, RINGLEN);
  int t_early  = imod(t_center - TR_DELTA, RINGLEN);
  int t_late   = imod(t_center + TR_DELTA, RINGLEN);

  float em_c = goertzel_ring_end(this->ring_, RINGLEN, t_center, GWIN, this->coeff_mark_);
  float es_c = goertzel_ring_end(this->ring_, RINGLEN, t_center, GWIN, this->coeff_space_);
  bool bit = em_c > es_c;

  float lem = std::log(em_c > LLR_EPS ? em_c : LLR_EPS);
  float les = std::log(es_c > LLR_EPS ? es_c : LLR_EPS);
  this->last_bit_llr_ = lem - les;

  this->update_preamble_lock_(em_c, es_c, bit, this->last_bit_llr_);

  bool primed = this->samples_seen_ >= (uint32_t) (DEC_DELAY + TR_DELTA + HALF_BEFORE + 1);
  float conf = 0.0f;
  {
    float d_center = std::fabs(em_c - es_c);
    float e_center = em_c + es_c;
    conf = d_center / (e_center + 1e-9f);
  }

  bool low_conf = (conf < this->fallback_conf_thresh_);
  if (low_conf && !this->fallback_active_) {
    this->fallback_active_ = true;
    ESP_LOGVV(TAG, "Fallback tracker engaged (conf=%.2f).", conf);
  } else if (!low_conf && this->fallback_active_) {
    this->fallback_active_ = false;
  }

  // Hysteretic + dwell confidence gate for the ALWAYS-ON timing loop.
  if (!this->tr_gate_open_) {
    if (conf >= TR_CONF_ENGAGE) {
      if (++this->tr_gate_run_ >= TR_GATE_DWELL)
        this->tr_gate_open_ = true;
    } else {
      this->tr_gate_run_ = 0;
    }
  } else {
    if (conf < TR_CONF_RELEASE) {
      this->tr_gate_open_ = false;
      this->tr_gate_run_ = 0;
    }
  }

  // A real SAME burst is in progress when we hold an effective preamble lock OR
  // are already capturing the payload. The INTEGRAL (rate) term only runs in
  // that window; off-signal it drains toward 0. lock precedes ZCZC and CAPTURE
  // covers the payload, so a genuine burst is still fully tracked.
  const bool locked_or_capture =
      this->lock_is_effective_() || (this->phase_state_ == CAPTURE);
  const float eff_clamp = locked_or_capture
                              ? this->tr_woff_clamp_
                              : this->tr_woff_clamp_ * TR_WOFF_CLAMP_UNLOCKED_FRAC;

  if (primed && this->tr_gate_open_) {
    float em_e = goertzel_ring_end(this->ring_, RINGLEN, t_early, GWIN, this->coeff_mark_);
    float es_e = goertzel_ring_end(this->ring_, RINGLEN, t_early, GWIN, this->coeff_space_);
    float em_l = goertzel_ring_end(this->ring_, RINGLEN, t_late,  GWIN, this->coeff_mark_);
    float es_l = goertzel_ring_end(this->ring_, RINGLEN, t_late,  GWIN, this->coeff_space_);

    // DECISION-DIRECTED (symbol-invariant) early-late timing error.
    const float eps = 1e-9f;
    float d_early = (em_e - es_e) / (em_e + es_e + eps);
    float d_late  = (em_l - es_l) / (em_l + es_l + eps);
    float sym_sign = (em_c >= es_c) ? 1.0f : -1.0f;
    float e = sym_sign * (d_early - d_late);

    bool acquiring = (this->good_bits_since_lock_ < N_GOOD_TO_TRACK);
    float kp = acquiring ? ACQ_KP : this->fallback_kp_;
    float ki = acquiring ? ACQ_KI : this->fallback_ki_;

    // PROPORTIONAL (phase) update -- ungated w.r.t. lock/capture (bounded,
    // non-accumulating), keeps the loop responsive the instant signal appears.
    float dphi = kp * conf * e;
    if (dphi >  TR_DPHI_CLAMP) dphi =  TR_DPHI_CLAMP;
    if (dphi < -TR_DPHI_CLAMP) dphi = -TR_DPHI_CLAMP;
    this->phase_ += dphi;

    if (locked_or_capture) {
      // INTEGRAL (rate) path -- ON-SIGNAL ONLY, floored confidence + slow leak +
      // back-calculation anti-windup.
      float conf_ki = (conf > TR_CONF_FLOOR) ? conf : TR_CONF_FLOOR;
      float w_unclamped = this->w_off_ + ki * conf_ki * e;
      w_unclamped -= TR_WOFF_LEAK * w_unclamped;
      bool saturated = (w_unclamped > eff_clamp) || (w_unclamped < -eff_clamp);
      if (saturated) {
        float w_sat = (w_unclamped > 0.0f) ? eff_clamp : -eff_clamp;
        this->w_off_ = w_sat - TR_ANTI_WINDUP_BETA * (w_unclamped - w_sat);
        if (this->w_off_ >  eff_clamp) this->w_off_ =  eff_clamp;
        if (this->w_off_ < -eff_clamp) this->w_off_ = -eff_clamp;
      } else {
        this->w_off_ = w_unclamped;
      }
    } else {
      // OFF-SIGNAL: never integrate; drain toward 0 inside the tighter clamp.
      this->w_off_ -= TR_WOFF_LEAK_UNLOCKED * this->w_off_;
      if (this->w_off_ >  eff_clamp) this->w_off_ =  eff_clamp;
      if (this->w_off_ < -eff_clamp) this->w_off_ = -eff_clamp;
    }

    if (conf >= TR_CONF_ENGAGE && this->good_bits_since_lock_ < 0x7fffffff)
      this->good_bits_since_lock_++;
  } else {
    // Gate closed (or not primed). On-signal HOLD w_off_ (bumpless); off-signal
    // keep DRAINING toward 0 so noise offset cannot linger into the next burst.
    if (!locked_or_capture) {
      this->w_off_ -= TR_WOFF_LEAK_UNLOCKED * this->w_off_;
      if (this->w_off_ >  eff_clamp) this->w_off_ =  eff_clamp;
      if (this->w_off_ < -eff_clamp) this->w_off_ = -eff_clamp;
    }
  }

  // ---- Periodic DEBUG summary every DBG_SUMMARY_EVERY bits: rolling mean conf,
  // last conf, current w_off_ in ppm, gate state, and lock state. ----
  this->dbg_bit_count_++;
  this->dbg_conf_accum_ += conf;
  if (this->dbg_conf_n_ < 0xFFFF) this->dbg_conf_n_++;
  this->dbg_last_conf_ = conf;
  if ((this->dbg_bit_count_ & (DBG_SUMMARY_EVERY - 1u)) == 0u) {
    float mean_conf = (this->dbg_conf_n_ != 0)
                          ? (this->dbg_conf_accum_ / (float) this->dbg_conf_n_)
                          : 0.0f;
    float wppm = (this->phase_inc_ != 0.0f) ? (this->w_off_ / this->phase_inc_) * 1e6f : 0.0f;
    ESP_LOGD(TAG, "TL summary bits=%" PRIu32 " mean_conf=%.2f last_conf=%.2f wppm=%+.1f gate=%c lock=%c",
             this->dbg_bit_count_, mean_conf, this->dbg_last_conf_, wppm,
             this->tr_gate_open_ ? 'O' : 'C',
             this->lock_is_effective_() ? 'Y' : 'N');
    this->dbg_conf_accum_ = 0.0f;
    this->dbg_conf_n_ = 0;
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
  this->eom_n_count_ = 0;
  this->bad_char_run_ = 0;
  this->soft_cur_.clear();
  this->cur_low_trust_ = false;
  // Do NOT tear down lock/timing here. Only clear the run accumulators so the
  // next burst's preamble is counted fresh; preamble_locked_/w_off_/last_lock_ms_
  // and the timing-gate state persist across the ~1s inter-burst gap.
  this->clear_preamble_run_();
}

void SAMEDecoder::reset_capture_() {
  this->rearm_sync_();
  this->burst_idx_ = 0;
  this->last_burst_ms_ = 0;
  this->early_emitted_ = false;
  this->fallback_sync_used_ = false;
  this->session_emitted_header_.clear();
  for (int i = 0; i < 3; i++) this->bursts_[i].clear();
  for (int i = 0; i < 3; i++) this->soft_bursts_[i].clear();
  for (int i = 0; i < 3; i++) this->burst_low_trust_[i] = false;
  this->soft_cur_.clear();
  this->bad_char_run_ = 0;
  this->eom_n_count_ = 0;
  this->cur_low_trust_ = false;
  // decode_active_ is cleared explicitly by emit/EOM/watchdog, not here.
}

void SAMEDecoder::begin_new_capture_(int preamble_len, bool fallback) {
  this->phase_state_ = CAPTURE;
  this->soft_cur_.clear();
  this->bad_char_run_ = 0;
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
  const std::string &ref = this->bursts_;
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
  ESP_LOGD(TAG, "Burst %d ascii: '%s'%s", this->burst_idx_,
           sanitize_ascii(this->cur_burst_).c_str(),
           this->cur_low_trust_ ? " (low-trust)" : "");

  std::string this_burst = this->cur_burst_;
  SoftBurst this_soft = this->soft_cur_;
  bool this_low_trust = this->cur_low_trust_;
  this->soft_cur_.clear();

  if (this->header_is_strictly_valid_(this_burst))
    this->last_valid_header_ms_ = millis();

  if (this->burst_idx_ >= 1 && !this->same_message_as_current_(this_burst)) {
    ESP_LOGD(TAG, "Differing burst mid-collection: finalising old, starting new.");
    this->reset_capture_();
    this->bursts_ = this_burst;
    this->soft_bursts_ = this_soft;
    this->burst_low_trust_ = this_low_trust;
    this->burst_idx_ = 1;
    this->last_burst_ms_ = millis();

    if (!this->early_emitted_ && !this_low_trust &&
        this->header_is_complete_(this->bursts_)) {
      ESP_LOGD(TAG, "Immediate emit on promoted COMPLETE differing burst.");
      this->vote_and_emit_(false, this->fallback_sync_used_);
      this->early_emitted_ = true;
    }

    this->rearm_sync_();
    return;
  }

  if (this->burst_idx_ < 3) {
    this->bursts_[this->burst_idx_] = this_burst;
    this->soft_bursts_[this->burst_idx_] = this_soft;
    this->burst_low_trust_[this->burst_idx_] = this_low_trust;
  }
  this->burst_idx_++;
  this->last_burst_ms_ = millis();

  if (this->burst_idx_ >= 3) {
    this->vote_and_emit_(false, this->fallback_sync_used_);
    this->reset_capture_();
    return;
  }

  if (this->burst_idx_ == 1 && !this->early_emitted_ && !this_low_trust &&
      this->header_is_complete_(this_burst)) {
    ESP_LOGD(TAG, "Immediate emit on single COMPLETE burst.");
    this->vote_and_emit_(false, this->fallback_sync_used_);
    this->early_emitted_ = true;
    this->rearm_sync_();
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
    this->sync_shift_ = (this->sync_shift_ >> 1) | ((uint32_t) (bit ? 1u : 0u) << 31);

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

    if (t1 || t1b || t2 || t3) {
      int preamble_len;
      bool fb = false;
      const char *tier;
      if (t1 || t1b) { preamble_len = 4; tier = t1 ? "ZCZC" : "ZCZC(misread)"; }
      else if (t2)   { preamble_len = 2; fb = true; tier = "ZC- fallback"; }
      else           { preamble_len = 3; tier = "CZC/ZCZ clip"; }  // t3

      if (this->burst_idx_ == 0)
        this->fallback_sync_used_ = fb || t3;

      bool locked_now = this->lock_is_effective_();
      this->cur_low_trust_ = !locked_now;

      ESP_LOGD(TAG, "Sync via %s (locked=%s)%s. Capture start.",
               tier, locked_now ? "yes" : "no",
               this->cur_low_trust_ ? " [LOW-TRUST]" : "");

      if (!this->decode_active_) {
        this->decode_active_ = true;
        this->decode_active_since_ = millis();
      }
      this->fire_sync_once_();

      this->begin_new_capture_(preamble_len, fb);
    }
    return;
  }

  // CAPTURE
  this->soft_cur_.llr.push_back(this->last_bit_llr_);

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
      bool has_context = (!this->eom_require_context_) ||
                         (this->last_valid_header_ms_ != 0 &&
                          (uint32_t) (millis() - this->last_valid_header_ms_) <= this->eom_context_ms_);
      if (has_context) {
        ESP_LOGD(TAG, "EOM (NNNN) - closing session.");
        if (this->burst_idx_ >= 1)
          this->vote_and_emit_(true, this->fallback_sync_used_);
        this->reset_capture_();
        this->reset_preamble_lock_();   // message boundary: full lock teardown
        this->decode_active_ = false;
        this->fire_eom_();
      } else {
        ESP_LOGD(TAG, "NNNN seen but no recent header context; ignoring.");
        this->eom_n_count_ = 0;
        this->rearm_sync_();
      }
      return;
    }
  } else {
    this->eom_n_count_ = 0;
  }

  if (!is_valid_same_char(c)) {
    this->cur_burst_ += '?';
    this->bad_char_run_++;
    if (this->bad_char_run_ >= 4 || this->cur_burst_.size() >= (size_t) MAX_HEADER_BYTES) {
      this->finish_burst_();
    }
    return;
  }
  this->bad_char_run_ = 0;

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

bool SAMEDecoder::header_is_complete_(const std::string &header) {
  if (!this->header_is_strictly_valid_(header))
    return false;
  size_t plus = header.find('+');
  if (plus == std::string::npos)
    return false;
  size_t dash_after = header.find('-', plus + 1);
  if (dash_after == std::string::npos)
    return false;
  if (dash_after <= plus + 1)
    return false;
  if (dash_after + 1 >= header.size())
    return false;
  size_t issue_end = header.find('-', dash_after + 1);
  size_t issue_len = (issue_end == std::string::npos)
                         ? (header.size() - (dash_after + 1))
                         : (issue_end - (dash_after + 1));
  return issue_len >= 4;
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
  this->eom_n_count_ = 0;
  this->idle_edge_samples_ = 48000 / 2;
  this->reset_preamble_lock_();   // message boundary: full lock teardown
  this->decode_active_ = false;
}

void SAMEDecoder::vote_and_emit_(bool from_timeout, bool fallback_synced) {
  size_t maxlen = 0;
  for (int i = 0; i < 3; i++) maxlen = std::max(maxlen, this->bursts_[i].size());
  std::string hard;
  hard.reserve(maxlen);
  for (size_t i = 0; i < maxlen; i++) {
    char best = 0; int bestcount = 0;
    for (int a = 0; a < 3; a++) {
      if (i >= this->bursts_[a].size()) continue;
      char ca = this->bursts_[a][i]; int cnt = 0;
      for (int c = 0; c < 3; c++)
        if (i < this->bursts_[c].size() && this->bursts_[c][i] == ca) cnt++;
      if (cnt > bestcount) { bestcount = cnt; best = ca; }
    }
    if (best) hard += best;
  }
  std::string hard_header = this->canonicalize_front_(hard);

  SoftResult sr = SoftCombiner::combine(this->soft_bursts_, 3, hard_header);
  std::string header = this->canonicalize_front_(sr.header);

  ESP_LOGD(TAG, "Voted (soft=%s, margin=%.2f) -> '%s'",
           sr.ok ? "ok" : "fallback", sr.mean_margin, sanitize_ascii(header).c_str());

  if (header.rfind("ZCZC", 0) != 0) {
    ESP_LOGW(TAG, "Missing ZCZC; discard.");
    return;
  }

  int collected = 0;
  for (int i = 0; i < 3; i++) if (!this->bursts_[i].empty()) collected++;

  int low_trust_collected = 0;
  int trusted_collected = 0;
  for (int i = 0; i < 3; i++) {
    if (this->bursts_[i].empty()) continue;
    if (this->burst_low_trust_[i]) low_trust_collected++;
    else trusted_collected++;
  }
  (void) low_trust_collected;

  bool low_trust_uncorroborated = (trusted_collected == 0) && (collected < 2);
  bool weak = (from_timeout && collected < 2) || fallback_synced || low_trust_uncorroborated;
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
      this->decode_active_ = false;
      this->fire_eom_();
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
      this->decode_active_ = false;
      this->fire_eom_();
      return;
    }
  }

  this->session_emitted_header_ = header;
  this->last_global_header_ = header;
  this->last_global_ms_ = now;
  this->last_valid_header_ms_ = now;
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
