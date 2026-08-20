// components/same_decoder/same_decoder.h
// Preamble-locked fixed-phase SAME decoder with always-on tracked timing.
//
// Timing architecture (v5.1):
//  * PRIMARY: lock the bit clock on the 0xAB preamble (47 CFR 11.31). Declaring
//    lock performs a ONE-SHOT phase snap aligning phase_ to bit-center via a
//    sub-bit offset search; it NEVER touches w_off_ (frequency stays bumpless).
//    PREAMBLE QUALIFIER: with a bit-centered one-bit Goertzel, an UNALIGNED
//    window on the ALTERNATING preamble straddles two opposite-tone bits, so
//    per-bit confidence is inherently LOW until the sampler is centered. The
//    qualifier therefore keys on SUSTAINED ALTERNATION + tone presence (which
//    voice cannot mimic for 32 bits), NOT on high per-bit confidence. This
//    breaks the lock<->conf<->phase-snap deadlock.
//  * TRACKING: a signed, decision-directed early-late loop stays alive for the
//    session (soft confidence gate). The INTEGRAL (rate) term runs only while a
//    real burst is in progress (lock effective or CAPTURE); off-signal it drains
//    toward zero with anti-windup (no noise windup). w_off_ is reset only on
//    session boundaries (emit/EOM/watchdog/feed-gap) and is bumpless across the
//    inter-burst gap.
//  * DETECTION: the Goertzel window is one full bit long (GWIN=92), centered on
//    the bit instant via a fixed decision delay (DEC_DELAY).
//
// Lock persistence: carried across the ~1s inter-burst gap (time-aged via
// LOCK_HOLD_MS). An unlocked ZCZC start is allowed but low-trust (strict-valid +
// 2-of-3 corroboration only).

#pragma once

#include "esphome/core/component.h"
#include "esphome/core/automation.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"

#include <string>
#include <vector>
#include <cstdint>
#include <atomic>
#include <cmath>

namespace esphome {
namespace same_decoder {

// ============================================================================
// Soft-decision SAME burst combiner (best-2-of-3).
//
// Given up to 3 bursts, each stored as an array of per-bit pseudo-LLRs
//   l = ln(Em) - ln(Es)
// (additive across repeats, gain-invariant), this module:
//   1. Slices each burst into 8-bit LSB-first ASCII characters.
//   2. Dash-anchors and field-aligns the bursts (bounding slip damage to a field).
//   3. Combines per-bit LLRs across aligned bursts and hard-decides.
//   4. Applies light SAME grammar to score/repair (lenient: never hard-blocks on
//      an unknown event code).
// It NEVER touches the DSP/clock; it only post-processes buffered LLRs.
//
// LSB-first is mandated by the SAME/EAS spec: "The least-significant bit of each
// byte is transmitted first, including the preamble." The 8th bit is the ASCII
// null bit (7-bit ASCII + one null bit = full 8-bit byte, per 47 CFR 11.31).
// ============================================================================

// One captured burst: the raw per-bit LLRs (LSB-first, 8 per character).
struct SoftBurst {
  std::vector<float> llr;   // one entry per received bit
  void clear() { this->llr.clear(); }
  size_t nbits() const { return this->llr.size(); }
  size_t nchars() const { return this->llr.size() / 8; }
};

struct SoftResult {
  std::string header;       // best combined header string
  float mean_margin{0.0f};  // mean per-character decision margin (diagnostic)
  int bursts_used{0};
  bool ok{false};
};

class SoftCombiner {
 public:
  // Combine up to 3 soft bursts into a best-estimate header (best-2-of-3).
  // hard_fallback: if soft combining fails a sanity check, the caller's hard
  // majority result (may be empty) is returned instead so we never regress.
  static SoftResult combine(const SoftBurst *bursts, int nbursts,
                            const std::string &hard_fallback);

 private:
  // Decode one burst's LLRs into (chars, per-char confidence, per-bit LLR ptr).
  struct DecodedBurst {
    std::string chars;                // hard chars from this burst alone
    std::vector<float> char_min_abs;  // min|l| per char (fragility)
    const std::vector<float> *llr{nullptr};
  };

  static char decide_char_(const float *bit_llr, float *out_min_abs);
  static void decode_burst_(const SoftBurst &b, DecodedBurst &out);

  // Split a hard char string into dash-delimited fields (indices into chars).
  static std::vector<std::pair<int, int>> field_spans_(const std::string &s);

  // Combine bit-LLRs across bursts for a given char index (with per-burst
  // char-offset alignment already resolved). Returns the decided char.
  static char combine_char_(const DecodedBurst *db, const int *char_index,
                            int nbursts, float *out_margin);
};

struct SameAlert {
  std::string id;
  std::string event_code;
  std::string event_name;
  std::string severity;
  std::string originator;
  std::string status;
  std::string areas_csv;
  std::string timing;
  std::string onset_iso;
  std::string expires_iso;
  std::string sender;
  std::string raw_header;
};

class AlertTrigger : public Trigger<> {
 public:
  AlertTrigger() = default;
};

class SyncTrigger : public Trigger<> {
 public:
  SyncTrigger() = default;
};

class EomTrigger : public Trigger<> {
 public:
  EomTrigger() = default;
};

class PreambleTrigger : public Trigger<> {
 public:
  PreambleTrigger() = default;
};

class SAMEDecoder : public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::LATE; }

  void set_sample_rate(uint32_t rate);
  void set_gain(float g) { this->gain_ = g; }
  float get_gain() const { return this->gain_; }
  void set_freq_offset_hz(float hz) { this->freq_offset_hz_ = hz; }
  void set_agc_enable(bool e) { this->agc_enable_ = e; }
  void set_agc_target(float t) { this->agc_target_ = t; }
  void set_agc_min_gain(float g) { this->agc_min_gain_ = g; }
  void set_agc_max_gain(float g) { this->agc_max_gain_ = g; }
  void set_timeout_ms(uint32_t v) { this->timeout_ms_.store(v, std::memory_order_relaxed); }
  uint32_t get_timeout_ms() const { return this->timeout_ms_.load(std::memory_order_relaxed); }
  void set_single_burst_min_ms(uint32_t v) { this->single_burst_min_ms_ = v; }
  void set_post_emit_dead_ms(uint32_t v) { this->post_emit_dead_ms_ = v; }
  void set_resend_suppress_ms(uint32_t v) { this->resend_suppress_ms_.store(v, std::memory_order_relaxed); }
  uint32_t get_resend_suppress_ms() const { return this->resend_suppress_ms_.load(std::memory_order_relaxed); }

  // --- Preamble-lock parameter setters ---
  void set_preamble_lock_bits(int v) { this->preamble_lock_bits_ = v; }
  void set_preamble_energy_mult(float v) { this->preamble_energy_mult_.store(v, std::memory_order_relaxed); }
  // Retained for config compatibility; no longer a hard gate.
  void set_preamble_balance_max(float v) { this->preamble_balance_max_ = v; }
  void set_lock_confidence_min(float v) { this->lock_confidence_min_ = v; }
  void set_residual_drift_ppm(float v) { this->residual_drift_ppm_ = v; }
  void set_fallback_conf_thresh(float v) { this->fallback_conf_thresh_ = v; }
  void set_fallback_kp(float v) { this->fallback_kp_ = v; }
  void set_fallback_ki(float v) { this->fallback_ki_ = v; }

  void set_eom_require_context(bool v) { this->eom_require_context_ = v; }
  void set_eom_context_ms(uint32_t v) { this->eom_context_ms_ = v; }

  void set_decode_watchdog_ms(uint32_t v) { this->decode_watchdog_ms_.store(v, std::memory_order_relaxed); }

  void set_decode_count_sensor(sensor::Sensor *s) { this->decode_count_sensor_ = s; }
  void set_last_raw_sensor(text_sensor::TextSensor *s) { this->last_raw_sensor_ = s; }
  void register_alert_trigger(AlertTrigger *t) { this->alert_triggers_.push_back(t); }
  void register_sync_trigger(SyncTrigger *t) { this->sync_triggers_.push_back(t); }
  void register_eom_trigger(EomTrigger *t) { this->eom_triggers_.push_back(t); }
  void register_preamble_trigger(PreambleTrigger *t) { this->preamble_triggers_.push_back(t); }

  void set_api_connected(bool connected);
  void feed_bytes(const std::vector<uint8_t> &data);
  uint32_t ms_since_last_feed() const;

  std::string last_alert_id() const { return this->last_.id; }
  std::string last_event_code() const { return this->last_.event_code; }
  std::string last_event_name() const { return this->last_.event_name; }
  std::string last_severity() const { return this->last_.severity; }
  std::string last_originator() const { return this->last_.originator; }
  std::string last_status() const { return this->last_.status; }
  std::string last_areas_csv() const { return this->last_.areas_csv; }
  std::string last_onset_iso() const { return this->last_.onset_iso; }
  std::string last_expires_iso() const { return this->last_.expires_iso; }
  std::string last_sender() const { return this->last_.sender; }
  std::string last_raw_header() const { return this->last_.raw_header; }

 protected:
  void feed_sample_(int16_t s);
  void emit_bit_(bool bit);
  void rearm_sync_();
  void reset_capture_();
  void begin_new_capture_(int preamble_len, bool fallback);
  void vote_and_emit_(bool from_timeout, bool fallback_synced);
  void finish_burst_();
  void compute_coeffs_();
  void update_agc_(float mag);
  void note_emit_();
  void check_decode_watchdog_();

  // --- Preamble lock helpers ---
  // Qualifier: tone-present + sustained alternation (NOT high per-bit conf).
  void update_preamble_lock_(float mark_e, float space_e, bool bit, float llr);
  void reset_preamble_lock_();
  void clear_preamble_run_();
  bool lock_is_effective_() const;
  void request_phase_snap_();

  void fire_sync_once_();
  void fire_eom_();
  void fire_preamble_once_();

  bool parse_header_(const std::string &header, SameAlert &out);
  bool header_is_strictly_valid_(const std::string &header);
  bool header_is_complete_(const std::string &header);
  bool header_passes_semantic_(const SameAlert &a) const;
  bool same_message_as_current_(const std::string &new_burst);
  bool fuzzy_equal_(const std::string &a, const std::string &b);
  std::string canonicalize_front_(const std::string &voted);
  std::string describe_(const std::string &code);
  std::string severity_for_(const std::string &code);
  bool is_known_code_(const std::string &code);
  std::string make_id_(const SameAlert &a);
  void publish_alert_(const SameAlert &a);
  void dispatch_alert_(const SameAlert &a);
  void deliver_or_buffer_(const SameAlert &a);
  void flush_pending_();

  uint32_t sample_rate_{48000};
  float gain_{1.0f};
  float freq_offset_hz_{0.0f};
  sensor::Sensor *decode_count_sensor_{nullptr};
  text_sensor::TextSensor *last_raw_sensor_{nullptr};
  std::vector<AlertTrigger *> alert_triggers_;
  std::vector<SyncTrigger *> sync_triggers_;
  std::vector<EomTrigger *> eom_triggers_;
  std::vector<PreambleTrigger *> preamble_triggers_;

  SameAlert last_{};
  uint32_t decode_count_{0};

  static constexpr int ALERT_Q_LEN = 4;
  SameAlert alert_queue_[ALERT_Q_LEN];
  std::atomic<uint32_t> q_head_{0};
  std::atomic<uint32_t> q_tail_{0};
  std::atomic<uint32_t> sync_pending_{0};
  std::atomic<uint32_t> eom_pending_{0};
  std::atomic<uint32_t> preamble_pending_{0};

  bool api_connected_{false};
  std::atomic<bool> api_connected_changed_{false};
  static constexpr size_t PENDING_MAX = 16;
  std::vector<SameAlert> pending_;

  std::string session_emitted_header_;
  std::string last_global_header_;
  uint32_t last_global_ms_{0};
  // Cross-session re-emit window. Replaces the fixed IMMEDIATE_DUP_MS. Tunable
  // at runtime from Home Assistant (0 .. 900000 ms). 0 => never suppress
  // cross-session repeats. In-transmission repeats are always collapsed by the
  // (untimed) session_emitted_header_ key, independent of this value.
  std::atomic<uint32_t> resend_suppress_ms_{3000};

  std::atomic<uint32_t> last_feed_ms_{0};
  static constexpr uint32_t FEED_GAP_MS = 250;
  float env_slow_{0.0f};
  static constexpr float ENV_ALPHA = 0.0025f;
  static constexpr float ENV_SILENCE = 200.0f;
  static constexpr float ENV_RISE_MULT = 4.0f;
  bool was_idle_{true};
  uint32_t idle_edge_samples_{0};

  float samples_per_bit_{92.16f};
  float phase_inc_{1.0f / 92.16f};
  float coeff_mark_{1.926090f};
  float coeff_space_{1.958313f};

  static constexpr int GWIN = 92;
  static constexpr int HALF_AFTER = (GWIN - 1) / 2;          // 45
  static constexpr int HALF_BEFORE = GWIN - 1 - HALF_AFTER;  // 46
  static constexpr int TR_DELTA = 16;
  static constexpr int DEC_DELAY = HALF_BEFORE + TR_DELTA;   // 62
  static constexpr int RINGLEN = 256;
  int16_t ring_[RINGLEN];
  int ring_pos_{0};
  float phase_{0.0f};

  static constexpr float SAMPLE_MAX = 32767.0f;
  static constexpr float SOFT_KNEE = 24576.0f;
  static float soft_clip_(float v);

  bool agc_enable_{false};
  float agc_target_{8000.0f};
  float agc_min_gain_{0.25f};
  float agc_max_gain_{32.0f};
  float agc_gain_{1.0f};
  float agc_peak_{0.0f};
  static constexpr float AGC_ATTACK = 0.002f;
  static constexpr float AGC_RELEASE = 0.0002f;

  // Hysteretic + dwell gate for the always-on timing loop.
  static constexpr float TR_CONF_ENGAGE = 0.35f;
  static constexpr float TR_CONF_RELEASE = 0.25f;
  static constexpr int   TR_GATE_DWELL = 8;
  static constexpr float TR_DPHI_CLAMP = 0.125f;
  static constexpr float TR_CONF_FLOOR = 0.04f;
  static constexpr float TR_WOFF_LEAK = 1e-4f;
  bool tr_gate_open_{false};
  int  tr_gate_run_{0};

  // Idle / off-signal hardening.
  static constexpr float TR_WOFF_LEAK_UNLOCKED = 1e-3f;
  static constexpr float TR_WOFF_CLAMP_UNLOCKED_FRAC = 0.25f;
  static constexpr float TR_ANTI_WINDUP_BETA = 0.10f;

  // Two-stage timing-loop gains.
  static constexpr float ACQ_KP = 0.25f;
  static constexpr float ACQ_KI = 0.006f;
  static constexpr int   N_GOOD_TO_TRACK = 48;
  int good_bits_since_lock_{0};

  float w_off_{0.0f};
  uint32_t samples_seen_{0};

  float last_bit_llr_{0.0f};
  SoftBurst soft_bursts_[3];
  SoftBurst soft_cur_;
  static constexpr float LLR_EPS = 1.0f;
  int bad_char_run_{0};

  // Debug telemetry: a rolling summary at DEBUG every DBG_SUMMARY_EVERY bits
  // (file-scope constant in the .cpp). Fixed-size accumulators (no allocation).
  uint32_t dbg_bit_count_{0};
  float    dbg_conf_accum_{0.0f};
  uint16_t dbg_conf_n_{0};
  float    dbg_last_conf_{0.0f};

  // ---------------- Preamble lock state (v5.1 qualifier) ----------------
  // Qualify a bit-period on: (a) tone energy over the adaptive floor, and
  // (b) SUSTAINED ALTERNATION (a running score that rises on toggles, decays on
  // repeats). PRE_CONF_MIN is only a noise floor (an unaligned window on the
  // alternating preamble inherently reads moderate conf; requiring high conf
  // deadlocks lock). A run of >= preamble_lock_bits_ qualifying bits at mean
  // |LLR| >= lock_confidence_min_ declares LOCK and arms the phase snap.
  int preamble_lock_bits_{32};
  std::atomic<float> preamble_energy_mult_{8.0f};
  float preamble_balance_max_{0.40f};   // retained for config compat; not gated
  float lock_confidence_min_{0.5f};
  float residual_drift_ppm_{2000.0f};

  static constexpr float PRE_CONF_MIN = 0.15f;   // noise floor only
  static constexpr int   PRE_ALT_UP = 2;         // score step on a bit toggle
  static constexpr int   PRE_ALT_DOWN = 3;       // score step on a repeat (decay faster)
  static constexpr int   PRE_ALT_MAX = 32;       // score clamp
  static constexpr int   PRE_ALT_OK = 24;        // "strongly alternating" threshold
  int  pre_alt_score_{0};
  int  pre_last_bit_{-1};

  float pre_noise_floor_{1.0f};
  static constexpr float PRE_FLOOR_ALPHA = 0.01f;
  int pre_run_{0};
  double pre_run_conf_sum_{0};
  bool preamble_locked_{false};
  bool was_preamble_locked_{false};
  uint32_t last_lock_ms_{0};
  int byte_phase_{0};

  // Phase-snap state (one-shot bit-center alignment).
  static constexpr int SNAP_CENTERS = 4;
  static constexpr int SNAP_USE = 2;
  static constexpr int SNAP_THETA_MAX = HALF_BEFORE;
  static constexpr int SNAP_THETA_STEP = 3;
  uint32_t center_hist_[SNAP_CENTERS]{0};
  int center_hist_count_{0};
  int center_hist_pos_{0};
  bool snap_pending_{false};
  float snap_delta_phase_{0.0f};

  static constexpr uint32_t LOCK_HOLD_MS = 1200;

  float fallback_conf_thresh_{0.20f};
  float fallback_kp_{0.08f};
  float fallback_ki_{0.0025f};
  bool fallback_active_{false};
  float tr_woff_clamp_{0.01f * (1.0f / 92.16f)};

  uint32_t last_burst_ms_{0};
  std::atomic<uint32_t> timeout_ms_{3000};
  uint32_t single_burst_min_ms_{7000};
  uint32_t post_emit_dead_ms_{800};
  uint32_t last_emit_ms_{0};

  bool eom_require_context_{true};
  uint32_t eom_context_ms_{120000};
  uint32_t last_valid_header_ms_{0};

  std::atomic<uint32_t> decode_watchdog_ms_{10000};
  bool decode_active_{false};
  uint32_t decode_active_since_{0};

  enum Phase { HUNT_SYNC, CAPTURE };
  Phase phase_state_{HUNT_SYNC};
  uint32_t sync_shift_{0};

  static constexpr uint32_t SYNC_ZCZC =
      (static_cast<uint32_t>('C') << 24) |
      (static_cast<uint32_t>('Z') << 16) |
      (static_cast<uint32_t>('C') << 8) |
      (static_cast<uint32_t>('Z'));
  static constexpr int SYNC_MAX_HAMMING = 1;

  static constexpr uint32_t SYNC_ZC_DASH_24 =
      (static_cast<uint32_t>('-') << 16) |
      (static_cast<uint32_t>('C') << 8) |
      (static_cast<uint32_t>('Z'));

  static constexpr uint32_t SYNC_CZC_24 =
      (static_cast<uint32_t>('C') << 16) |
      (static_cast<uint32_t>('Z') << 8) |
      (static_cast<uint32_t>('C'));
  static constexpr uint32_t SYNC_ZCZ_24 =
      (static_cast<uint32_t>('Z') << 16) |
      (static_cast<uint32_t>('C') << 8) |
      (static_cast<uint32_t>('Z'));

  static constexpr uint32_t MASK24 = 0x00FFFFFFu;
  bool fallback_sync_used_{false};

  bool cur_low_trust_{false};
  bool burst_low_trust_[3]{false, false, false};

  uint8_t cur_byte_{0};
  int cur_nbits_{0};
  std::string cur_burst_;
  std::string bursts_[3];
  int burst_idx_{0};
  static constexpr int MAX_HEADER_BYTES = 268;

  static constexpr int MIN_BURSTS_TO_EMIT = 2;
  static constexpr float BURST_MAX_MISMATCH = 0.10f;
  bool bursts_agree_(int count);
  bool early_emitted_{false};
  static constexpr float FIELD_FUZZ = 0.20f;

  bool plus_seen_{false};
  int tail_count_{0};
  static constexpr int TAIL_MIN = 14;
  static constexpr int TAIL_COMPLETE = 21;

  int eom_n_count_{0};
};

}  // namespace same_decoder
}  // namespace esphome
