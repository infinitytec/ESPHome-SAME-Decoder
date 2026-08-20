// components/same_decoder/same_decoder.h
// Preamble-locked fixed-phase SAME decoder with always-on tracked timing.
//
// Timing architecture (redesign, v4 phase-snap):
//  * PRIMARY: lock the bit clock on the 0xAB preamble (the preamble exists to
//    "set asynchronous decoder clocking cycles", 47 CFR 11.31). Declaring lock
//    performs a ONE-SHOT phase snap: it aligns phase_ to bit-center using a
//    sub-bit offset search over recent preamble bit-periods. It still NEVER
//    touches w_off_ (frequency remains bumpless); only phase_ is adjusted, once.
//  * TRACKING: a signed early-late timing loop stays alive for the entire
//    session (gated by a soft, low-threshold confidence gate, with the update
//    weighted by confidence) so the sampler is never open-loop between lock and
//    ZCZC. The converged w_off_ correction is preserved across the lock event
//    and the ~1s inter-burst gap, keeping the payload on bit-center. w_off_ is
//    reset ONLY on true session boundaries (emit/EOM/watchdog/feed-gap).
//    The early-late discriminator is DECISION-DIRECTED (symbol-invariant): it
//    is multiplied by the decided-bit sign so the timing gradient points the
//    same way on mark and space bits and does not cancel on mixed payload. The
//    INTEGRAL (rate) term runs only while a real burst is in progress (lock
//    effective or CAPTURE); off-signal it drains toward zero (no noise windup).
//  * DETECTION: the Goertzel analysis window is one full bit long (GWIN=92) and
//    is CENTERED on the bit instant via a fixed decision delay (DEC_DELAY), so
//    the decision variable is symmetric about the bit and mark/space
//    discrimination is maximized.
//
// Lock persistence: the preamble lock is carried across the ~1s inter-burst gap
// (time-aged via LOCK_HOLD_MS so it cannot bridge a full message boundary). An
// unlocked ZCZC start is allowed but flagged low-trust: it can never emit on its
// own and must pass strict structural validation + 2-of-3 corroboration.

#pragma once

#include "esphome/core/component.h"
#include "esphome/core/automation.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "same_soft.h"

#include <string>
#include <vector>
#include <cstdint>
#include <atomic>
#include <cmath>

namespace esphome {
namespace same_decoder {

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

  // --- Preamble-lock parameter setters ---
  void set_preamble_lock_bits(int v) { this->preamble_lock_bits_ = v; }
  void set_preamble_energy_mult(float v) { this->preamble_energy_mult_.store(v, std::memory_order_relaxed); }
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
  // Update the preamble correlator with the current bit-period tone energies and
  // the just-decided bit; declares/refreshes lock. On the transition into lock
  // it arms a ONE-SHOT sub-bit phase snap (see request_phase_snap_); it never
  // touches w_off_ (frequency remains bumpless).
  void update_preamble_lock_(float mark_e, float space_e, bool bit, float llr);
  // Full teardown: clears timing state AND last_lock_ms_ (message boundaries).
  void reset_preamble_lock_();
  // Per-burst run-accumulator clear ONLY; preserves lock/w_off_/recency/gate so
  // bursts 2/3 stay locked with converged timing across the ~1s inter-burst gap.
  void clear_preamble_run_();
  // Strong lock now, or held one within LOCK_HOLD_MS (bridges inter-burst gap).
  bool lock_is_effective_() const;

  // --- Phase snap (one-shot bit-center alignment at lock) ---
  // Search recent preamble bit-centers over a sub-bit offset grid and arm a
  // one-shot phase_ adjustment that snaps the sampler onto bit-center. Adjusts
  // phase_ ONLY (never w_off_), preserving the bumpless-frequency philosophy.
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
  static constexpr uint32_t IMMEDIATE_DUP_MS = 3000;

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

  // Detection window: one full bit long and CENTERED on the bit instant.
  //   GWIN         : Goertzel window length (~= samples_per_bit for best SNR).
  //   HALF_AFTER   : samples of the window that lie at/after the bit center.
  //   HALF_BEFORE  : samples of the window that lie before the bit center.
  //   TR_DELTA     : early/late window offset (~0.17 bit).
  //   DEC_DELAY    : fixed decision latency so a window centered on the bit
  //                  instant (and its LATE early-late window) is fully in the
  //                  past. DEC_DELAY = HALF_BEFORE + TR_DELTA guarantees the
  //                  late window end is <= s_now.
  static constexpr int GWIN = 92;
  static constexpr int HALF_AFTER = (GWIN - 1) / 2;      // 45
  static constexpr int HALF_BEFORE = GWIN - 1 - HALF_AFTER;  // 46
  static constexpr int TR_DELTA = 16;
  static constexpr int DEC_DELAY = HALF_BEFORE + TR_DELTA;   // 62
  static constexpr int RINGLEN = 256;                    // > DEC_DELAY + TR_DELTA + HALF_BEFORE
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

  // Hysteretic + dwell gate for the always-on timing loop. Raised thresholds
  // (item 5) so intermittent voice formants cannot chatter the gate open on
  // non-signal audio: a clean SAME preamble/header sits at conf ~0.8-0.99 and
  // clears these easily, while continuous NWR voice rarely sustains conf above
  // TR_CONF_ENGAGE for TR_GATE_DWELL consecutive bits. The per-update magnitude
  // is also weighted by confidence in feed_sample_(); the integral path uses a
  // confidence FLOOR (on-signal) so it never freezes during real payload runs.
  static constexpr float TR_CONF_ENGAGE = 0.35f;   // open gate above this
  static constexpr float TR_CONF_RELEASE = 0.25f;  // close gate below this
  static constexpr int   TR_GATE_DWELL = 8;        // consecutive good bits to open
  static constexpr float TR_DPHI_CLAMP = 0.125f;
  // Confidence floor for the ON-SIGNAL integral (rate) update so w_off_ keeps
  // tracking slow drift through low-confidence payload runs instead of freezing.
  static constexpr float TR_CONF_FLOOR = 0.04f;
  // Very slow leak on w_off_ toward zero WHILE ON-SIGNAL (locked/CAPTURE),
  // preventing unbounded wander if the loop is briefly starved of good timing.
  static constexpr float TR_WOFF_LEAK = 1e-4f;
  // Runtime state for the hysteretic timing gate (driven in feed_sample_).
  bool tr_gate_open_{false};   // gate currently open (timing loop active)
  int  tr_gate_run_{0};        // consecutive high-conf bits toward opening

  // --- Idle / off-signal hardening (items 1-5) ---
  // The INTEGRAL (w_off_) update runs ONLY when a real burst is in progress
  // (lock effective OR CAPTURE). Off-signal it never integrates and instead
  // DRAINS toward 0 with the stronger unlocked leak, held inside a tighter
  // unlocked clamp. Back-calculation anti-windup keeps w_off_ from welding to
  // the rail even under on-signal transients.
  static constexpr float TR_WOFF_LEAK_UNLOCKED = 1e-3f;        // stronger drain off-signal (10x)
  static constexpr float TR_WOFF_CLAMP_UNLOCKED_FRAC = 0.25f;  // tighter clamp when not locked/capture
  static constexpr float TR_ANTI_WINDUP_BETA = 0.10f;          // back-calculation bleed (0 < beta <= 1)

  // Two-stage timing-loop gains. Acquisition gains are higher so the loop pulls
  // in quickly right after lock; once N_GOOD_TO_TRACK good bits have elapsed we
  // fall back to the gentle tracking gains (the configured fallback_kp/ki). The
  // acquisition window spans well into the early payload so the rate correction
  // (w_off_) has time to converge before the gains soften.
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

  // ---------------- Debug telemetry (read-only; does NOT affect timing) -------
  // Per-bit lines are emitted at VERBOSE; a rolling summary at DEBUG every
  // DBG_SUMMARY_EVERY bits (DBG_SUMMARY_EVERY is a file-scope constant in the
  // .cpp). All accumulators are fixed-size (no allocation).
  uint32_t dbg_bit_count_{0};
  float    dbg_conf_accum_{0.0f};
  uint16_t dbg_conf_n_{0};
  float    dbg_last_conf_{0.0f};

  // Per-bit VERBOSE telemetry decimation: only 1 in DBG_DECIM_K updated bits is
  // logged, capping the line rate (~520/8 ~= 65 lines/sec) so VERBOSE cannot
  // flood the transport and starve mic_task (which previously faulted the MCU).
  static constexpr uint32_t DBG_DECIM_K = 8;
  uint32_t dbg_decim_{0};

  // ---------------- Sample-rate self-measurement (read-only diagnostic) -------
  // Confirms the effective audio rate the decoder is actually receiving. Raw
  // samples are counted in feed_bytes (NOT feed_sample_, which can early-return
  // during post-emit dead-time). Once per ~second we log the measured Hz; if it
  // is far from the configured sample_rate_ (e.g. ~44100 vs 48000), the bit
  // clock is fundamentally mis-scaled and NO w_off_ correction can compensate.
  uint32_t sr_meas_samples_{0};   // raw samples fed since last measurement
  uint32_t sr_meas_last_ms_{0};   // millis() of last measurement (0 = uninit)

  // ---------------- Preamble-lock diagnostic (read-only; throttled) -----------
  // Logs WHICH gate (tone / balance / confidence) is blocking preamble lock.
  // Throttled to reason-changes + every PRE_DIAG_EVERY calls so it cannot flood
  // at 520 Hz. Purely observational; touches no DSP or lock state. Expected
  // finding with bit-centered detection: each clean preamble bit is a single
  // tone, so balance = |m-s|/(m+s) ~ 0.99 FAILS the 'balanced' gate.
  static constexpr int PRE_DIAG_EVERY = 32;
  int  pre_diag_bits_{0};
  bool pre_diag_last_qual_{false};
  bool pre_diag_last_bal_{false};
  bool pre_diag_last_tone_{false};
  int  pre_diag_last_run_{0};

  // ---------------- Preamble lock state (redesign) ----------------
  // The preamble is 0xAB = 10101011 repeated; LSB-first that is a regular
  // alternating-ish transition pattern. We qualify each bit-period as a valid
  // preamble bit-period when: (a) tone energy exceeds preamble_energy_mult_ over
  // the adaptive noise floor, and (b) mark/space are sufficiently balanced
  // across the alternation (|m-s|/(m+s) < preamble_balance_max_). When a run of
  // >= preamble_lock_bits_ qualifying bit-periods is seen at mean |LLR| >=
  // lock_confidence_min_, we declare LOCK and arm a ONE-SHOT phase snap that
  // aligns phase_ to bit-center (frequency/w_off_ untouched -- still bumpless).
  // The lock is CARRIED across the ~1s inter-burst gap (see LOCK_HOLD_MS); it is
  // fully torn down on emit/EOM/watchdog/feed-gap.
  //
  // NOTE (diagnostic in progress): the 'balanced' gate above is suspected to be
  // incompatible with bit-centered detection (clean preamble bits are single
  // tones => high imbalance). The PRE diag logging confirms this before any fix.
  int preamble_lock_bits_{32};
  std::atomic<float> preamble_energy_mult_{8.0f};
  float preamble_balance_max_{0.40f};
  float lock_confidence_min_{0.5f};
  float residual_drift_ppm_{2000.0f};

  float pre_noise_floor_{1.0f};
  static constexpr float PRE_FLOOR_ALPHA = 0.01f;
  int pre_run_{0};              // consecutive qualifying preamble bit-periods
  double pre_run_conf_sum_{0};  // sum of |LLR| over the current run
  bool preamble_locked_{false};
  bool was_preamble_locked_{false};
  uint32_t last_lock_ms_{0};    // millis() of most recent lock (0 = never/torn down)
  int byte_phase_{0};           // reserved/diagnostic; not used by the payload path

  // --- Phase-snap state (one-shot bit-center alignment) ---
  // last_centers_ holds the ring index of recent decided bit-centers (absolute
  // sample index of the bit instant). A small history is enough for the snap
  // search; RINGLEN=256 safely covers the last two centers plus GWIN/TR_DELTA.
  static constexpr int SNAP_CENTERS = 4;      // capacity of the recent-center log
  static constexpr int SNAP_USE = 2;          // centers actually used in the search
  static constexpr int SNAP_THETA_MAX = HALF_BEFORE;  // +/- search span in samples
  static constexpr int SNAP_THETA_STEP = 3;   // grid step in samples
  uint32_t center_hist_[SNAP_CENTERS]{0};     // absolute sample index of each center
  int center_hist_count_{0};                  // how many valid entries
  int center_hist_pos_{0};                    // next write slot (ring)
  bool snap_pending_{false};                  // a snap adjustment is armed
  float snap_delta_phase_{0.0f};              // fraction-of-bit adjustment to apply

  // Lock is carried across the inter-burst gap for up to this long, then
  // time-aged out. Sized above the standard's ~1s inter-burst pause but short
  // enough that it cannot bridge a full message boundary.
  static constexpr uint32_t LOCK_HOLD_MS = 1200;

  // Timing tracker (early-late). Gains are shared across the whole session; the
  // always-on gate (above) decides when they are applied.
  float fallback_conf_thresh_{0.20f};
  float fallback_kp_{0.08f};
  float fallback_ki_{0.0025f};
  bool fallback_active_{false};      // diagnostic: low-confidence run in progress
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

  // Low-trust: a capture that started from a ZCZC match while NOT effectively
  // locked. Such a capture can never emit on its own and must pass strict
  // structural validation + 2-of-3 corroboration before it counts.
  bool cur_low_trust_{false};                     // current in-progress capture
  bool burst_low_trust_[3]{false, false, false};  // per-collected-burst flag

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
