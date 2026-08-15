// components/same_decoder/same_decoder.h
// Soft-decision SAME decoder with continuous timing recovery, AB preamble
// lock + phase re-center, multi-burst voting, and decode watchdog.
//
// SAME digital format (NWS / FCC):
//   - Preamble: 16 × 0xAB (binary 10101011, LSB first) before every header
//     and every EOM. Used for bit sync, byte framing, and AGC.
//   - Data: 520 + 5/6 baud AFSK, mark 2083⅓ Hz, space 1562.5 Hz,
//     continuous phase, 8-bit bytes, no start/stop bits.
//   - Header begins with ASCII "ZCZC" and is repeated three times.
//
// Primary arming path is a robust AB correlator that, once locked, forces a
// phase re-center so the continuous early/late TR loop starts from a known
// mid-bit sampling point. Short ZC/ZCZC tiers remain as secondary fallbacks.
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
  void set_ab_required(bool v) { this->ab_required_ = v; }

  void set_preamble_status(bool v) { this->preamble_status_ = v; }
  void set_preamble_energy_mult(float v) { this->preamble_energy_mult_.store(v, std::memory_order_relaxed); }

  void set_eom_require_context(bool v) { this->eom_require_context_ = v; }
  void set_eom_context_ms(uint32_t v) { this->eom_context_ms_ = v; }

  void set_decode_watchdog_ms(uint32_t v) { this->decode_watchdog_ms_.store(v, std::memory_order_relaxed); }

  void set_decode_count_sensor(sensor::Sensor *s) { this->decode_count_sensor_ = s; }
  void set_last_raw_sensor(text_sensor::TextSensor *s) { this->last_raw_sensor_ = s; }
  void register_alert_trigger(AlertTrigger *t) { this->alert_triggers_.push_back(t); }
  void register_sync_trigger(SyncTrigger *t) { this->sync_triggers_.push_back(t); }
  void register_eom_trigger(EomTrigger *t) { this->eom_triggers_.push_back(t); }

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
  void reprime_detector_();
  void reset_capture_();
  void begin_new_capture_(int preamble_len, bool fallback);
  void vote_and_emit_(bool from_timeout, bool fallback_synced);
  void finish_burst_();
  void compute_coeffs_();
  void update_agc_(float mag);
  bool ab_preamble_ok_() const;
  void note_emit_();
  void check_decode_watchdog_();
  void lock_from_ab_preamble_();

  void update_preamble_gate_(float mark_e, float space_e);

  void fire_sync_once_();
  void fire_eom_();

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

  SameAlert last_{};
  uint32_t decode_count_{0};

  static constexpr int ALERT_Q_LEN = 4;
  SameAlert alert_queue_[ALERT_Q_LEN];
  std::atomic<uint32_t> q_head_{0};
  std::atomic<uint32_t> q_tail_{0};
  std::atomic<uint32_t> sync_pending_{0};
  std::atomic<uint32_t> eom_pending_{0};

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
  static constexpr int GWIN = 64;
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
  static constexpr float AGC_PEAK_ALPHA = 0.001f;
  static constexpr float AGC_ATTACK = 0.002f;
  static constexpr float AGC_RELEASE = 0.0002f;

  static constexpr int CENTER_LAG = GWIN / 2;
  static constexpr int TR_DELTA = 12;
  static constexpr float TR_KP = 0.06f;
  static constexpr float TR_KI = 0.0015f;
  static constexpr float TR_CONF_MIN = 0.20f;
  static constexpr float TR_DPHI_CLAMP = 0.125f;
  float tr_woff_clamp_{0.002f * (1.0f / 92.16f)};
  float w_off_{0.0f};
  uint32_t samples_seen_{0};

  float last_bit_llr_{0.0f};
  SoftBurst soft_bursts_[3];
  SoftBurst soft_cur_;
  static constexpr float LLR_EPS = 1.0f;
  int bad_char_run_{0};

  bool preamble_status_{true};
  std::atomic<float> preamble_energy_mult_{8.0f};
  float pre_noise_floor_{1.0f};
  static constexpr float PRE_FLOOR_ALPHA = 0.01f;
  static constexpr float PRE_BALANCE_MAX = 0.40f;
  bool preamble_present_{false};
  uint32_t pre_on_ms_{0};
  uint32_t pre_off_ms_{0};
  bool tone_gate_{false};
  static constexpr uint32_t PRE_ON_DWELL_MS = 25;
  static constexpr uint32_t PRE_OFF_DWELL_MS = 150;

  uint32_t last_burst_ms_{0};
  std::atomic<uint32_t> timeout_ms_{3000};
  uint32_t single_burst_min_ms_{7000};
  uint32_t post_emit_dead_ms_{800};
  uint32_t last_emit_ms_{0};

  bool eom_require_context_{true};
  uint32_t eom_context_ms_{120000};
  uint32_t last_valid_header_ms_{0};

  // Decode watchdog: if a decode session (LED on) persists this long without a
  // successful emit or EOM, abandon it (emit valid partial, else discard) and
  // clear the indicator so state never hangs.
  std::atomic<uint32_t> decode_watchdog_ms_{10000};
  bool decode_active_{false};        // true from sync until emit/EOM/watchdog
  uint32_t decode_active_since_{0};  // millis() when decode_active_ went true

  // AB preamble (0xAB = 10101011 LSB-first). Spec requires 16 consecutive
  // bytes before every header/EOM. We track a running match quality that
  // tolerates single-bit errors and, once locked, force a phase re-center.
  // The lock is latched for AB_LOCK_HOLD_MS after acquisition so that the
  // subsequent non-AB header bits do not immediately clear it (the preamble
  // is only ~246 ms long at 520.83 baud; ZCZC follows immediately after).
  bool ab_required_{false};
  uint8_t ab_byte_{0};
  int ab_nbits_{0};
  int ab_match_count_{0};
  bool ab_locked_{false};
  uint32_t ab_lock_ms_{0};   // millis() when lock was acquired
  // Minimum consecutive good AB bytes to declare lock and re-center phase.
  // Spec is 16; 6 is a practical compromise for noisy streams while limiting
  // false locks on random audio.
  static constexpr int AB_MIN_MATCH = 2;       // legacy soft gate
  static constexpr int AB_LOCK_THRESH = 6;     // strong lock → phase re-center
  static constexpr int AB_MAX_COUNT = 32;
  // Hold the lock this long after acquisition so ZCZC hunt still sees it.
  // 16 AB bytes ≈ 246 ms; hold a bit longer to cover the start of the header.
  static constexpr uint32_t AB_LOCK_HOLD_MS = 500;

  bool ab_lock_active_() const;

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

  static constexpr uint32_t SYNC_ZC_16 =
      (static_cast<uint32_t>('C') << 8) |
      (static_cast<uint32_t>('Z'));

  static constexpr uint32_t MASK16 = 0x0000FFFFu;
  static constexpr uint32_t MASK24 = 0x00FFFFFFu;
  bool fallback_sync_used_{false};

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
