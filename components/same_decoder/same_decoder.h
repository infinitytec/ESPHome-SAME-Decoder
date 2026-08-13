// components/same_decoder/same_decoder.h
#pragma once

#include "esphome/core/component.h"
#include "esphome/core/automation.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"

#include <string>
#include <vector>
#include <cstdint>
#include <atomic>

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
  std::string timing;        // valid-time + issue-time tail, e.g. "+0030-2780415"
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

class SAMEDecoder : public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::LATE; }

  void set_sample_rate(uint32_t rate) { this->sample_rate_ = rate; }
  void set_gain(float g) { this->gain_ = g; }
  float get_gain() const { return this->gain_; }
  void set_timeout_ms(uint32_t v) { this->timeout_ms_.store(v, std::memory_order_relaxed); }
  uint32_t get_timeout_ms() const { return this->timeout_ms_.load(std::memory_order_relaxed); }
  void set_decode_count_sensor(sensor::Sensor *s) { this->decode_count_sensor_ = s; }
  void set_last_raw_sensor(text_sensor::TextSensor *s) { this->last_raw_sensor_ = s; }
  void register_alert_trigger(AlertTrigger *t) { this->alert_triggers_.push_back(t); }
  void register_sync_trigger(SyncTrigger *t) { this->sync_triggers_.push_back(t); }

  void set_api_connected(bool connected);

  void feed_bytes(const std::vector<uint8_t> &data);

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
  void reset_capture_();
  void begin_new_capture_(bool fallback);
  void vote_and_emit_(bool from_timeout, bool fallback_synced);
  void finish_burst_();

  bool parse_header_(const std::string &header, SameAlert &out);
  bool header_is_strictly_valid_(const std::string &header);
  bool same_message_as_current_(const std::string &new_burst);
  bool fuzzy_equal_(const std::string &a, const std::string &b);
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
  sensor::Sensor *decode_count_sensor_{nullptr};
  text_sensor::TextSensor *last_raw_sensor_{nullptr};
  std::vector<AlertTrigger *> alert_triggers_;
  std::vector<SyncTrigger *> sync_triggers_;

  SameAlert last_{};
  uint32_t decode_count_{0};

  // ---- Cross-thread hand-off (mic_task -> main loop) ----
  static constexpr int ALERT_Q_LEN = 4;
  SameAlert alert_queue_[ALERT_Q_LEN];
  std::atomic<uint32_t> q_head_{0};
  std::atomic<uint32_t> q_tail_{0};

  // ---- Sync-detected hand-off (mic_task -> main loop) ----
  std::atomic<uint32_t> sync_pending_{0};

  // ---- Guaranteed alert delivery (survive brief API disconnects) ----
  bool api_connected_{false};
  std::atomic<bool> api_connected_changed_{false};
  static constexpr size_t PENDING_MAX = 16;
  std::vector<SameAlert> pending_;

  // ---- Dedup: SESSION-based (not wall-clock window) ----
  std::string session_emitted_header_;
  std::string last_global_header_;
  uint32_t    last_global_ms_{0};
  static constexpr uint32_t IMMEDIATE_DUP_MS = 3000;   // tight same-message spill guard

  // ---- Timing / sampling ----
  static constexpr float SAMPLES_PER_BIT = 92.16f;
  static constexpr float PHASE_INC       = 1.0f / 92.16f;
  static constexpr int   GWIN            = 64;
  static constexpr int   RINGLEN         = 256;
  int16_t ring_[RINGLEN];
  int     ring_pos_{0};
  float   phase_{0.0f};

  // ---- Soft clipping (input headroom) ----
  static constexpr float SAMPLE_MAX   = 32767.0f;
  static constexpr float SOFT_KNEE    = 24576.0f;
  static float soft_clip_(float v);

  // ---- Timing recovery (early/late gate) ----
  static constexpr int   CENTER_LAG      = GWIN / 2;
  static constexpr int   TR_DELTA        = 12;
  static constexpr float TR_KP           = 0.06f;
  static constexpr float TR_KI           = 0.0015f;
  static constexpr float TR_CONF_MIN     = 0.20f;
  static constexpr float TR_DPHI_CLAMP   = 0.125f;
  static constexpr float TR_WOFF_CLAMP   = 0.002f * PHASE_INC;
  float   w_off_{0.0f};
  uint32_t samples_seen_{0};

  // ---- Burst-collection timeout ----
  uint32_t last_burst_ms_{0};
  std::atomic<uint32_t> timeout_ms_{3000};
  static constexpr uint32_t SINGLE_BURST_MIN_MS = 7000;

  // ---- Sync / framing ----
  enum Phase { HUNT_SYNC, CAPTURE };
  Phase   phase_state_{HUNT_SYNC};
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
  static constexpr uint32_t MASK24 = 0x00FFFFFFu;
  bool fallback_sync_used_{false};

  // ---- Byte / burst assembly ----
  uint8_t     cur_byte_{0};
  int         cur_nbits_{0};
  std::string cur_burst_;
  std::string bursts_[3];
  int         burst_idx_{0};
  static constexpr int MAX_HEADER_BYTES = 268;

  // ---- Early-emit on agreeing bursts ----
  static constexpr int MIN_BURSTS_TO_EMIT = 2;
  static constexpr float BURST_MAX_MISMATCH = 0.10f;
  bool bursts_agree_(int count);
  bool early_emitted_{false};

  // Fuzzy tolerance (fraction of differing chars) used only for ORG/EEE when
  // deciding same-vs-new message. Area and timing are compared EXACTLY.
  static constexpr float FIELD_FUZZ = 0.20f;

  // ---- Header termination (structure-driven) ----
  bool  plus_seen_{false};
  int   tail_count_{0};
  static constexpr int TAIL_MIN = 14;
  static constexpr int TAIL_COMPLETE = 21;
};

}  // namespace same_decoder
}  // namespace esphome
