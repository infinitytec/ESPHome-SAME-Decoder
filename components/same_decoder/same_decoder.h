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
  std::string onset_iso;
  std::string expires_iso;
  std::string sender;
  std::string raw_header;
};

class AlertTrigger : public Trigger<> {
 public:
  AlertTrigger() = default;
};

// Fires when a ZCZC preamble is acquired (a SAME burst has begun). Used as a
// "signal detected / decode-in-progress" indicator. Marshalled to the main
// loop thread (see loop()) so any Native API / light interaction is safe.
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

  // Called from YAML (api on_client_connected / on_client_disconnected) to tell
  // the decoder whether the Native API is currently usable. When it transitions
  // to connected, loop() flushes any alerts that were buffered while offline.
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
  void vote_and_emit_();
  void finish_burst_();

  bool parse_header_(const std::string &header, SameAlert &out);
  std::string describe_(const std::string &code);
  std::string severity_for_(const std::string &code);
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
  std::atomic<uint32_t> q_head_{0};   // consumer index (written by loop)
  std::atomic<uint32_t> q_tail_{0};   // producer index (written by mic_task)

  // ---- Sync-detected hand-off (mic_task -> main loop) ----
  std::atomic<uint32_t> sync_pending_{0};

  // ---- Guaranteed alert delivery (survive brief API disconnects) ----
  // homeassistant.event is fire-and-forget: if the API is disconnected when an
  // alert is dispatched, the event is silently dropped. For a safety decoder we
  // must not lose a decode during a momentary reconnect. So dispatch_alert_()
  // calls deliver_or_buffer_(): if the API is connected, fire immediately;
  // otherwise stash the alert in pending_. On the next connect
  // (set_api_connected(true)), loop() calls flush_pending_() to fire every
  // buffered alert in order. All of this runs on the main loop thread.
  bool api_connected_{false};
  std::atomic<bool> api_connected_changed_{false};   // set by set_api_connected(), drained in loop()
  static constexpr size_t PENDING_MAX = 16;           // max buffered alerts across a disconnect
  std::vector<SameAlert> pending_;                    // alerts awaiting a live API

  // ---- Timing / sampling ----
  static constexpr float SAMPLES_PER_BIT = 92.16f;
  static constexpr float PHASE_INC       = 1.0f / 92.16f;   // 0.01085069 nominal
  static constexpr int   GWIN            = 64;              // Goertzel window (< bit)
  static constexpr int   RINGLEN         = 256;            // ring size
  int16_t ring_[RINGLEN];                                   // circular sample history
  int     ring_pos_{0};
  float   phase_{0.0f};                                     // bit-clock phase accumulator

  // ---- Soft clipping (input headroom) ----
  // The hardware PGA (ES8388 ADCControl1) plus the software gain can drive
  // samples past full scale. Hard-clipping to +-32767 turns the AFSK tones into
  // square waves and destroys the mark/space Goertzel ratio, which is fatal for
  // marginal/trimmed captures. Instead we soft-clip with a tanh-style knee: the
  // signal is linear up to SOFT_KNEE and then compresses smoothly toward the
  // rail, preserving tone shape even when the slider is pushed high.
  static constexpr float SAMPLE_MAX   = 32767.0f;
  static constexpr float SOFT_KNEE    = 24576.0f;          // 0.75 * full scale: linear below this
  static float soft_clip_(float v);

  // ---- Timing recovery (early/late gate) ----
  static constexpr int   CENTER_LAG      = GWIN / 2;
  static constexpr int   TR_DELTA        = 12;
  static constexpr float TR_KP           = 0.06f;
  static constexpr float TR_KI           = 0.0015f;
  static constexpr float TR_CONF_MIN     = 0.20f;          // lowered for low-level captures
  static constexpr float TR_DPHI_CLAMP   = 0.125f;
  static constexpr float TR_WOFF_CLAMP   = 0.002f * PHASE_INC;
  float   w_off_{0.0f};
  uint32_t samples_seen_{0};

  // ---- Burst-collection timeout (Option A, mic_task driven) ----
  uint32_t last_burst_ms_{0};
  std::atomic<uint32_t> timeout_ms_{3000};

  // ---- Sync / framing ----
  enum Phase { HUNT_SYNC, CAPTURE };
  Phase   phase_state_{HUNT_SYNC};
  uint32_t sync_shift_{0};

  static constexpr uint32_t SYNC_ZCZC =
      (static_cast<uint32_t>('C') << 24) |
      (static_cast<uint32_t>('Z') << 16) |
      (static_cast<uint32_t>('C') << 8) |
      (static_cast<uint32_t>('Z'));                          // == 0x435A435A

  // Fuzzy ZCZC match tolerance: accept the preamble if the rolling 32-bit
  // window is within this Hamming distance of the ideal SYNC_ZCZC pattern.
  static constexpr int SYNC_MAX_HAMMING = 1;

  // ---- Byte / burst assembly ----
  uint8_t     cur_byte_{0};
  int         cur_nbits_{0};
  std::string cur_burst_;
  std::string bursts_[3];
  int         burst_idx_{0};
  static constexpr int MAX_HEADER_BYTES = 268;

  // ---- Early-emit on agreeing bursts ----
  static constexpr int MIN_BURSTS_TO_EMIT = 2;              // emit once this many agree
  static constexpr float BURST_MAX_MISMATCH = 0.10f;        // <=10% differing chars = "agree"
  bool bursts_agree_(int count);

  // ---- Header termination (structure-driven) ----
  bool  plus_seen_{false};
  int   tail_count_{0};
  static constexpr int TAIL_MIN = 14;
  static constexpr int TAIL_COMPLETE = 21;
};

}  // namespace same_decoder
}  // namespace esphome
