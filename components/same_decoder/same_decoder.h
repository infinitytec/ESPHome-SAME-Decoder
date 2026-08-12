// components/same_decoder/same_decoder.h
#pragma once

#include "esphome/core/component.h"
#include "esphome/core/automation.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"

#include <string>
#include <vector>
#include <cstdint>

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

class SAMEDecoder : public Component {
 public:
  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::LATE; }

  void set_sample_rate(uint32_t rate) { this->sample_rate_ = rate; }
  void set_gain(float g) { this->gain_ = g; }
  void set_decode_count_sensor(sensor::Sensor *s) { this->decode_count_sensor_ = s; }
  void set_last_raw_sensor(text_sensor::TextSensor *s) { this->last_raw_sensor_ = s; }
  void register_alert_trigger(AlertTrigger *t) { this->alert_triggers_.push_back(t); }

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

  bool parse_header_(const std::string &header, SameAlert &out);
  std::string describe_(const std::string &code);
  std::string severity_for_(const std::string &code);
  std::string make_id_(const SameAlert &a);
  void publish_alert_(const SameAlert &a);

  uint32_t sample_rate_{48000};
  float gain_{1.0f};
  sensor::Sensor *decode_count_sensor_{nullptr};
  text_sensor::TextSensor *last_raw_sensor_{nullptr};
  std::vector<AlertTrigger *> alert_triggers_;

  SameAlert last_{};
  uint32_t decode_count_{0};

  // ---- Timing / sampling ----
  static constexpr float SAMPLES_PER_BIT = 92.16f;
  static constexpr float PHASE_INC       = 1.0f / 92.16f;   // 0.01085069
  static constexpr int   GWIN            = 64;              // Goertzel window (< bit)
  int16_t ring_[128];                                       // circular sample history
  int     ring_pos_{0};
  float   phase_{0.0f};                                     // bit-clock phase accumulator

  // ---- Sync / framing ----
  enum Phase { HUNT_SYNC, CAPTURE };
  Phase   phase_state_{HUNT_SYNC};
  uint32_t sync_shift_{0};                                  // rolling 32-bit bit history

  // LSB-first 'ZCZC' = 0x5A 0x43 0x5A 0x43 transmitted LSB-first.
  // As a 32-bit value with FIRST-received bit in LSB position of the window,
  // the assembled pattern is 0x435A435A. The hunt shifter inserts the newest
  // bit at bit 31 and shifts older bits toward LSB, so after 32 bits the first
  // received 'Z' occupies the low byte and the last received 'C' the high byte.
  // Built from characters so the value can't silently drift again:
  //   high byte -> last received 'C', low byte -> first received 'Z'.
  static constexpr uint32_t SYNC_ZCZC =
      (static_cast<uint32_t>('C') << 24) |
      (static_cast<uint32_t>('Z') << 16) |
      (static_cast<uint32_t>('C') << 8) |
      (static_cast<uint32_t>('Z'));                          // == 0x435A435A

  // ---- Byte / burst assembly ----
  uint8_t     cur_byte_{0};
  int         cur_nbits_{0};
  std::string cur_burst_;
  std::string bursts_[3];
  int         burst_idx_{0};
  static constexpr int MAX_HEADER_BYTES = 268;
};

}  // namespace same_decoder
}  // namespace esphome
