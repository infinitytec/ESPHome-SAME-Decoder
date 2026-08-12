// components/same_decoder/same_decoder.h
#pragma once

#include "esphome/core/component.h"
#include "esphome/core/automation.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"

#include <string>
#include <vector>
#include <cstdint>
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
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::LATE; }

  void set_sample_rate(uint32_t rate) { this->sample_rate_ = rate; }
  void set_gain(float g) { this->gain_ = g; }
  void set_burst_timeout_ms(uint32_t ms) { this->burst_timeout_ms_ = ms; }
  void set_decode_count_sensor(sensor::Sensor *s) { this->decode_count_sensor_ = s; }
  void set_last_raw_sensor(text_sensor::TextSensor *s) { this->last_raw_sensor_ = s; }
  void set_quality_sensor(sensor::Sensor *s) { this->quality_sensor_ = s; }
  void set_snr_sensor(sensor::Sensor *s) { this->snr_sensor_ = s; }
  void set_freq_offset_sensor(sensor::Sensor *s) { this->freq_offset_sensor_ = s; }
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
  void emit_bit_(bool hard_bit, float soft);
  void reset_capture_();
  void vote_and_emit_();
  void update_goertzel_coeffs_();
  float goertzel_ring_(int start, int n, float coeff) const;
  int hamming32_(uint32_t a, uint32_t b) const;

  bool parse_header_(const std::string &header, SameAlert &out);
  std::string describe_(const std::string &code);
  std::string severity_for_(const std::string &code);
  std::string make_id_(const SameAlert &a);
  void publish_alert_(const SameAlert &a);

  // ---- Configuration ----
  uint32_t sample_rate_{48000};
  float gain_{1.0f};
  // After the first burst of a group, finish with whatever we have if no more
  // bursts arrive within this window (SAME repeats are ~1 s apart; 12 s is safe).
  uint32_t burst_timeout_ms_{12000};

  sensor::Sensor *decode_count_sensor_{nullptr};
  text_sensor::TextSensor *last_raw_sensor_{nullptr};
  sensor::Sensor *quality_sensor_{nullptr};
  sensor::Sensor *snr_sensor_{nullptr};
  sensor::Sensor *freq_offset_sensor_{nullptr};
  std::vector<AlertTrigger *> alert_triggers_;

  SameAlert last_{};
  uint32_t decode_count_{0};

  // ---- Timing / sampling ----
  float samples_per_bit_{92.16f};
  float phase_inc_{1.0f / 92.16f};
  static constexpr int GWIN = 80;
  static constexpr int RING_LEN = 160;
  int16_t ring_[RING_LEN]{};
  int ring_pos_{0};
  float phase_{0.0f};

  static constexpr float EARLY_FRAC = 0.25f;
  static constexpr float LATE_FRAC = 0.25f;

  // ---- Frequency agility (multi-bin + light AFC) ----
  static constexpr float NOM_MARK_HZ = 2083.333f;
  static constexpr float NOM_SPACE_HZ = 1562.500f;
  static constexpr int NUM_BINS = 5;
  static constexpr float BIN_STEP_HZ = 30.0f;

  float coeff_mark_[NUM_BINS]{};
  float coeff_space_[NUM_BINS]{};
  int best_bin_{2};
  float freq_offset_hz_{0.0f};
  float afc_alpha_{0.08f};

  // ---- Soft decision / quality / SNR ----
  float soft_accum_{0.0f};
  int soft_count_{0};
  float burst_quality_[3]{0.f, 0.f, 0.f};
  float last_quality_{0.0f};
  float last_snr_db_{0.0f};

  // Running estimates for a simple SNR-style metric (mark/space vs residual).
  float energy_signal_{0.0f};
  float energy_noise_{0.0f};
  static constexpr float SNR_ALPHA = 0.05f;

  float agc_level_{1.0f};
  static constexpr float AGC_TARGET = 1.0e6f;
  static constexpr float AGC_ALPHA = 0.02f;

  // ---- Sync / framing ----
  enum Phase { HUNT_SYNC, CAPTURE };
  Phase phase_state_{HUNT_SYNC};
  uint32_t sync_shift_{0};

  static constexpr uint32_t SYNC_ZCZC =
      (static_cast<uint32_t>('C') << 24) |
      (static_cast<uint32_t>('Z') << 16) |
      (static_cast<uint32_t>('C') << 8) |
      (static_cast<uint32_t>('Z'));  // 0x435A435A

  static constexpr int MAX_SYNC_HAMMING = 2;

  // ---- Byte / burst assembly ----
  uint8_t cur_byte_{0};
  int cur_nbits_{0};
  std::string cur_burst_;
  std::string bursts_[3];
  int burst_idx_{0};
  static constexpr int MAX_HEADER_BYTES = 268;

  // Timestamp of first burst in the current group (millis). 0 = idle.
  uint32_t group_start_ms_{0};
  // Last time any bit activity advanced the capture state machine.
  uint32_t last_activity_ms_{0};
};

}  // namespace same_decoder
}  // namespace esphome
