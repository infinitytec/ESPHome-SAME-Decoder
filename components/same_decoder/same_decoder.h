// components/same_decoder/same_decoder.h
#pragma once

#include "esphome/core/component.h"
#include "esphome/core/automation.h"
#include "esphome/components/i2s_audio/i2s_audio.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"

#include <string>
#include <vector>
#include <cstdint>

namespace esphome {
namespace same_decoder {

// Parsed fields of one decoded SAME header.
struct SameAlert {
  std::string id;            // DERIVED stable hash (event_code+areas+onset)
  std::string event_code;    // REAL  (EEE)
  std::string event_name;    // DERIVED (code table -> name)
  std::string severity;      // DERIVED (code table -> tier)
  std::string originator;    // REAL  (ORG)
  std::string status;        // DERIVED (Actual/Test)
  std::string areas_csv;     // REAL  (PSSCCC list, comma-separated)
  std::string onset_iso;     // DERIVED (JJJHHMM -> ISO8601)
  std::string expires_iso;   // DERIVED (purge -> ISO8601)
  std::string sender;        // REAL  (LLLLLLLL)
  std::string raw_header;    // REAL  (full ZCZC-...)
};

// Automation trigger fired once per decoded alert.
// Defined BEFORE SAMEDecoder uses it. No parameters — on_alert lambdas read
// the accessors off the component via id(same_dec) in YAML.
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

  // ---- Setters called from codegen (__init__.py) ----
  void set_i2s_parent(i2s_audio::I2SAudioComponent *parent) { this->i2s_parent_ = parent; }
  void set_din_pin(uint8_t pin) { this->din_pin_ = pin; }
  void set_sample_rate(uint32_t rate) { this->sample_rate_ = rate; }
  void set_decode_count_sensor(sensor::Sensor *s) { this->decode_count_sensor_ = s; }
  void set_last_raw_sensor(text_sensor::TextSensor *s) { this->last_raw_sensor_ = s; }
  void register_alert_trigger(AlertTrigger *t) { this->alert_triggers_.push_back(t); }

  // ---- Public accessors used by on_alert lambdas in the YAML ----
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
  // ---- DSP pipeline ----
  size_t read_samples_(int16_t *buf, size_t max_samples);   // hardware-binding (port)
  bool start_i2s_();                                        // arm demod
  bool find_preamble_();                                    // preamble hunt (0xAB alternation)
  bool assemble_burst_(std::string &header_out);            // 3-burst 2-of-3 vote

  // ---- Parsing / mapping (implemented) ----
  bool parse_header_(const std::string &header, SameAlert &out);
  std::string describe_(const std::string &code);       // code -> name
  std::string severity_for_(const std::string &code);   // code -> tier
  std::string make_id_(const SameAlert &a);             // stable hash
  void publish_alert_(const SameAlert &a);              // fire trigger + sinks

  // ---- Config ----
  i2s_audio::I2SAudioComponent *i2s_parent_{nullptr};
  uint8_t din_pin_{35};
  uint32_t sample_rate_{48000};
  sensor::Sensor *decode_count_sensor_{nullptr};
  text_sensor::TextSensor *last_raw_sensor_{nullptr};
  std::vector<AlertTrigger *> alert_triggers_;

  // ---- State ----
  SameAlert last_{};
  uint32_t decode_count_{0};
  bool i2s_ready_{false};

  // ---- DSP state ----
  bool last_preamble_bit_{false};
  int  preamble_run_{0};
  static constexpr int PREAMBLE_MIN_ALT = 24;    // consecutive alt bits to lock
  static constexpr int MAX_HEADER_BYTES = 268;   // SAME max header length cap
};

}  // namespace same_decoder
}  // namespace esphome
