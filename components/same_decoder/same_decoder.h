// my_components/same_decoder/same_decoder.h
#pragma once
#include "esphome/core/component.h"
#include "esphome/core/automation.h"
#include "esphome/components/i2s_audio/i2s_audio.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include <string>
#include <vector>

namespace esphome {
namespace same_decoder {

// Parsed SAME header, modeled loosely on the CAP fields cap_alerts uses.
struct SAMEMessage {
  std::string originator;              // ORG:  WXR / CIV / EAS / PEP
  std::string event_code;             // EEE:  TOR, SVR, RWT, ...
  std::vector<std::string> areas;     // PSSCCC list (6-digit each)
  std::string purge;                  // +TTTT
  std::string issued;                 // JJJHHMM
  std::string sender;                 // LLLLLLLL station id
  std::string raw;                    // full ZCZC-...-+... string
};

class OnAlertTrigger : public Trigger<> {};

class SAMEDecoder : public Component {
 public:
  void setup() override;
  void loop() override;
  float get_setup_priority() const override { return setup_priority::LATE; }

  // --- setters wired from __init__.py ---
  void set_i2s_parent(i2s_audio::I2SAudioComponent *p) { this->i2s_ = p; }
  void set_din_pin(int pin) { this->din_pin_ = pin; }
  void set_sample_rate(int sr) { this->sample_rate_ = sr; }
  void set_fips_source(text_sensor::TextSensor *s) { this->fips_source_ = s; }
  void set_alert_active(binary_sensor::BinarySensor *s) { this->alert_active_ = s; }
  void set_event_code_sensor(text_sensor::TextSensor *s) { this->t_event_code_ = s; }
  void set_event_sensor(text_sensor::TextSensor *s) { this->t_event_ = s; }
  void set_severity_sensor(text_sensor::TextSensor *s) { this->t_severity_ = s; }
  void set_originator_sensor(text_sensor::TextSensor *s) { this->t_originator_ = s; }
  void set_fips_affected_sensor(text_sensor::TextSensor *s) { this->t_areas_ = s; }
  void set_raw_header_sensor(text_sensor::TextSensor *s) { this->t_raw_ = s; }
  void set_sender_sensor(text_sensor::TextSensor *s) { this->t_sender_ = s; }
  void set_expires_sensor(sensor::Sensor *s) { this->s_expires_ = s; }
  void add_on_alert_trigger(OnAlertTrigger *t) { this->alert_triggers_.push_back(t); }

 protected:
  // ---- I2S ----
  bool start_i2s_();
  int read_samples_(int16_t *buf, int max);

  // ---- DSP: AFSK demodulation (520.83 baud, mark 2083.3 Hz, space 1562.5 Hz) ----
  // TODO(tuning): Goertzel or FIR correlator per bit period. This is the part
  // that MUST be tuned against captured live NWR audio; the skeleton below
  // performs a per-bit tone comparison and needs real-world hardening for
  // noise, DC offset, and clock drift.
  bool demodulate_bit_(const int16_t *samples, int n, bool &bit_out);

  // ---- Framing / protocol ----
  bool find_preamble_();               // 0xAB x16 then "ZCZC"
  bool assemble_burst_(std::string &out);
  bool parse_header_(const std::string &burst, SAMEMessage &msg);

  // ---- Matching + publish ----
  bool matches_my_fips_(const SAMEMessage &msg);
  std::string severity_for_(const std::string &event_code);   // -> CAP tier
  std::string describe_(const std::string &event_code);       // -> "Tornado Warning"
  void publish_(const SAMEMessage &msg, bool mine);

  // config
  i2s_audio::I2SAudioComponent *i2s_{nullptr};
  int din_pin_{35};
  int sample_rate_{48000};

  // entities
  text_sensor::TextSensor *fips_source_{nullptr};
  binary_sensor::BinarySensor *alert_active_{nullptr};
  text_sensor::TextSensor *t_event_code_{nullptr};
  text_sensor::TextSensor *t_event_{nullptr};
  text_sensor::TextSensor *t_severity_{nullptr};
  text_sensor::TextSensor *t_originator_{nullptr};
  text_sensor::TextSensor *t_areas_{nullptr};
  text_sensor::TextSensor *t_raw_{nullptr};
  text_sensor::TextSensor *t_sender_{nullptr};
  sensor::Sensor *s_expires_{nullptr};
  std::vector<OnAlertTrigger *> alert_triggers_;

  // demod state
  std::vector<uint8_t> bit_buffer_;
};

}  // namespace same_decoder
}  // namespace esphome
