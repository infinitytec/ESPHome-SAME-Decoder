// components/same_decoder/same_decoder.cpp
#include "same_decoder.h"
#include "same_event_codes.h"
#include "esphome/core/log.h"

#include <functional>   // std::hash
#include <sstream>
#include <iomanip>

namespace esphome {
namespace same_decoder {

static const char *const TAG = "same_decoder";

void SAMEDecoder::setup() {
  ESP_LOGCONFIG(TAG, "Setting up SAME decoder (din=GPIO%u, %u Hz)...",
                this->din_pin_, this->sample_rate_);
  this->i2s_ready_ = this->start_i2s_();
  if (!this->i2s_ready_) {
    ESP_LOGW(TAG, "I2S RX not configured yet (DSP scaffold). Decoder idle.");
  }
}

void SAMEDecoder::dump_config() {
  ESP_LOGCONFIG(TAG, "SAME Decoder:");
  ESP_LOGCONFIG(TAG, "  DIN pin: GPIO%u", this->din_pin_);
  ESP_LOGCONFIG(TAG, "  Sample rate: %u Hz", this->sample_rate_);
  ESP_LOGCONFIG(TAG, "  Alert triggers: %u", (unsigned) this->alert_triggers_.size());
  ESP_LOGCONFIG(TAG, "  NOTE: DSP core is scaffold — no live decodes yet.");
}

void SAMEDecoder::loop() {
  // ---- DSP pipeline is stubbed; this is the intended shape. ----
  if (!this->i2s_ready_)
    return;

  if (!this->find_preamble_())
    return;

  std::string header;
  if (!this->assemble_burst_(header))
    return;

  SameAlert alert;
  if (!this->parse_header_(header, alert))
    return;

  this->publish_alert_(alert);
}

// ---------------- DSP STUBS (the real work remains) ----------------
size_t SAMEDecoder::read_samples_(int16_t *buf, size_t max_samples) {
  (void) buf; (void) max_samples;
  return 0;  // TODO: read PCM from I2S RX channel.
}
bool SAMEDecoder::start_i2s_() {
  // TODO: configure the I2S RX channel on din_pin_ at sample_rate_.
  return false;
}
bool SAMEDecoder::find_preamble_() {
  // TODO: Goertzel/AFSK preamble + ZCZC hunt.
  return false;
}
bool SAMEDecoder::assemble_burst_(std::string &header_out) {
  (void) header_out;
  // TODO: clock recovery + 3-burst majority vote -> full header string.
  return false;
}

// ---------------- Parsing / mapping (implemented) ----------------
std::string SAMEDecoder::describe_(const std::string &code) {
  auto it = SAME_EVENT_CODES.find(code);
  if (it != SAME_EVENT_CODES.end())
    return it->second.name;
  return code;  // unknown code: fall back to the raw 3-letter code
}

std::string SAMEDecoder::severity_for_(const std::string &code) {
  auto it = SAME_EVENT_CODES.find(code);
  if (it != SAME_EVENT_CODES.end())
    return it->second.severity;
  return "Unknown";
}

std::string SAMEDecoder::make_id_(const SameAlert &a) {
  // Stable across re-broadcasts of the SAME alert: event+areas+onset.
  std::string key = a.event_code + "|" + a.areas_csv + "|" + a.onset_iso;
  size_t h = std::hash<std::string>{}(key);
  std::ostringstream os;
  os << std::hex << h;
  return os.str();
}

bool SAMEDecoder::parse_header_(const std::string &header, SameAlert &out) {
  // Expected: ZCZC-ORG-EEE-PSSCCC-PSSCCC...+TTTT-JJJHHMM-LLLLLLLL-
  // NOTE: This is the field-splitting skeleton. Timestamp math (JJJHHMM/purge
  // -> ISO) is left as a TODO to keep this honest; wire it when validating
  // against real captures.
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

  out.originator = parts[1];                 // REAL
  out.event_code = parts[2];                 // REAL
  out.event_name = this->describe_(out.event_code);        // DERIVED
  out.severity   = this->severity_for_(out.event_code);    // DERIVED

  // Areas: everything after event code until the token containing '+'.
  std::string areas;
  for (size_t i = 3; i < parts.size(); i++) {
    if (parts[i].find('+') != std::string::npos) break;
    if (!areas.empty()) areas += ",";
    areas += parts[i];
  }
  out.areas_csv = areas;                     // REAL

  // Status: RWT/RMT/DMO/NPT => Test, else Actual (simple heuristic).
  out.status = (out.event_code == "RWT" || out.event_code == "RMT" ||
                out.event_code == "DMO" || out.event_code == "NPT")
                   ? "Test" : "Actual";      // DERIVED

  // TODO: parse purge (+TTTT) and issue (JJJHHMM) into onset_iso/expires_iso.
  out.onset_iso = "";                        // DERIVED (TODO)
  out.expires_iso = "";                      // DERIVED (TODO)
  out.sender = parts.empty() ? "" : parts.back();  // REAL (LLLLLLLL, last field)

  out.id = this->make_id_(out);              // DERIVED stable hash
  return true;
}

void SAMEDecoder::publish_alert_(const SameAlert &a) {
  this->last_ = a;
  this->decode_count_++;

  if (this->decode_count_sensor_ != nullptr)
    this->decode_count_sensor_->publish_state((float) this->decode_count_);
  if (this->last_raw_sensor_ != nullptr)
    this->last_raw_sensor_->publish_state(a.raw_header);

  ESP_LOGI(TAG, "Decoded SAME: %s (%s) areas=%s",
           a.event_name.c_str(), a.event_code.c_str(), a.areas_csv.c_str());

  // Fire the on_alert automation(s); lambdas read the accessors off `this`.
  for (auto *t : this->alert_triggers_)
    t->trigger();
}

}  // namespace same_decoder
}  // namespace esphome
