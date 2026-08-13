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

  uint32_t sample_rate_{48000};
  float gain_{1.0f};
  sensor::Sensor *decode_count_sensor_{nullptr};
  text_sensor::TextSensor *last_raw_sensor_{nullptr};
  std::vector<AlertTrigger *> alert_triggers_;
  std::vector<SyncTrigger *> sync_triggers_;

  SameAlert last_{};
  uint32_t decode_count_{0};

  // ---- Cross-thread hand-off (mic_task -> main loop) ----
  // The DSP runs on the microphone task. publish_state()/Trigger::trigger()
  // send Native API traffic and MUST run on the main loop thread, so we queue
  // decoded alerts here and dispatch them from loop(). Lock-free single-
  // producer (mic_task) / single-consumer (loop) ring buffer so closely-spaced
  // alerts are not dropped (only dropped if the queue is genuinely full).
  static constexpr int ALERT_Q_LEN = 4;
  SameAlert alert_queue_[ALERT_Q_LEN];
  std::atomic<uint32_t> q_head_{0};   // consumer index (written by loop)
  std::atomic<uint32_t> q_tail_{0};   // producer index (written by mic_task)

  // ---- Sync-detected hand-off (mic_task -> main loop) ----
  // ZCZC acquisition happens on mic_task; the sync trigger must fire on the
  // main loop. A single atomic "pending" counter is incremented on mic_task
  // and drained (fired) in loop(). We only need edge notification, so a small
  // saturating counter is sufficient and lock-free.
  std::atomic<uint32_t> sync_pending_{0};

  // ---- Timing / sampling ----
  static constexpr float SAMPLES_PER_BIT = 92.16f;
  static constexpr float PHASE_INC       = 1.0f / 92.16f;   // 0.01085069 nominal
  static constexpr int   GWIN            = 64;              // Goertzel window (< bit)
  static constexpr int   RINGLEN         = 256;            // ring size (grown for early/late windows)
  int16_t ring_[RINGLEN];                                   // circular sample history
  int     ring_pos_{0};
  float   phase_{0.0f};                                     // bit-clock phase accumulator

  // ---- Timing recovery (early/late gate) ----
  static constexpr int   CENTER_LAG      = GWIN / 2;        // window end sits this far back of "now"
  static constexpr int   TR_DELTA        = 12;             // early/late offset in samples
  static constexpr float TR_KP           = 0.06f;          // proportional phase gain
  static constexpr float TR_KI           = 0.0015f;        // integral rate gain
  static constexpr float TR_CONF_MIN     = 0.20f;          // min confidence to adapt (lowered for low-level captures)
  static constexpr float TR_DPHI_CLAMP   = 0.125f;         // max per-bit phase nudge (bit fraction)
  static constexpr float TR_WOFF_CLAMP   = 0.002f * PHASE_INC;  // max rate correction (+-0.2%)
  float   w_off_{0.0f};
  uint32_t samples_seen_{0};

  // ---- Burst-collection timeout (Option A, mic_task driven) ----
  // If we have collected 1-2 header bursts but the 3rd never arrives within
  // timeout_ms_, emit what we have and reset so a missed burst does not block
  // the next alert. Checked inside feed_sample_ (mic_task), so all burst state
  // stays single-threaded. Note: if the audio feed goes fully silent, mic_task
  // stops running and the timeout cannot fire - acceptable for a continuous
  // radio feed. timeout_ms_ is atomic so the main-loop slider can adjust it.
  uint32_t last_burst_ms_{0};
  std::atomic<uint32_t> timeout_ms_{3000};

  // ---- Sync / framing ----
  enum Phase { HUNT_SYNC, CAPTURE };
  Phase   phase_state_{HUNT_SYNC};
  uint32_t sync_shift_{0};

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

  // Fuzzy ZCZC match tolerance: accept the preamble if the rolling 32-bit
  // window is within this Hamming distance of the ideal SYNC_ZCZC pattern.
  // 1 tolerates a single flipped bit in the preamble so a marginally-clipped
  // or noisy first burst still acquires. Kept small to bound false-sync risk.
  static constexpr int SYNC_MAX_HAMMING = 1;

  // ---- Byte / burst assembly ----
  uint8_t     cur_byte_{0};
  int         cur_nbits_{0};
  std::string cur_burst_;
  std::string bursts_[3];
  int         burst_idx_{0};
  static constexpr int MAX_HEADER_BYTES = 268;

  // ---- Early-emit on agreeing bursts ----
  // A full SAME message repeats the header 3x so a receiver can majority-vote.
  // If the first repeat is clipped/lost (common on trimmed test files), only 2
  // bursts arrive and, on a file feed, the mic_task timeout cannot fire once
  // audio ends - so the two good bursts would be discarded. To fix this we
  // emit as soon as 2 collected bursts agree closely (below the mismatch
  // ceiling), rather than always waiting for the 3rd. 3 bursts still vote as
  // before. This preserves the majority-vote guarantee while recovering
  // otherwise-lost 2-burst decodes.
  static constexpr int MIN_BURSTS_TO_EMIT = 2;              // emit once this many agree
  static constexpr float BURST_MAX_MISMATCH = 0.10f;        // <=10% differing chars = "agree"
  bool bursts_agree_(int count);

  // ---- Header termination (structure-driven) ----
  // A ZCZC header ends at the trailing '-' after the fixed tail
  // "+TTTT-JJJHHMM-LLLLLLLL". It does NOT contain "NNNN" (that is a separate
  // End-Of-Message transmission). We anchor on '+', then count the fixed tail.
  //  PRIMARY  : stop at the closing '-' once past TAIL_MIN.
  //  FALLBACK : if that dash is missed/garbled, stop once the full fixed tail
  //             length (TAIL_COMPLETE chars) has been consumed - the header is
  //             structurally complete by then, so an otherwise-good decode is
  //             not lost.
  // TAIL_MIN     = 4(TTTT) +1(-) +7(JJJHHMM) +1(-) +1 = 14 (earliest valid dash)
  // TAIL_COMPLETE= 4(TTTT) +1(-) +7(JJJHHMM) +1(-) +8(LLLLLLLL) = 21 (tail done)
  bool  plus_seen_{false};                                  // have we seen '+' yet
  int   tail_count_{0};                                     // chars counted since '+'
  static constexpr int TAIL_MIN = 14;                       // min tail chars before closing '-'
  static constexpr int TAIL_COMPLETE = 21;                  // full fixed tail length after '+'
};

}  // namespace same_decoder
}  // namespace esphome
