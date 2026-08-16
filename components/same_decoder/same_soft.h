// components/same_decoder/same_soft.h
// Soft-decision SAME burst combiner (best-2-of-3).
//
// Given up to 3 bursts, each stored as an array of per-bit pseudo-LLRs
//   l = ln(Em) - ln(Es)
// (additive across repeats, gain-invariant), this module:
//   1. Slices each burst into 8-bit LSB-first ASCII characters.
//   2. Dash-anchors and field-aligns the bursts (bounding slip damage to a field).
//   3. Combines per-bit LLRs across aligned bursts and hard-decides.
//   4. Applies light SAME grammar to score/repair (lenient: never hard-blocks on
//      an unknown event code).
// It NEVER touches the DSP/clock; it only post-processes buffered LLRs.
//
// LSB-first is mandated by the SAME/EAS spec: "The least-significant bit of each
// byte is transmitted first, including the preamble." The 8th bit is the ASCII
// null bit (7-bit ASCII + one null bit = full 8-bit byte, per 47 CFR 11.31).
#pragma once

#include <string>
#include <vector>
#include <cstdint>

namespace esphome {
namespace same_decoder {

// One captured burst: the raw per-bit LLRs (LSB-first, 8 per character).
struct SoftBurst {
  std::vector<float> llr;   // one entry per received bit
  void clear() { this->llr.clear(); }
  size_t nbits() const { return this->llr.size(); }
  size_t nchars() const { return this->llr.size() / 8; }
};

struct SoftResult {
  std::string header;       // best combined header string
  float mean_margin{0.0f};  // mean per-character decision margin (diagnostic)
  int bursts_used{0};
  bool ok{false};
};

class SoftCombiner {
 public:
  // Combine up to 3 soft bursts into a best-estimate header (best-2-of-3).
  // hard_fallback: if soft combining fails a sanity check, the caller's hard
  // majority result (may be empty) is returned instead so we never regress.
  static SoftResult combine(const SoftBurst *bursts, int nbursts,
                            const std::string &hard_fallback);

 private:
  // Decode one burst's LLRs into (chars, per-char confidence, per-bit LLR ptr).
  struct DecodedBurst {
    std::string chars;                // hard chars from this burst alone
    std::vector<float> char_min_abs;  // min|l| per char (fragility)
    const std::vector<float> *llr{nullptr};
  };

  static char decide_char_(const float *bit_llr, float *out_min_abs);
  static void decode_burst_(const SoftBurst &b, DecodedBurst &out);

  // Split a hard char string into dash-delimited fields (indices into chars).
  static std::vector<std::pair<int, int>> field_spans_(const std::string &s);

  // Combine bit-LLRs across bursts for a given char index (with per-burst
  // char-offset alignment already resolved). Returns the decided char.
  static char combine_char_(const DecodedBurst *db, const int *char_index,
                            int nbursts, float *out_margin);

  // Lenient SAME grammar helpers (used for scoring/repair, not hard-blocking).
  static bool looks_like_org_(const std::string &f);
  static bool looks_like_event_(const std::string &f);
  static bool looks_like_fips_(const std::string &f);
};

}  // namespace same_decoder
}  // namespace esphome
