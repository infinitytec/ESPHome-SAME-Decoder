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

class So
