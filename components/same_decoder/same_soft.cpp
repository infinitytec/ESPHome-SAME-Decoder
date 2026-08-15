// components/same_decoder/same_soft.cpp
#include "same_soft.h"
#include "esphome/core/log.h"

#include <algorithm>
#include <cmath>
#include <cctype>

namespace esphome {
namespace same_decoder {

static const char *const TAG = "same_soft";

// Decide one 8-bit LSB-first character from 8 per-bit LLRs.
// LLR sign convention: l > 0 => bit '1' (mark), l < 0 => bit '0' (space).
char SoftCombiner::decide_char_(const float *bit_llr, float *out_min_abs) {
  uint8_t byte = 0;
  float min_abs = 1e30f;
  for (int k = 0; k < 8; k++) {
    float l = bit_llr[k];
    if (l > 0.0f) byte |= (1u << k);   // LSB-first
    float a = std::fabs(l);
    if (a < min_abs) min_abs = a;
  }
  if (out_min_abs) *out_min_abs = min_abs;
  return (char) byte;
}

void SoftCombiner::decode_burst_(const SoftBurst &b, DecodedBurst &out) {
  out.chars.clear();
  out.char_min_abs.clear();
  out.llr = &b.llr;
  size_t nchars = b.nchars();
  out.chars.reserve(nchars);
  out.char_min_abs.reserve(nchars);
  for (size_t i = 0; i < nchars; i++) {
    float min_abs = 0.0f;
    char c = decide_char_(&b.llr[i * 8], &min_abs);
    out.chars += c;
    out.char_min_abs.push_back(min_abs);
  }
}

std::vector<std::pair<int, int>> SoftCombiner::field_spans_(const std::string &s) {
  std::vector<std::pair<int, int>> spans;
  int start = 0;
  for (int i = 0; i < (int) s.size(); i++) {
    if (s[i] == '-') {
      spans.emplace_back(start, i);
      start = i + 1;
    }
  }
  if (start < (int) s.size())
    spans.emplace_back(start, (int) s.size());
  return spans;
}

bool SoftCombiner::looks_like_org_(const std::string &f) {
  return f == "EAS" || f == "CIV" || f == "WXR" || f == "PEP";
}

bool SoftCombiner::looks_like_event_(const std::string &f) {
  if (f.size() != 3) return false;
  for (char c : f) if (!std::isupper((unsigned char) c)) return false;
  return true;
}

bool SoftCombiner::looks_like_fips_(const std::string &f) {
  if (f.size() != 6) return false;
  for (char c : f) if (!std::isdigit((unsigned char) c)) return false;
  return true;
}

char SoftCombiner::combine_char_(const DecodedBurst *db, const int *char_index,
                                 int nbursts, float *out_margin) {
  float combined[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  for (int b = 0; b < nbursts; b++) {
    int ci = char_index[b];
    if (ci < 0) continue;
    const std::vector<float> *llr = db[b].llr;
    size_t base = (size_t) ci * 8;
    if (base + 8 > llr->size()) continue;
    for (int k = 0; k < 8; k++)
      combined[k] += (*llr)[base + k];   // additive LLR combine
  }
  float min_abs = 0.0f;
  char c = decide_char_(combined, &min_abs);
  if (out_margin) *out_margin = min_abs;
  return c;
}

SoftResult SoftCombiner::combine(const SoftBurst *bursts, int nbursts,
                                 const std::string &hard_fallback) {
  SoftResult res;

  DecodedBurst db[3];
  int used = 0;
  for (int i = 0; i < nbursts && i < 3; i++) {
    if (bursts[i].nbits() >= 8 * 4) {   // at least "ZCZC"
      decode_burst_(bursts[i], db[used]);
      used++;
    }
  }

  if (used == 0) {
    res.header = hard_fallback;
    res.ok = !hard_fallback.empty();
    return res;
  }

  // FIX 2: choose the reference burst by QUALITY, not length. The reference is
  // the burst with the highest mean per-character confidence (mean min|l|),
  // among those that actually start with a plausible "ZCZC" front. A clean
  // burst must anchor the alignment, never a long-but-garbage one.
  int ref = -1;
  float best_quality = -1.0f;
  for (int b = 0; b < used; b++) {
    // Front sanity: require the first 4 chars to be "ZCZC" (soft-decided).
    if (db[b].chars.size() < 4) continue;
    if (db[b].chars.compare(0, 4, "ZCZC") != 0) continue;
    // Mean confidence over the burst.
    double sum = 0.0;
    for (float m : db[b].char_min_abs) sum += m;
    float q = db[b].char_min_abs.empty() ? 0.0f
                : (float) (sum / db[b].char_min_abs.size());
    if (q > best_quality) { best_quality = q; ref = b; }
  }

  // If no burst has a clean ZCZC front, fall back to hard result immediately.
  if (ref < 0) {
    ESP_LOGD(TAG, "Soft combine: no burst with clean ZCZC front; using hard fallback.");
    res.header = hard_fallback;
    res.ok = !hard_fallback.empty();
    return res;
  }

  const std::string &refc = db[ref].chars;
  auto ref_fields = field_spans_(refc);

  std::vector<std::vector<std::pair<int, int>>> burst_fields(used);
  for (int b = 0; b < used; b++)
    burst_fields[b] = field_spans_(db[b].chars);

  std::string out;
  out.reserve(refc.size());
  double margin_sum = 0.0;
  int margin_n = 0;

  for (size_t fi = 0; fi < ref_fields.size(); fi++) {
    int rstart = ref_fields[fi].first;
    int rend = ref_fields[fi].second;

    // Per-burst base offset for this field (dash-anchored slip bounding). A
    // burst only contributes to this field if it HAS a corresponding field of
    // similar length; otherwise it's skipped for this field (contributes -1),
    // which prevents a mis-length burst from shifting the combine.
    int base_off[3] = {0, 0, 0};
    bool contributes[3] = {false, false, false};
    for (int b = 0; b < used; b++) {
      if (b == ref) { base_off[b] = 0; contributes[b] = true; continue; }
      if (fi < burst_fields[b].size()) {
        int blen = burst_fields[b][fi].second - burst_fields[b][fi].first;
        int rlen = rend - rstart;
        // Only align if this burst's field length is within +/-1 of reference.
        if (std::abs(blen - rlen) <= 1) {
          base_off[b] = burst_fields[b][fi].first - rstart;
          contributes[b] = true;
        }
      }
    }

    for (int rp = rstart; rp < rend; rp++) {
      int char_index[3];
      for (int b = 0; b < used; b++) {
        if (!contributes[b]) { char_index[b] = -1; continue; }
        int idx = (b == ref) ? rp : (rp + base_off[b]);
        if (idx < 0 || idx >= (int) db[b].chars.size())
          idx = -1;
        char_index[b] = idx;
      }
      float margin = 0.0f;
      char c = combine_char_(db, char_index, used, &margin);
      out += c;
      margin_sum += margin;
      margin_n++;
    }

    if (fi + 1 < ref_fields.size())
      out += '-';
  }

  res.header = out;
  res.mean_margin = (margin_n > 0) ? (float) (margin_sum / margin_n) : 0.0f;
  res.bursts_used = used;

  // FIX 3: mandatory ZCZC anchoring + strict sanity. The combined output MUST
  // start with "ZCZC" and contain a '+'. If not, the alignment shifted the
  // front (which must never happen) -> fall back to hard majority.
  bool soft_ok = (out.rfind("ZCZC", 0) == 0) && (out.find('+') != std::string::npos);
  if (!soft_ok) {
    ESP_LOGD(TAG, "Soft combine failed sanity ('%s'); using hard fallback.",
             out.c_str());
    res.header = hard_fallback;
    res.ok = !hard_fallback.empty();
    return res;
  }

  ESP_LOGD(TAG, "Soft combine ok (bursts=%d, ref=%d, mean_margin=%.2f): '%s'",
           used, ref, res.mean_margin, out.c_str());
  res.ok = true;
  return res;
}

}  // namespace same_decoder
}  // namespace esphome
