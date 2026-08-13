// same_decoder/same_event_codes.h
// Complete NWS/EAS SAME event code table (all operational codes per weather.gov
// and the FCC Part 11 list, including the national/PEP activation tier).
// Severity tiers follow the FCC W/A/E/S convention; TOR/SVR/EVI are hard-coded
// exceptions. Adjust tiers to taste - they're the default suggestions.
#pragma once
#include <string>
#include <map>

namespace esphome {
namespace same_decoder {

struct EventInfo {
  const char *name;       // human-readable
  const char *severity;   // Minor | Moderate | Severe | Extreme
  bool enabled_default;   // whether this code passes by default
};

// Key = 3-letter SAME event code.
static const std::map<std::string, EventInfo> SAME_EVENT_CODES = {
  // ---- National / Presidential (PEP) ----
  {"EAN", {"Emergency Action Notification",  "Extreme",  true }},
  {"EAT", {"Emergency Action Termination",   "Severe",   true }},
  {"NIC", {"National Information Center",     "Severe",   true }},
  {"NPT", {"National Periodic Test",         "Minor",    false }},
  {"NAT", {"National Audible Test",          "Minor",    false }},
  {"NST", {"National Silent Test",           "Minor",    false }},

  // ---- Weather-Related ----
  {"BZW", {"Blizzard Warning",              "Severe",   true }},
  {"CFA", {"Coastal Flood Watch",           "Moderate", true }},
  {"CFW", {"Coastal Flood Warning",         "Severe",   true }},
  {"DSW", {"Dust Storm Warning",            "Severe",   true }},
  {"EWW", {"Extreme Wind Warning",          "Extreme",  true }},
  {"FFA", {"Flash Flood Watch",             "Moderate", true }},
  {"FFW", {"Flash Flood Warning",           "Severe",   true }},
  {"FFS", {"Flash Flood Statement",         "Minor",    true }},
  {"FLA", {"Flood Watch",                   "Moderate", true }},
  {"FLW", {"Flood Warning",                 "Severe",   true }},
  {"FLS", {"Flood Statement",               "Minor",    true }},
  {"FZW", {"Freeze Warning",                "Moderate", true }},
  {"HWA", {"High Wind Watch",               "Moderate", true }},
  {"HWW", {"High Wind Warning",             "Severe",   true }},
  {"HUA", {"Hurricane Watch",               "Severe",   true }},
  {"HUW", {"Hurricane Warning",             "Extreme",  true }},
  {"HLS", {"Hurricane Statement",           "Minor",    true }},
  {"HWO", {"Severe Weather Statement (Outlook)", "Minor", true }},
  {"MWS", {"Marine Weather Statement",      "Minor",    true }},
  {"SVA", {"Severe Thunderstorm Watch",     "Moderate", true }},
  {"SVR", {"Severe Thunderstorm Warning",   "Severe",   true }},  // FCC exception
  {"SVS", {"Severe Weather Statement",      "Minor",    true }},
  {"SQW", {"Snow Squall Warning",           "Severe",   true }},
  {"SMW", {"Special Marine Warning",        "Moderate", true }},
  {"SPS", {"Special Weather Statement",     "Minor",    true }},
  {"SSA", {"Storm Surge Watch",             "Severe",   true }},
  {"SSW", {"Storm Surge Warning",           "Extreme",  true }},
  {"TOA", {"Tornado Watch",                 "Moderate", true }},
  {"TOR", {"Tornado Warning",               "Extreme",  true }},  // FCC exception
  {"TRA", {"Tropical Storm Watch",          "Moderate", true }},
  {"TRW", {"Tropical Storm Warning",        "Severe",   true }},
  {"TSA", {"Tsunami Watch",                 "Severe",   true }},
  {"TSW", {"Tsunami Warning",               "Extreme",  true }},
  {"WSA", {"Winter Storm Watch",            "Moderate", true }},
  {"WSW", {"Winter Storm Warning",          "Severe",   true }},

  // ---- Non-Weather-Related (state/local optional) ----
  {"AVA", {"Avalanche Watch",               "Moderate", true }},
  {"AVW", {"Avalanche Warning",             "Severe",   true }},
  {"BLU", {"Blue Alert",                    "Severe",   true }},
  {"CAE", {"Child Abduction Emergency",     "Severe",   true }},
  {"CDW", {"Civil Danger Warning",          "Extreme",  true }},
  {"CEM", {"Civil Emergency Message",       "Severe",   true }},
  {"EQW", {"Earthquake Warning",            "Extreme",  true }},
  {"EVI", {"Evacuation Immediate",          "Extreme",  true }},  // FCC exception
  {"FRW", {"Fire Warning",                  "Severe",   true }},
  {"HMW", {"Hazardous Materials Warning",   "Severe",   true }},
  {"LEW", {"Law Enforcement Warning",       "Severe",   true }},
  {"LAE", {"Local Area Emergency",          "Moderate", true }},
  {"TOE", {"911 Telephone Outage Emergency","Moderate", true }},
  {"NMN", {"Network Message Notification",  "Minor",    true }},
  {"NUW", {"Nuclear Power Plant Warning",   "Extreme",  true }},
  {"RHW", {"Radiological Hazard Warning",   "Extreme",  true }},
  {"SPW", {"Shelter in Place Warning",      "Severe",   true }},
  {"VOW", {"Volcano Warning",               "Severe",   true }},

  // ---- Administrative (tests default OFF where applicable) ----
  {"ADR", {"Administrative Message",        "Minor",    true  }},
  {"DMO", {"Practice/Demo Warning",         "Minor",    false }},
  {"RMT", {"Required Monthly Test",         "Minor",    false }},
  {"RWT", {"Required Weekly Test",          "Minor",    false }},
};

}  // namespace same_decoder
}  // namespace esphome
