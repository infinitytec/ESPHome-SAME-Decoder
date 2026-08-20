# components/same_decoder/__init__.py
# Config schema + codegen for the SAME decoder.
# Redesign: preamble-locked fixed-phase sampler (primary) + automatic
# confidence-triggered closed-loop tracker (fallback). Timing is now locked on
# the 0xAB preamble ("set asynchronous decoder clocking cycles" per 47 CFR
# 11.31), NOT on the ZCZC letters. See the SAME/EAS protocol spec inline below.

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import automation
from esphome.components import sensor, text_sensor
from esphome.const import CONF_ID, CONF_SAMPLE_RATE, CONF_TRIGGER_ID

DEPENDENCIES = ["api"]
AUTO_LOAD = ["sensor", "text_sensor"]
CODEOWNERS = ["@infinitytec"]

CONF_DECODE_COUNT_SENSOR = "decode_count_sensor"
CONF_LAST_RAW_SENSOR = "last_raw_sensor"
CONF_ON_ALERT = "on_alert"
CONF_ON_SYNC = "on_sync"
CONF_ON_EOM = "on_eom"
CONF_ON_PREAMBLE = "on_preamble"
CONF_GAIN = "gain"
CONF_FREQ_OFFSET_HZ = "freq_offset_hz"
CONF_AGC_ENABLE = "agc_enable"
CONF_AGC_TARGET = "agc_target"
CONF_AGC_MIN_GAIN = "agc_min_gain"
CONF_AGC_MAX_GAIN = "agc_max_gain"
CONF_TIMEOUT_MS = "timeout_ms"
CONF_SINGLE_BURST_MIN_MS = "single_burst_min_ms"
CONF_POST_EMIT_DEAD_MS = "post_emit_dead_ms"

CONF_EOM_REQUIRE_CONTEXT = "eom_require_context"
CONF_EOM_CONTEXT_MS = "eom_context_ms"

CONF_DECODE_WATCHDOG_MS = "decode_watchdog_ms"

# resend_suppress_ms: cross-session re-emit window. An identical header arriving
#   in a NEW decode session is suppressed only if it comes within this many ms of
#   the previous emit; after the window elapses it re-emits. In-transmission
#   repeats (the 3 bursts of one message) are ALWAYS collapsed by the session key
#   and are unaffected by this setting. 0 = never suppress cross-session repeats.
CONF_RESEND_SUPPRESS_MS = "resend_suppress_ms"

# --- Preamble-lock (redesign) parameters ---
# preamble_lock_bits: the MINIMUM number of consecutive good preamble bit-periods
#   required to declare timing lock. This is a floor for reliable convergence,
#   NOT a requirement to receive the full 16-byte (128-bit) preamble. A partial
#   but sufficient run locks the clock; extra preamble bytes just keep the loop
#   warm until ZCZC arrives. Default 32 bits (~4 bytes) is comfortably below the
#   full preamble length and locks robustly on clean and typical off-air audio.
CONF_PREAMBLE_LOCK_BITS = "preamble_lock_bits"
# preamble_energy_mult: tone-energy multiple over the adaptive noise floor that
#   qualifies a sample window as "tone present" for the preamble correlator.
CONF_PREAMBLE_ENERGY_MULT = "preamble_energy_mult"
# preamble_balance_max: max |mark-space|/(mark+space) imbalance still accepted as
#   a valid alternating-preamble bit-period (rejects single-tone noise).
CONF_PREAMBLE_BALANCE_MAX = "preamble_balance_max"
# lock_confidence_min: mean per-bit decision margin (|LLR|) over the qualifying
#   preamble run required to accept the lock. Guards against locking on noise.
CONF_LOCK_CONFIDENCE_MIN = "lock_confidence_min"
# residual_drift_ppm: bound on the slow residual clock trim applied by the
#   fixed-phase sampler after lock (parts-per-million of the bit period).
CONF_RESIDUAL_DRIFT_PPM = "residual_drift_ppm"
# fallback_conf_thresh: per-symbol decision confidence (|LLR|) below which the
#   automatic closed-loop (Gardner/early-late) tracker engages, seeded by the
#   preamble-locked phase. Above it, the deterministic fixed-phase path runs.
CONF_FALLBACK_CONF_THRESH = "fallback_conf_thresh"
# fallback_kp / fallback_ki: PI gains for the fallback closed-loop tracker.
CONF_FALLBACK_KP = "fallback_kp"
CONF_FALLBACK_KI = "fallback_ki"

same_decoder_ns = cg.esphome_ns.namespace("same_decoder")
SAMEDecoder = same_decoder_ns.class_("SAMEDecoder", cg.Component)

AlertTrigger = same_decoder_ns.class_("AlertTrigger", automation.Trigger.template())
SyncTrigger = same_decoder_ns.class_("SyncTrigger", automation.Trigger.template())
EomTrigger = same_decoder_ns.class_("EomTrigger", automation.Trigger.template())
PreambleTrigger = same_decoder_ns.class_("PreambleTrigger", automation.Trigger.template())

# Keys removed across redesigns. Old configs fail loudly (never silently
# misbehave). The ZCZC-acquisition knobs are removed here because timing is now
# locked on the preamble, not acquired from the ZCZC letters.
_REMOVED_KEYS = {
    # Pre-redesign (already removed previously).
    "preamble_lock": "Timing now locks on the 0xAB preamble by design; there is no separate lock flag.",
    "preamble_min_density": "The bit-density preamble detector was removed.",
    "preamble_min_bits": "Replaced by 'preamble_lock_bits' (minimum consecutive good preamble bit-periods to lock).",
    "preamble_acq_gain": "PLL fast-acquire was removed (it corrupted decodes).",
    "preamble_lock_timeout_ms": "The preamble-lock watchdog is no longer needed.",
    # ZCZC-acquisition knobs removed in THIS redesign.
    "preamble_status": "The diagnostic-only preamble energy gate was replaced by a real preamble timing lock.",
    "ab_required": "AB preamble presence is now intrinsic to timing lock; a separate requirement flag is obsolete.",
    "pre_td_thresh": "ZCZC-acquisition transition-density gate removed; timing now locks on the preamble.",
    "pre_abd5_ham_thresh": "ZCZC-acquisition AB/D5 hamming gate removed; timing now locks on the preamble.",
    "tr_kp_acq_mult": "ZCZC-acquisition timing-loop multiplier removed; use 'fallback_kp' for the fallback tracker.",
    "tr_ki_acq_mult": "ZCZC-acquisition timing-loop multiplier removed; use 'fallback_ki' for the fallback tracker.",
    "kpki_slew_symbols": "Bumpless gearshift slew removed; the fixed-phase sampler locks directly on the preamble.",
    "tr_woff_clamp_acq_factor": "ZCZC-acquisition integrator clamp factor removed.",
    "zczc_ham_relaxed": "Relaxed ZCZC hamming (preamble-recent) removed; ZCZC is now sampled at preamble-locked timing.",
    "preamble_recent_ms": "The 'preamble recent' relaxation window removed; timing lock is explicit now.",
    "single_burst_min_ms_legacy": "Renamed; use 'single_burst_min_ms'.",
}


def _reject_removed_keys(config):
    for key, why in _REMOVED_KEYS.items():
        if key in config:
            raise cv.Invalid(
                f"'{key}' has been removed in the preamble-lock redesign. {why} "
                f"Please delete it from your 'same_decoder:' block."
            )
    return config


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(SAMEDecoder),
            cv.Optional(CONF_SAMPLE_RATE, default=48000): cv.int_range(min=8000, max=96000),
            cv.Optional(CONF_GAIN, default=1.0): cv.float_range(min=0.1, max=1000.0),
            cv.Optional(CONF_FREQ_OFFSET_HZ, default=0.0): cv.float_range(min=-200.0, max=200.0),
            cv.Optional(CONF_AGC_ENABLE, default=False): cv.boolean,
            cv.Optional(CONF_AGC_TARGET, default=8000.0): cv.float_range(min=1000.0, max=20000.0),
            cv.Optional(CONF_AGC_MIN_GAIN, default=0.25): cv.float_range(min=0.05, max=10.0),
            cv.Optional(CONF_AGC_MAX_GAIN, default=32.0): cv.float_range(min=1.0, max=1000.0),
            cv.Optional(CONF_TIMEOUT_MS, default=3000): cv.int_range(min=500, max=30000),
            cv.Optional(CONF_SINGLE_BURST_MIN_MS, default=7000): cv.int_range(min=1000, max=60000),
            cv.Optional(CONF_POST_EMIT_DEAD_MS, default=800): cv.int_range(min=0, max=10000),
            # Cross-session re-emit window (0 .. 15 min). 0 = never suppress
            # cross-session repeats. In-transmission repeats are always collapsed.
            cv.Optional(CONF_RESEND_SUPPRESS_MS, default=3000): cv.int_range(min=0, max=900000),
            # --- Preamble-lock parameters ---
            cv.Optional(CONF_PREAMBLE_LOCK_BITS, default=32): cv.int_range(min=8, max=128),
            cv.Optional(CONF_PREAMBLE_ENERGY_MULT, default=8.0): cv.float_range(min=2.0, max=50.0),
            cv.Optional(CONF_PREAMBLE_BALANCE_MAX, default=0.40): cv.float_range(min=0.05, max=0.9),
            cv.Optional(CONF_LOCK_CONFIDENCE_MIN, default=0.5): cv.float_range(min=0.0, max=50.0),
            cv.Optional(CONF_RESIDUAL_DRIFT_PPM, default=2000.0): cv.float_range(min=0.0, max=50000.0),
            cv.Optional(CONF_FALLBACK_CONF_THRESH, default=0.20): cv.float_range(min=0.0, max=50.0),
            cv.Optional(CONF_FALLBACK_KP, default=0.06): cv.float_range(min=0.0, max=1.0),
            cv.Optional(CONF_FALLBACK_KI, default=0.0015): cv.float_range(min=0.0, max=0.5),
            cv.Optional(CONF_EOM_REQUIRE_CONTEXT, default=True): cv.boolean,
            cv.Optional(CONF_EOM_CONTEXT_MS, default=120000): cv.int_range(min=1000, max=600000),
            cv.Optional(CONF_DECODE_WATCHDOG_MS, default=10000): cv.int_range(min=2000, max=60000),
            cv.Optional(CONF_DECODE_COUNT_SENSOR): cv.use_id(sensor.Sensor),
            cv.Optional(CONF_LAST_RAW_SENSOR): cv.use_id(text_sensor.TextSensor),
            cv.Optional(CONF_ON_ALERT): automation.validate_automation(
                {cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(AlertTrigger)}
            ),
            cv.Optional(CONF_ON_SYNC): automation.validate_automation(
                {cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(SyncTrigger)}
            ),
            cv.Optional(CONF_ON_EOM): automation.validate_automation(
                {cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(EomTrigger)}
            ),
            cv.Optional(CONF_ON_PREAMBLE): automation.validate_automation(
                {cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(PreambleTrigger)}
            ),
        }
    ).extend(cv.COMPONENT_SCHEMA),
    _reject_removed_keys,
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    cg.add(var.set_sample_rate(config[CONF_SAMPLE_RATE]))
    cg.add(var.set_gain(config[CONF_GAIN]))
    cg.add(var.set_freq_offset_hz(config[CONF_FREQ_OFFSET_HZ]))
    cg.add(var.set_agc_enable(config[CONF_AGC_ENABLE]))
    cg.add(var.set_agc_target(config[CONF_AGC_TARGET]))
    cg.add(var.set_agc_min_gain(config[CONF_AGC_MIN_GAIN]))
    cg.add(var.set_agc_max_gain(config[CONF_AGC_MAX_GAIN]))
    cg.add(var.set_timeout_ms(config[CONF_TIMEOUT_MS]))
    cg.add(var.set_single_burst_min_ms(config[CONF_SINGLE_BURST_MIN_MS]))
    cg.add(var.set_post_emit_dead_ms(config[CONF_POST_EMIT_DEAD_MS]))
    cg.add(var.set_resend_suppress_ms(config[CONF_RESEND_SUPPRESS_MS]))

    cg.add(var.set_preamble_lock_bits(config[CONF_PREAMBLE_LOCK_BITS]))
    cg.add(var.set_preamble_energy_mult(config[CONF_PREAMBLE_ENERGY_MULT]))
    cg.add(var.set_preamble_balance_max(config[CONF_PREAMBLE_BALANCE_MAX]))
    cg.add(var.set_lock_confidence_min(config[CONF_LOCK_CONFIDENCE_MIN]))
    cg.add(var.set_residual_drift_ppm(config[CONF_RESIDUAL_DRIFT_PPM]))
    cg.add(var.set_fallback_conf_thresh(config[CONF_FALLBACK_CONF_THRESH]))
    cg.add(var.set_fallback_kp(config[CONF_FALLBACK_KP]))
    cg.add(var.set_fallback_ki(config[CONF_FALLBACK_KI]))

    cg.add(var.set_eom_require_context(config[CONF_EOM_REQUIRE_CONTEXT]))
    cg.add(var.set_eom_context_ms(config[CONF_EOM_CONTEXT_MS]))
    cg.add(var.set_decode_watchdog_ms(config[CONF_DECODE_WATCHDOG_MS]))

    if CONF_DECODE_COUNT_SENSOR in config:
        s = await cg.get_variable(config[CONF_DECODE_COUNT_SENSOR])
        cg.add(var.set_decode_count_sensor(s))
    if CONF_LAST_RAW_SENSOR in config:
        ts = await cg.get_variable(config[CONF_LAST_RAW_SENSOR])
        cg.add(var.set_last_raw_sensor(ts))

    for conf in config.get(CONF_ON_ALERT, []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID])
        await automation.build_automation(trigger, [], conf)
        cg.add(var.register_alert_trigger(trigger))

    for conf in config.get(CONF_ON_SYNC, []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID])
        await automation.build_automation(trigger, [], conf)
        cg.add(var.register_sync_trigger(trigger))

    for conf in config.get(CONF_ON_EOM, []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID])
        await automation.build_automation(trigger, [], conf)
        cg.add(var.register_eom_trigger(trigger))

    for conf in config.get(CONF_ON_PREAMBLE, []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID])
        await automation.build_automation(trigger, [], conf)
        cg.add(var.register_preamble_trigger(trigger))
