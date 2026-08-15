# components/same_decoder/__init__.py
# Config schema + codegen for the SAME decoder (soft-decision + commercial-style TR).

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
CONF_GAIN = "gain"
CONF_FREQ_OFFSET_HZ = "freq_offset_hz"
CONF_AGC_ENABLE = "agc_enable"
CONF_AGC_TARGET = "agc_target"
CONF_AGC_MIN_GAIN = "agc_min_gain"
CONF_AGC_MAX_GAIN = "agc_max_gain"
CONF_TIMEOUT_MS = "timeout_ms"
CONF_SINGLE_BURST_MIN_MS = "single_burst_min_ms"
CONF_POST_EMIT_DEAD_MS = "post_emit_dead_ms"
CONF_AB_REQUIRED = "ab_required"

CONF_PREAMBLE_STATUS = "preamble_status"
CONF_PREAMBLE_ENERGY_MULT = "preamble_energy_mult"

CONF_EOM_REQUIRE_CONTEXT = "eom_require_context"
CONF_EOM_CONTEXT_MS = "eom_context_ms"

CONF_DECODE_WATCHDOG_MS = "decode_watchdog_ms"

# --- Option 3: continuous preamble-assisted acquisition ---
CONF_PRE_TD_THRESH = "pre_td_thresh"
CONF_PRE_ABD5_HAM_THRESH = "pre_abd5_ham_thresh"
CONF_TR_KP_ACQ_MULT = "tr_kp_acq_mult"
CONF_TR_KI_ACQ_MULT = "tr_ki_acq_mult"
CONF_KPKI_SLEW_SYMBOLS = "kpki_slew_symbols"
CONF_TR_WOFF_CLAMP_ACQ_FACTOR = "tr_woff_clamp_acq_factor"
CONF_ZCZC_HAM_RELAXED = "zczc_ham_relaxed"
CONF_PREAMBLE_RECENT_MS = "preamble_recent_ms"

same_decoder_ns = cg.esphome_ns.namespace("same_decoder")
SAMEDecoder = same_decoder_ns.class_("SAMEDecoder", cg.Component)

AlertTrigger = same_decoder_ns.class_("AlertTrigger", automation.Trigger.template())
SyncTrigger = same_decoder_ns.class_("SyncTrigger", automation.Trigger.template())
EomTrigger = same_decoder_ns.class_("EomTrigger", automation.Trigger.template())

_REMOVED_KEYS = {
    "preamble_lock": "Preamble no longer resets the clock; acquisition is via ZCZC tiers.",
    "preamble_min_density": "The bit-density preamble detector was removed.",
    "preamble_min_bits": "The bit-density preamble detector was removed.",
    "preamble_acq_gain": "PLL fast-acquire was removed (it corrupted decodes).",
    "preamble_lock_timeout_ms": "The preamble-lock watchdog is no longer needed.",
}


def _reject_removed_keys(config):
    for key, why in _REMOVED_KEYS.items():
        if key in config:
            raise cv.Invalid(
                f"'{key}' has been removed in the redesign. {why} "
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
            cv.Optional(CONF_AB_REQUIRED, default=False): cv.boolean,
            cv.Optional(CONF_PREAMBLE_STATUS, default=True): cv.boolean,
            cv.Optional(CONF_PREAMBLE_ENERGY_MULT, default=8.0): cv.float_range(min=2.0, max=50.0),
            cv.Optional(CONF_PRE_TD_THRESH, default=0.70): cv.float_range(min=0.5, max=1.0),
            cv.Optional(CONF_PRE_ABD5_HAM_THRESH, default=2): cv.int_range(min=0, max=6),
            cv.Optional(CONF_TR_KP_ACQ_MULT, default=2.0): cv.float_range(min=1.0, max=8.0),
            cv.Optional(CONF_TR_KI_ACQ_MULT, default=1.5): cv.float_range(min=1.0, max=8.0),
            cv.Optional(CONF_KPKI_SLEW_SYMBOLS, default=16): cv.int_range(min=1, max=128),
            cv.Optional(CONF_TR_WOFF_CLAMP_ACQ_FACTOR, default=0.6): cv.float_range(min=0.1, max=1.0),
            cv.Optional(CONF_ZCZC_HAM_RELAXED, default=2): cv.int_range(min=1, max=4),
            cv.Optional(CONF_PREAMBLE_RECENT_MS, default=200): cv.int_range(min=0, max=5000),
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
    cg.add(var.set_ab_required(config[CONF_AB_REQUIRED]))
    cg.add(var.set_preamble_status(config[CONF_PREAMBLE_STATUS]))
    cg.add(var.set_preamble_energy_mult(config[CONF_PREAMBLE_ENERGY_MULT]))
    cg.add(var.set_pre_td_thresh(config[CONF_PRE_TD_THRESH]))
    cg.add(var.set_pre_abd5_ham_thresh(config[CONF_PRE_ABD5_HAM_THRESH]))
    cg.add(var.set_tr_kp_acq_mult(config[CONF_TR_KP_ACQ_MULT]))
    cg.add(var.set_tr_ki_acq_mult(config[CONF_TR_KI_ACQ_MULT]))
    cg.add(var.set_kpki_slew_symbols(config[CONF_KPKI_SLEW_SYMBOLS]))
    cg.add(var.set_tr_woff_clamp_acq_factor(config[CONF_TR_WOFF_CLAMP_ACQ_FACTOR]))
    cg.add(var.set_zczc_ham_relaxed(config[CONF_ZCZC_HAM_RELAXED]))
    cg.add(var.set_preamble_recent_ms(config[CONF_PREAMBLE_RECENT_MS]))
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
