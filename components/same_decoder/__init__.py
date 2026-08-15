# components/same_decoder/__init__.py
# Config schema + codegen for the SAME decoder.
# Audio is delivered from YAML via microphone: on_data -> feed_bytes(), so this
# schema does NOT own the microphone/I2S.
#
# Reliability extensions:
#   - dynamic Goertzel coeffs from sample_rate + optional freq_offset_hz
#   - soft AGC (optional, default off)
#   - T0 tone-based preamble lock (PRIMARY acquisition, enabled by default)
#       * phase-independent bit-transition-density detector
#       * PLL reset + fast-acquire during preamble, then normal tracking
#       * watchdog releases a stuck lock (tunable preamble_lock_timeout_ms)
#   - immediate emit on a single structurally-valid burst
#   - optional AB preamble correlator (default off - preserves original sensitivity)
#   - configurable burst timeouts + post-emit dead-time
#   - stronger header validation (structural + light semantic)
#   - NNNN EOM detection (never emitted as an alert; fires on_eom)

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

# ---- T0 preamble-lock (PRIMARY acquisition, enabled by default) ----
CONF_PREAMBLE_LOCK = "preamble_lock"
CONF_PREAMBLE_MIN_DENSITY = "preamble_min_density"
CONF_PREAMBLE_MIN_BITS = "preamble_min_bits"
CONF_PREAMBLE_ACQ_GAIN = "preamble_acq_gain"
CONF_PREAMBLE_LOCK_TIMEOUT_MS = "preamble_lock_timeout_ms"

same_decoder_ns = cg.esphome_ns.namespace("same_decoder")
SAMEDecoder = same_decoder_ns.class_("SAMEDecoder", cg.Component)

AlertTrigger = same_decoder_ns.class_("AlertTrigger", automation.Trigger.template())
SyncTrigger = same_decoder_ns.class_("SyncTrigger", automation.Trigger.template())
EomTrigger = same_decoder_ns.class_("EomTrigger", automation.Trigger.template())

CONFIG_SCHEMA = cv.Schema(
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
        # ---- T0 preamble lock ----
        cv.Optional(CONF_PREAMBLE_LOCK, default=True): cv.boolean,
        cv.Optional(CONF_PREAMBLE_MIN_DENSITY, default=0.75): cv.float_range(min=0.5, max=1.0),
        cv.Optional(CONF_PREAMBLE_MIN_BITS, default=32): cv.int_range(min=8, max=256),
        cv.Optional(CONF_PREAMBLE_ACQ_GAIN, default=4.0): cv.float_range(min=1.0, max=20.0),
        cv.Optional(CONF_PREAMBLE_LOCK_TIMEOUT_MS, default=1500): cv.int_range(min=200, max=10000),
        cv.Optional(CONF_DECODE_COUNT_SENSOR): cv.use_id(sensor.Sensor),
        cv.Optional(CONF_LAST_RAW_SENSOR): cv.use_id(text_sensor.TextSensor),
        cv.Optional(CONF_ON_ALERT): automation.validate_automation(
            {
                cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(AlertTrigger),
            }
        ),
        cv.Optional(CONF_ON_SYNC): automation.validate_automation(
            {
                cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(SyncTrigger),
            }
        ),
        cv.Optional(CONF_ON_EOM): automation.validate_automation(
            {
                cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(EomTrigger),
            }
        ),
    }
).extend(cv.COMPONENT_SCHEMA)


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

    # ---- T0 preamble lock ----
    cg.add(var.set_preamble_lock(config[CONF_PREAMBLE_LOCK]))
    cg.add(var.set_preamble_min_density(config[CONF_PREAMBLE_MIN_DENSITY]))
    cg.add(var.set_preamble_min_bits(config[CONF_PREAMBLE_MIN_BITS]))
    cg.add(var.set_preamble_acq_gain(config[CONF_PREAMBLE_ACQ_GAIN]))
    cg.add(var.set_preamble_lock_timeout_ms(config[CONF_PREAMBLE_LOCK_TIMEOUT_MS]))

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
