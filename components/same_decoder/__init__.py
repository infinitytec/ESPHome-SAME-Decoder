# components/same_decoder/__init__.py
# Config schema + codegen for the SAME decoder.
# Audio is delivered from YAML via microphone: on_data -> feed_bytes(), so this
# schema does NOT own the microphone/I2S. Adds an optional software `gain`
# applied inside feed_bytes() (the source-level gain_factor is not reachable
# through the on_data bridge, so we apply our own — and allow >64 since the
# incoming line-in level can be very low).
#
# Reliability extensions (v2):
#   - dynamic Goertzel coeffs from sample_rate + optional freq_offset_hz
#   - soft AGC (optional)
#   - AB preamble correlator (always on)
#   - configurable burst timeouts + post-emit dead-time
#   - stronger header validation
#   - NNNN EOM detection

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
CONF_GAIN = "gain"
CONF_FREQ_OFFSET_HZ = "freq_offset_hz"
CONF_AGC_ENABLE = "agc_enable"
CONF_AGC_TARGET = "agc_target"
CONF_AGC_MIN_GAIN = "agc_min_gain"
CONF_AGC_MAX_GAIN = "agc_max_gain"
CONF_TIMEOUT_MS = "timeout_ms"
CONF_SINGLE_BURST_MIN_MS = "single_burst_min_ms"
CONF_POST_EMIT_DEAD_MS = "post_emit_dead_ms"

same_decoder_ns = cg.esphome_ns.namespace("same_decoder")
SAMEDecoder = same_decoder_ns.class_("SAMEDecoder", cg.Component)

AlertTrigger = same_decoder_ns.class_("AlertTrigger", automation.Trigger.template())
SyncTrigger = same_decoder_ns.class_("SyncTrigger", automation.Trigger.template())

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(SAMEDecoder),
        cv.Optional(CONF_SAMPLE_RATE, default=48000): cv.int_range(min=8000, max=96000),
        cv.Optional(CONF_GAIN, default=1.0): cv.float_range(min=0.1, max=1000.0),
        cv.Optional(CONF_FREQ_OFFSET_HZ, default=0.0): cv.float_range(min=-200.0, max=200.0),
        cv.Optional(CONF_AGC_ENABLE, default=True): cv.boolean,
        cv.Optional(CONF_AGC_TARGET, default=8000.0): cv.float_range(min=1000.0, max=20000.0),
        cv.Optional(CONF_AGC_MIN_GAIN, default=0.25): cv.float_range(min=0.05, max=10.0),
        cv.Optional(CONF_AGC_MAX_GAIN, default=64.0): cv.float_range(min=1.0, max=1000.0),
        cv.Optional(CONF_TIMEOUT_MS, default=3000): cv.int_range(min=500, max=30000),
        cv.Optional(CONF_SINGLE_BURST_MIN_MS, default=7000): cv.int_range(min=1000, max=60000),
        cv.Optional(CONF_POST_EMIT_DEAD_MS, default=1500): cv.int_range(min=0, max=10000),
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
