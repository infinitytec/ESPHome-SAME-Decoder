# components/same_decoder/__init__.py
# Config schema + codegen for the SAME decoder.
# Audio is delivered from YAML via microphone: on_data -> feed_bytes(), so this
# schema does NOT own the microphone/I2S. Adds optional software `gain`,
# burst-group timeout, and diagnostic sensors (quality / SNR / frequency offset).

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
CONF_QUALITY_SENSOR = "quality_sensor"
CONF_SNR_SENSOR = "snr_sensor"
CONF_FREQ_OFFSET_SENSOR = "freq_offset_sensor"
CONF_ON_ALERT = "on_alert"
CONF_GAIN = "gain"
CONF_BURST_TIMEOUT = "burst_timeout"

same_decoder_ns = cg.esphome_ns.namespace("same_decoder")
SAMEDecoder = same_decoder_ns.class_("SAMEDecoder", cg.Component)

AlertTrigger = same_decoder_ns.class_("AlertTrigger", automation.Trigger.template())

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(SAMEDecoder),
        cv.Optional(CONF_SAMPLE_RATE, default=48000): cv.int_range(min=8000, max=96000),
        cv.Optional(CONF_GAIN, default=1.0): cv.float_range(min=1.0, max=1000.0),
        # After first burst of a group, emit partial result if no further bursts
        # arrive within this time (SAME repeats are ~1 s apart).
        cv.Optional(CONF_BURST_TIMEOUT, default="12s"): cv.positive_time_period_milliseconds,
        cv.Optional(CONF_DECODE_COUNT_SENSOR): cv.use_id(sensor.Sensor),
        cv.Optional(CONF_LAST_RAW_SENSOR): cv.use_id(text_sensor.TextSensor),
        cv.Optional(CONF_QUALITY_SENSOR): cv.use_id(sensor.Sensor),
        cv.Optional(CONF_SNR_SENSOR): cv.use_id(sensor.Sensor),
        cv.Optional(CONF_FREQ_OFFSET_SENSOR): cv.use_id(sensor.Sensor),
        cv.Optional(CONF_ON_ALERT): automation.validate_automation(
            {
                cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(AlertTrigger),
            }
        ),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    cg.add(var.set_sample_rate(config[CONF_SAMPLE_RATE]))
    cg.add(var.set_gain(config[CONF_GAIN]))
    cg.add(var.set_burst_timeout_ms(config[CONF_BURST_TIMEOUT]))

    if CONF_DECODE_COUNT_SENSOR in config:
        s = await cg.get_variable(config[CONF_DECODE_COUNT_SENSOR])
        cg.add(var.set_decode_count_sensor(s))
    if CONF_LAST_RAW_SENSOR in config:
        ts = await cg.get_variable(config[CONF_LAST_RAW_SENSOR])
        cg.add(var.set_last_raw_sensor(ts))
    if CONF_QUALITY_SENSOR in config:
        s = await cg.get_variable(config[CONF_QUALITY_SENSOR])
        cg.add(var.set_quality_sensor(s))
    if CONF_SNR_SENSOR in config:
        s = await cg.get_variable(config[CONF_SNR_SENSOR])
        cg.add(var.set_snr_sensor(s))
    if CONF_FREQ_OFFSET_SENSOR in config:
        s = await cg.get_variable(config[CONF_FREQ_OFFSET_SENSOR])
        cg.add(var.set_freq_offset_sensor(s))

    for conf in config.get(CONF_ON_ALERT, []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID])
        await automation.build_automation(trigger, [], conf)
        cg.add(var.register_alert_trigger(trigger))
