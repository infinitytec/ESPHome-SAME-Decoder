# components/same_decoder/__init__.py
# Config schema + codegen for the SAME decoder.
# Audio is delivered from YAML via microphone: on_data -> feed_bytes(), so this
# schema does NOT own the microphone/I2S. Adds an optional software `gain`
# applied inside feed_bytes() (the source-level gain_factor is not reachable
# through the on_data bridge, so we apply our own — and allow >64 since the
# incoming line-in level can be very low).

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
CONF_GAIN = "gain"

same_decoder_ns = cg.esphome_ns.namespace("same_decoder")
SAMEDecoder = same_decoder_ns.class_("SAMEDecoder", cg.Component)

AlertTrigger = same_decoder_ns.class_("AlertTrigger", automation.Trigger.template())

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(SAMEDecoder),
        cv.Optional(CONF_SAMPLE_RATE, default=48000): cv.int_range(min=8000, max=96000),
        cv.Optional(CONF_GAIN, default=1.0): cv.float_range(min=1.0, max=1000.0),
        cv.Optional(CONF_DECODE_COUNT_SENSOR): cv.use_id(sensor.Sensor),
        cv.Optional(CONF_LAST_RAW_SENSOR): cv.use_id(text_sensor.TextSensor),
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
