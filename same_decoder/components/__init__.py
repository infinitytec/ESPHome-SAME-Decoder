# my_components/same_decoder/__init__.py
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import automation
from esphome.components import i2s_audio, text_sensor, sensor, binary_sensor
from esphome.const import CONF_ID, CONF_SAMPLE_RATE

CODEOWNERS = ["@you"]
DEPENDENCIES = ["i2s_audio"]
AUTO_LOAD = ["text_sensor", "sensor", "binary_sensor"]

same_ns = cg.esphome_ns.namespace("same_decoder")
SAMEDecoder = same_ns.class_("SAMEDecoder", cg.Component)
OnAlertTrigger = same_ns.class_("OnAlertTrigger", automation.Trigger.template())

CONF_I2S_DIN_PIN = "i2s_din_pin"
CONF_FIPS_SOURCE = "fips_source"
CONF_ALERT_ACTIVE = "alert_active"
CONF_EVENT_CODE_SENSOR = "event_code_sensor"
CONF_EVENT_SENSOR = "event_sensor"
CONF_SEVERITY_SENSOR = "severity_sensor"
CONF_ORIGINATOR_SENSOR = "originator_sensor"
CONF_FIPS_AFFECTED_SENSOR = "fips_affected_sensor"
CONF_RAW_HEADER_SENSOR = "raw_header_sensor"
CONF_SENDER_SENSOR = "sender_sensor"
CONF_EXPIRES_SENSOR = "expires_sensor"
CONF_ON_ALERT = "on_alert"

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(SAMEDecoder),
    cv.GenerateID(i2s_audio.CONF_I2S_AUDIO_ID): cv.use_id(i2s_audio.I2SAudioComponent),
    cv.Required(CONF_I2S_DIN_PIN): cv.int_,
    cv.Optional(CONF_SAMPLE_RATE, default=48000): cv.int_range(min=8000, max=48000),
    cv.Required(CONF_FIPS_SOURCE): cv.use_id(text_sensor.TextSensor),
    cv.Required(CONF_ALERT_ACTIVE): cv.use_id(binary_sensor.BinarySensor),
    cv.Required(CONF_EVENT_CODE_SENSOR): cv.use_id(text_sensor.TextSensor),
    cv.Required(CONF_EVENT_SENSOR): cv.use_id(text_sensor.TextSensor),
    cv.Required(CONF_SEVERITY_SENSOR): cv.use_id(text_sensor.TextSensor),
    cv.Required(CONF_ORIGINATOR_SENSOR): cv.use_id(text_sensor.TextSensor),
    cv.Required(CONF_FIPS_AFFECTED_SENSOR): cv.use_id(text_sensor.TextSensor),
    cv.Required(CONF_RAW_HEADER_SENSOR): cv.use_id(text_sensor.TextSensor),
    cv.Required(CONF_SENDER_SENSOR): cv.use_id(text_sensor.TextSensor),
    cv.Required(CONF_EXPIRES_SENSOR): cv.use_id(sensor.Sensor),
    cv.Optional(CONF_ON_ALERT): automation.validate_automation({
        cv.GenerateID(automation.CONF_TRIGGER_ID): cv.declare_id(OnAlertTrigger),
    }),
}).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    parent = await cg.get_variable(config[i2s_audio.CONF_I2S_AUDIO_ID])
    cg.add(var.set_i2s_parent(parent))
    cg.add(var.set_din_pin(config[CONF_I2S_DIN_PIN]))
    cg.add(var.set_sample_rate(config[CONF_SAMPLE_RATE]))

    for key, setter in [
        (CONF_FIPS_SOURCE, var.set_fips_source),
        (CONF_ALERT_ACTIVE, var.set_alert_active),
        (CONF_EVENT_CODE_SENSOR, var.set_event_code_sensor),
        (CONF_EVENT_SENSOR, var.set_event_sensor),
        (CONF_SEVERITY_SENSOR, var.set_severity_sensor),
        (CONF_ORIGINATOR_SENSOR, var.set_originator_sensor),
        (CONF_FIPS_AFFECTED_SENSOR, var.set_fips_affected_sensor),
        (CONF_RAW_HEADER_SENSOR, var.set_raw_header_sensor),
        (CONF_SENDER_SENSOR, var.set_sender_sensor),
        (CONF_EXPIRES_SENSOR, var.set_expires_sensor),
    ]:
        cg.add(setter(await cg.get_variable(config[key])))

    for conf in config.get(CONF_ON_ALERT, []):
        trigger = cg.new_Pvariable(conf[automation.CONF_TRIGGER_ID])
        await automation.build_automation(trigger, [], conf)
        cg.add(var.add_on_alert_trigger(trigger))
