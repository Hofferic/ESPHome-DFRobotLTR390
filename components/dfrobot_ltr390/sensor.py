import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    CONF_ID,
    CONF_GAIN,
    CONF_RESOLUTION,
    CONF_UPDATE_INTERVAL,
    DEVICE_CLASS_ILLUMINANCE,
    DEVICE_CLASS_IRRADIANCE,
    ICON_BRIGHTNESS_5,
    STATE_CLASS_MEASUREMENT,
    UNIT_LUX,
)
from . import DFRobotLTR390Component, CONF_DFROBOT_LTR390_ID

DEPENDENCIES = ["dfrobot_ltr390"]

CONF_AMBIENT_LIGHT = "ambient_light"
CONF_UV_INDEX = "uv_index"
CONF_MEASUREMENT_RATE = "measurement_rate"

GAINS = {
    1: 0x00,
    3: 0x01,
    6: 0x02,
    9: 0x03,
    18: 0x04,
}

RESOLUTIONS = {
    13: 0x00,
    16: 0x01,
    17: 0x02,
    18: 0x03,
    19: 0x04,
    20: 0x05,
}

MEASUREMENT_RATES = {
    25: 0x00,
    50: 0x01,
    100: 0x02,
    200: 0x03,
    500: 0x04,
    1000: 0x05,
    2000: 0x06,
}

def validate_measurement_rate(value):
    value = cv.positive_time_period_milliseconds(value).total_milliseconds
    if value not in MEASUREMENT_RATES:
        raise cv.Invalid(f"Invalid measurement rate: {value}ms")
    return value

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_DFROBOT_LTR390_ID): cv.use_id(DFRobotLTR390Component),
        cv.Optional(CONF_AMBIENT_LIGHT): sensor.sensor_schema(
            unit_of_measurement=UNIT_LUX,
            icon=ICON_BRIGHTNESS_5,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_ILLUMINANCE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_UV_INDEX): sensor.sensor_schema(
            icon=ICON_BRIGHTNESS_5,
            accuracy_decimals=2,
            device_class=DEVICE_CLASS_IRRADIANCE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_GAIN, default=3): cv.enum(GAINS, int=True),
        cv.Optional(CONF_RESOLUTION, default=18): cv.enum(RESOLUTIONS, int=True),
        cv.Optional(CONF_MEASUREMENT_RATE, default="100ms"): validate_measurement_rate,
    }
)
.extend(cv.polling_component_schema("60s"))  # Default update interval


async def to_code(config):
    parent = await cg.get_variable(config[CONF_DFROBOT_LTR390_ID])
    
    if CONF_AMBIENT_LIGHT in config:
        conf = config[CONF_AMBIENT_LIGHT]
        sens = await sensor.new_sensor(conf)
        cg.add(parent.set_ambient_light_sensor(sens))
    
    if CONF_UV_INDEX in config:
        conf = config[CONF_UV_INDEX]
        sens = await sensor.new_sensor(conf)
        cg.add(parent.set_uv_index_sensor(sens))
    
    cg.add(parent.set_gain(GAINS[config[CONF_GAIN]]))
    cg.add(parent.set_resolution(RESOLUTIONS[config[CONF_RESOLUTION]]))
    cg.add(parent.set_measurement_rate(MEASUREMENT_RATES[config[CONF_MEASUREMENT_RATE]]))
