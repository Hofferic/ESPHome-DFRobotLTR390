import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c
from esphome.const import CONF_ID

DEPENDENCIES = ["i2c"]
MULTI_CONF = True

dfrobot_ltr390_ns = cg.esphome_ns.namespace("dfrobot_ltr390")
DFRobotLTR390Component = dfrobot_ltr390_ns.class_(
    "DFRobotLTR390Component", cg.PollingComponent, i2c.I2CDevice
)

CONF_DFROBOT_LTR390_ID = "dfrobot_ltr390_id"

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(DFRobotLTR390Component),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(cv.polling_component_schema("60s"))  # Default update interval
    .extend(i2c.i2c_device_schema(0x1C))
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)
