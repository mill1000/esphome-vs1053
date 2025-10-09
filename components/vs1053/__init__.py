from typing import Optional

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import spi
from esphome.const import CONF_ID

CODEOWNERS = ["@mill1000"]
DEPENDENCIES = ["spi"]

MULTI_CONF = True
CONF_VS1053_ID = "vs1053_id"

vs1053_ns = cg.esphome_ns.namespace("vs1053")
VS1053 = vs1053_ns.class_(
    "VS1053", spi.SPIDevice, cg.Component)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(VS1053),
    }
).extend(spi.spi_device_schema(cs_pin_required=True))


async def to_code(config) -> None:
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await spi.register_spi_device(var, config)
