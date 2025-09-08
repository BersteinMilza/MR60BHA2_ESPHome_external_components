import esphome.codegen as cg
from esphome.components import text_sensor
import esphome.config_validation as cv
from esphome.const import (
    ICON_CHIP,
    ICON_INFORMATION,
)

from . import CONF_MR60BHA2_ID, MR60BHA2Component

DEPENDENCIES = ["seeed_mr60bha2"]

CONF_TARGET_INFO = "target_info"
CONF_FIRMWARE_VERSION = "firmware_version"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_MR60BHA2_ID): cv.use_id(MR60BHA2Component),
        cv.Optional(CONF_TARGET_INFO): text_sensor.text_sensor_schema(
            icon=ICON_INFORMATION_OUTLINE
        ),
        cv.Optional(CONF_FIRMWARE_VERSION): text_sensor.text_sensor_schema(
            icon=ICON_CHIP
        ),
    }
)

async def to_code(config):
    mr60bha2_component = await cg.get_variable(config[CONF_MR60BHA2_ID])
    if target_info_config := config.get(CONF_TARGET_INFO):
        sens = await text_sensor.new_text_sensor(target_info_config)
        cg.add(mr60bha2_component.set_target_info_text_sensor(sens))
    if firmware_version_config := config.get(CONF_FIRMWARE_VERSION):
        sens = await text_sensor.new_text_sensor(firmware_version_config)
        cg.add(mr60bha2_component.set_firmware_version_text_sensor(sens))
