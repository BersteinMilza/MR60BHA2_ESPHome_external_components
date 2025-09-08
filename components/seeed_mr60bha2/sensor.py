import esphome.codegen as cg
from esphome.components import sensor
import esphome.config_validation as cv
from esphome.const import (
    CONF_DISTANCE,
    DEVICE_CLASS_DISTANCE,
    ICON_HEART_PULSE,
    ICON_PULSE,
    ICON_SIGNAL,
    ICON_COUNTER,
    STATE_CLASS_MEASUREMENT,
    UNIT_BEATS_PER_MINUTE,
    UNIT_CENTIMETER,
    ICON_ROTATE_RIGHT,
    CONF_X,
    CONF_Y,
    UNIT_METER,
    ICON_AXIS_ARROW,
)

from . import CONF_MR60BHA2_ID, MR60BHA2Component

DEPENDENCIES = ["seeed_mr60bha2"]

CONF_BREATH_RATE = "breath_rate"
CONF_HEART_RATE = "heart_rate"
CONF_NUM_TARGETS = "num_targets"
CONF_TOTAL_PHASE = "total_phase"
CONF_BREATH_PHASE = "breath_phase"
CONF_HEART_PHASE = "heart_phase"

# New keys for targets
CONF_TARGET_1 = "target_1"
CONF_TARGET_2 = "target_2"
CONF_TARGET_3 = "target_3"

# Schema for a single target's X and Y sensors
TARGET_SCHEMA = cv.Schema({
    cv.Optional(CONF_X): sensor.sensor_schema(
        unit_of_measurement=UNIT_METER,
        icon=ICON_AXIS_ARROW,
        accuracy_decimals=2,
        state_class=STATE_CLASS_MEASUREMENT,
    ),
    cv.Optional(CONF_Y): sensor.sensor_schema(
        unit_of_measurement=UNIT_METER,
        icon=ICON_AXIS_ARROW,
        accuracy_decimals=2,
        state_class=STATE_CLASS_MEASUREMENT,
    ),
})

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(CONF_MR60BHA2_ID): cv.use_id(MR60BHA2Component),
    cv.Optional(CONF_BREATH_RATE): sensor.sensor_schema(),
    cv.Optional(CONF_HEART_RATE): sensor.sensor_schema(),
    cv.Optional(CONF_DISTANCE): sensor.sensor_schema(),
    cv.Optional(CONF_NUM_TARGETS): sensor.sensor_schema(),
    cv.Optional(CONF_TOTAL_PHASE): sensor.sensor_schema(),
    cv.Optional(CONF_BREATH_PHASE): sensor.sensor_schema(),
    cv.Optional(CONF_HEART_PHASE): sensor.sensor_schema(),
    # Add new target schemas
    cv.Optional(CONF_TARGET_1): TARGET_SCHEMA,
    cv.Optional(CONF_TARGET_2): TARGET_SCHEMA,
    cv.Optional(CONF_TARGET_3): TARGET_SCHEMA,
})

async def to_code(config):
    mr60bha2_component = await cg.get_variable(config[CONF_MR60BHA2_ID])
    
    # Standard sensors (simplified for brevity)
    for key in [CONF_BREATH_RATE, CONF_HEART_RATE, CONF_DISTANCE, CONF_NUM_TARGETS, CONF_TOTAL_PHASE, CONF_BREATH_PHASE, CONF_HEART_PHASE]:
        if key in config:
            sens = await sensor.new_sensor(config[key])
            cg.add(getattr(mr60bha2_component, f"set_{key}_sensor")(sens))

    # New target sensors
    for i in range(1, 4):
        target_key = f"target_{i}"
        if target_config := config.get(target_key):
            if x_config := target_config.get(CONF_X):
                sens = await sensor.new_sensor(x_config)
                cg.add(getattr(mr60bha2_component, f"set_{target_key}_x_sensor")(sens))
            if y_config := target_config.get(CONF_Y):
                sens = await sensor.new_sensor(y_config)
                cg.add(getattr(mr6e0bha2_component, f"set_{target_key}_y_sensor")(sens))
