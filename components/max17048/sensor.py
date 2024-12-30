import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c, sensor
from esphome.const import (
    CONF_BATTERY_LEVEL,
    CONF_BATTERY_VOLTAGE,
    CONF_RATE,
    CONF_CONFIGURATION,
    CONF_ID,
    CONF_VERSION,
    DEVICE_CLASS_BATTERY,
    UNIT_HOUR,
    UNIT_PERCENT,
    UNIT_VOLT,
    STATE_CLASS_MEASUREMENT,
)

DEPENDENCIES = ["i2c"]

max17048_ns = cg.esphome_ns.namespace("max17048")
MAX17048Component = max17048_ns.class_(
    "MAX17048Component", cg.PollingComponent, i2c.I2CDevice
)

MAX17048BatteryLevelDevice = max17048_ns.class_("MAX17048BatteryLevelDevice", i2c.I2CDevice)


CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(MAX17048Component),
            cv.Optional(CONF_BATTERY_VOLTAGE): sensor.sensor_schema(
                unit_of_measurement=UNIT_VOLT,
                accuracy_decimals=3,  # Actual Resolution: 78.125µV/cell
                device_class=DEVICE_CLASS_BATTERY,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_BATTERY_LEVEL): sensor.sensor_schema(
                unit_of_measurement=UNIT_PERCENT,
                accuracy_decimals=2,  # Actual Resolution: 1/256th of a percent
                device_class=DEVICE_CLASS_BATTERY,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_RATE): sensor.sensor_schema(
                unit_of_measurement=f"{UNIT_PERCENT}/{UNIT_HOUR}",
                accuracy_decimals=1,  # Actual Resolution: 0.208%/hr
                device_class=DEVICE_CLASS_BATTERY,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_CONFIGURATION): sensor.sensor_schema(
                unit_of_measurement=f"0x",
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_BATTERY,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_VERSION): sensor.sensor_schema(
                unit_of_measurement=UNIT_PERCENT,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_NONE,
                state_class=STATE_CLASS_MEASUREMENT,
            )
        }
    )
    .extend(cv.polling_component_schema("60s"))
    .extend(i2c.i2c_device_schema(0x36))
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])

    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)
    
    if CONF_BATTERY_VOLTAGE in config:
        battery_v_sensor = await sensor.new_sensor(config[CONF_BATTERY_VOLTAGE])
        cg.add(var.set_battery_v_sensor(battery_v_sensor))

    if CONF_BATTERY_LEVEL in config:
        battery_soc_sensor = await sensor.new_sensor(config[CONF_BATTERY_LEVEL])
        cg.add(var.set_battery_soc_sensor(battery_soc_sensor))
    
    if CONF_RATE in config:
        battery_soc_rate_sensor = await sensor.new_sensor(config[CONF_RATE])
        cg.add(var.set_battery_soc_rate_sensor(battery_soc_rate_sensor))

    if CONF_CONFIGURATION in config:
        config_sensor = await sensor.new_sensor(config[CONF_CONFIGURATION])
        cg.add(var.set_config_sensor(config_sensor))

    if CONF_VERSION in config:
        version_sensor = await sensor.new_sensor(config[CONF_VERSION])
        cg.add(var.set_version_sensor(version_sensor))
