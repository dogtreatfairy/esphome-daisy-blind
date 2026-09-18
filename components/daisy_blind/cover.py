import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.components import cover
from esphome.const import CONF_ID, CONF_DIR_PIN, CONF_SLEEP_PIN, CONF_STEP_PIN, CONF_PORT

DEPENDENCIES = ["network"]

daisy_blind_ns = cg.esphome_ns.namespace("daisy_blind")
DaisyBlind = daisy_blind_ns.class_("DaisyBlind", cover.Cover, cg.Component)

CONFIG_SCHEMA = cv.All(
    cover.cover_schema(DaisyBlind)
    .extend(
        {
            cv.Required(CONF_STEP_PIN): pins.gpio_output_pin_schema,
            cv.Required(CONF_DIR_PIN): pins.gpio_output_pin_schema,
            cv.Optional(CONF_SLEEP_PIN): pins.gpio_output_pin_schema,
            cv.Optional(CONF_PORT, default=44820): cv.port,
        }
    )
    .extend(cv.COMPONENT_SCHEMA),
    cv.only_with_arduino,
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cover.register_cover(var, config)
    await cg.register_component(var, config)

    step = await cg.gpio_pin_expression(config[CONF_STEP_PIN])
    cg.add(var.set_step_pin(step))
    dir_ = await cg.gpio_pin_expression(config[CONF_DIR_PIN])
    cg.add(var.set_dir_pin(dir_))
    if CONF_SLEEP_PIN in config:
        sleep = await cg.gpio_pin_expression(config[CONF_SLEEP_PIN])
        cg.add(var.set_sleep_pin(sleep))
    cg.add(var.set_port(config[CONF_PORT]))
