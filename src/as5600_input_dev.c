#include <zephyr/device.h>
#include <zephyr/init.h>

/* Minimal stub driver to register the scroll_input DT node as a real device,
 * so DEVICE_DT_GET and INPUT_CALLBACK_DEFINE can reference it. */
#define DT_DRV_COMPAT zmk_as5600_input

static int as5600_input_init(const struct device *dev)
{
    return 0;
}

#define AS5600_INPUT_DEFINE(n) \
    DEVICE_DT_INST_DEFINE(n, as5600_input_init, NULL, NULL, NULL, \
                          POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE, NULL);

DT_INST_FOREACH_STATUS_OKAY(AS5600_INPUT_DEFINE)