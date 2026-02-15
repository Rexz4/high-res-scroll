#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/printk.h>

static const struct i2c_dt_spec as5600 = I2C_DT_SPEC_GET(DT_ALIAS(sensor));

int main(void) {
    int32_t last_angle = -1;
    
    printk("AS5600 Serial Debug Start\n");
    printk("--------------------------\n");

    if (!device_is_ready(as5600.bus)) {
        printk("Error: I2C bus not ready!\n");
        return 0;
    }

    while (1) {
        uint8_t buffer[2];
        // Register 0x0C is the start of the RAW ANGLE (high byte)
        if (i2c_write_read_dt(&as5600, (uint8_t[]){0x0C}, 1, buffer, 2) == 0) {
            uint16_t angle = (((uint16_t)buffer[0] << 8) | buffer[1]) & 0x0FFF;

            if (last_angle != -1) {
                int32_t delta = (int32_t)angle - last_angle;

                // Handle zero-crossing for 12-bit sensor (4096 steps)
                if (delta > 2048)  delta -= 4096;
                if (delta < -2048) delta += 4096;

                if (delta != 0) {
                    printk("Angle: %4d | Delta: %4d\n", angle, delta);
                }
            } else {
                printk("First read: %d\n", angle);
            }
            last_angle = angle;
        } else {
            printk("I2C Read Failed!\n");
        }

        k_msleep(100); // Slowed down for readability
    }
}