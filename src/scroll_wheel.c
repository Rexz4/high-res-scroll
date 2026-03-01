#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/input/input.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/drivers/led_strip.h>
#include <zmk/ble.h>
#include <zmk/pm.h>
#include <zephyr/settings/settings.h>

#define AS5600_RAW_ANGLE_REG  0x0C
#define AS5600_MAX_ANGLE      4096
#define AS5600_HALF_MAX       2048
#define SAMPLE_INTERVAL_MS    4
#define LONG_PRESS_MS         1500
#define LONGEST_PRESS_MS      3000

static const struct i2c_dt_spec as5600     = I2C_DT_SPEC_GET(DT_NODELABEL(as5600));
static const struct gpio_dt_spec btn_power = GPIO_DT_SPEC_GET(DT_NODELABEL(power_button), gpios);
static const struct gpio_dt_spec btn_sens  = GPIO_DT_SPEC_GET(DT_NODELABEL(sens_button), gpios);
static const struct device *led_strip      = DEVICE_DT_GET(DT_NODELABEL(ws2812));
static const struct device *scroll_dev     = DEVICE_DT_GET(DT_NODELABEL(scroll_input));

static struct led_rgb pixels[1];

static int sens_levels[] = {16, 8};
static int sens_idx = 0;
static atomic_t current_ticks_per_scroll = ATOMIC_INIT(16);

static int read_angle(uint16_t *out)
{
    uint8_t reg = AS5600_RAW_ANGLE_REG;
    uint8_t buf[2];
    int ret = i2c_write_read_dt(&as5600, &reg, 1, buf, 2);
    if (ret == 0) {
        *out = ((uint16_t)buf[0] << 8) | buf[1];
    }
    return ret;
}

static int16_t compute_delta(uint16_t prev, uint16_t curr)
{
    int16_t delta = (int16_t)curr - (int16_t)prev;
    if (delta >  AS5600_HALF_MAX) delta -= AS5600_MAX_ANGLE;
    if (delta < -AS5600_HALF_MAX) delta += AS5600_MAX_ANGLE;
    return delta;
}

static void led_set_color(uint8_t r, uint8_t g, uint8_t b)
{
    pixels[0].r = r;
    pixels[0].g = g;
    pixels[0].b = b;
    led_strip_update_rgb(led_strip, pixels, 1);
}

static void led_blink(int times, int on_ms, int off_ms, uint8_t r, uint8_t g, uint8_t b)
{
    for (int i = 0; i < times; i++) {
        led_set_color(r, g, b);
        k_msleep(on_ms);
        led_set_color(0, 0, 0);
        k_msleep(off_ms);
    }
}

static void clear_bonds_work_fn(struct k_work *work)
{
    zmk_ble_clear_all_bonds();
}

static K_WORK_DEFINE(clear_bonds_work, clear_bonds_work_fn);

/* Scroll handling */
static void scroll_thread_fn(void *a, void *b, void *c)
{
    if (!device_is_ready(as5600.bus)) {
        printk("AS5600 bus not ready\n");
        return;
    }

    /* Check magnet status */
    uint8_t status_reg = 0x0B;
    uint8_t status;
    i2c_write_read_dt(&as5600, &status_reg, 1, &status, 1);
    printk("Magnet: detected=%d too_weak=%d too_strong=%d\n",
           !!(status & BIT(5)),
           !!(status & BIT(4)),
           !!(status & BIT(3)));

    /* Enable watchdog (WD bit) + 1 LSB hysteresis */
    uint8_t conf_reg = 0x07;
    uint8_t conf_buf[2];
    i2c_write_read_dt(&as5600, &conf_reg, 1, conf_buf, 2);
    conf_buf[0] |= 0x20;
    conf_buf[1] |= (1 << 2);
    uint8_t conf_write[] = {0x07, conf_buf[0], conf_buf[1]};
    i2c_write_dt(&as5600, conf_write, sizeof(conf_write));

    uint16_t prev_angle = 0;
    int32_t  accumulator = 0;
    read_angle(&prev_angle);

    while (1) {
        k_msleep(SAMPLE_INTERVAL_MS);

        uint16_t curr_angle;
        if (read_angle(&curr_angle) != 0) {
            continue;
        }

        int16_t delta = compute_delta(prev_angle, curr_angle);
        prev_angle = curr_angle;

        if (delta == 0) continue;

        accumulator += delta;
        int32_t ticks = atomic_get(&current_ticks_per_scroll);
        int32_t scroll_ticks = accumulator / ticks;

        if (scroll_ticks != 0) {
            accumulator -= scroll_ticks * ticks;
            input_report_rel(scroll_dev, INPUT_REL_WHEEL, (int16_t)scroll_ticks, true, K_NO_WAIT);
        }
    }
}

K_THREAD_DEFINE(scroll_tid, 1024, scroll_thread_fn, NULL, NULL, NULL, K_PRIO_PREEMPT(10), 0, 0);

/* Power and BT button */
static void power_button_thread_fn(void *a, void *b, void *c)
{
    if (!device_is_ready(led_strip)) {
        printk("LED strip not ready\n");
        return;
    }
    if (!gpio_is_ready_dt(&btn_power)) {
        printk("Power button not ready\n");
        return;
    }

    gpio_pin_configure_dt(&btn_power, GPIO_INPUT);
    led_set_color(0, 0, 0);

    bool last_state = false;
    int64_t press_time = 0;
    bool longest_triggered = false;

    while (1) {
        bool current_state = gpio_pin_get_dt(&btn_power) > 0;

        if (current_state && !last_state) {
            press_time = k_uptime_get();
            longest_triggered = false;
        } else if (current_state && last_state && !longest_triggered) {
            if (k_uptime_get() - press_time >= LONGEST_PRESS_MS) {
                longest_triggered = true;
                printk("Clearing bonds\n");
                led_blink(3, 100, 100, 50, 25, 0);
                k_work_submit(&clear_bonds_work);
            }
        } else if (!current_state && last_state && !longest_triggered) {
            int64_t held_ms = k_uptime_get() - press_time;

            if (held_ms >= LONG_PRESS_MS) {
                printk("Entering soft off\n");
                led_blink(2, 100, 100, 50, 0, 0);
                settings_save();  /* force flush all settings to flash */
                k_msleep(1000);    /* wait for flash write to complete */
                zmk_pm_soft_off();
            } else {
                printk("BT reset\n");
                led_blink(1, 100, 100, 0, 0, 50);
                zmk_ble_prof_disconnect(0);
            }
        }

        last_state = current_state;
        k_msleep(50);
    }
}

K_THREAD_DEFINE(led_btn_power_tid, 1024, power_button_thread_fn, NULL, NULL, NULL, K_PRIO_PREEMPT(7), 0, 0);

/* Sensitivity button toggle */
static void sens_button_thread_fn(void *a, void *b, void *c)
{
    if (!gpio_is_ready_dt(&btn_sens)) {
        printk("Sensitivity button not ready\n");
        return;
    }

    gpio_pin_configure_dt(&btn_sens, GPIO_INPUT);

    bool last_btn_state = false;

    while (1) {
        bool current_btn_state = gpio_pin_get_dt(&btn_sens) > 0;

        if (current_btn_state && !last_btn_state) {
            sens_idx = (sens_idx + 1) % ARRAY_SIZE(sens_levels);
            atomic_set(&current_ticks_per_scroll, sens_levels[sens_idx]);
            printk("Sensitivity: %d\n", sens_levels[sens_idx]);
        }

        last_btn_state = current_btn_state;
        k_msleep(50);
    }
}

K_THREAD_DEFINE(sens_btn_tid, 512, sens_button_thread_fn, NULL, NULL, NULL, K_PRIO_PREEMPT(8), 0, 0);