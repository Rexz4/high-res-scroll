#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/input/input.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>
#include <zephyr/sys/atomic.h>
#include <zmk/ble.h>
#include <zephyr/bluetooth/bluetooth.h>

#define AS5600_RAW_ANGLE_REG 0x0C
#define AS5600_MAX_ANGLE 4096
#define AS5600_HALF_MAX 2048
#define SAMPLE_INTERVAL_MS 4
#define LONG_PRESS_MS 1500
#define LONGEST_PRESS_MS 3000

static const struct i2c_dt_spec as5600 = I2C_DT_SPEC_GET(DT_NODELABEL(as5600));
static const struct gpio_dt_spec led_status = GPIO_DT_SPEC_GET(DT_NODELABEL(status_led), gpios);
static const struct gpio_dt_spec btn_power = GPIO_DT_SPEC_GET(DT_NODELABEL(power_button), gpios);
static const struct gpio_dt_spec btn_sens = GPIO_DT_SPEC_GET(DT_NODELABEL(sens_button), gpios);

static const struct device *scroll_dev = DEVICE_DT_GET(DT_NODELABEL(scroll_input));

static int sens_levels[] = {8, 16};
static int sens_idx = 0;

static bool advertising_stopped = false;

static atomic_t current_ticks_per_scroll = ATOMIC_INIT(8);

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

static void clear_bonds_work_fn(struct k_work *work)
{
    zmk_ble_clear_all_bonds();
}

static K_WORK_DEFINE(clear_bonds_work, clear_bonds_work_fn);

static void led_blink(int times, int on_ms, int off_ms)
{
    for (int i = 0; i < times; i++) {
        gpio_pin_set_dt(&led_status, 1);
        k_msleep(on_ms);
        gpio_pin_set_dt(&led_status, 0);
        k_msleep(off_ms);
    }
}

/* Scroll handling */
static void scroll_thread_fn(void *a, void *b, void *c)
{
    if (!device_is_ready(as5600.bus)) {
        printk("AS5600 bus not ready\n");
        return;
    }

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
    if (!gpio_is_ready_dt(&led_status) || !gpio_is_ready_dt(&btn_power)) {
        printk("GPIO not ready\n");
        return;
    }

    gpio_pin_configure_dt(&led_status, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&btn_power, GPIO_INPUT);

    bool last_state = false;
    int64_t press_time = 0;
    bool longest_triggered = false;

    while (1) {
        bool current_state = gpio_pin_get_dt(&btn_power) > 0;

        if (current_state && !last_state) {
            press_time = k_uptime_get();
            longest_triggered = false;
        } else if (current_state && last_state && !longest_triggered) {
            /* Still held — check longest threshold */
            if (k_uptime_get() - press_time >= LONGEST_PRESS_MS) {
                longest_triggered = true;
                printk("Clearing bonds\n");
                led_blink(3, 100, 100);
                k_work_submit(&clear_bonds_work);
            }
        } else if (!current_state && last_state && !longest_triggered) {
            int64_t held_ms = k_uptime_get() - press_time;

            if (held_ms >= LONG_PRESS_MS) {
                /* Long press — deep sleep toggle (later) */
                /* On wake, ZMK will call update_advertising on BT ready */
                led_blink(2, 100, 100);
            } else {
                /* Short press — disconnect and re-advertise */
                printk("BT reset\n");
                led_blink(1, 100, 100);
                zmk_ble_prof_disconnect(0);
                /* ZMK disconnect callback automatically calls update_advertising */
            }
        }

        last_state = current_state;
        k_msleep(50);
    }
}

K_THREAD_DEFINE(led_btn_power_tid, 512, power_button_thread_fn, NULL, NULL, NULL, K_PRIO_PREEMPT(12), 0, 0);

/* Sensitivity button toggle */
static void sens_button_thread_fn(void *a, void *b, void *c)
{
    if (!gpio_is_ready_dt(&led_status) || !gpio_is_ready_dt(&btn_sens)) {
        printk("GPIO not ready\n");
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

K_THREAD_DEFINE(sens_btn_power_tid, 512, sens_button_thread_fn, NULL, NULL, NULL, K_PRIO_PREEMPT(11), 0, 0);