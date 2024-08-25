/*
 * Copyright (c) 2022 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT ti_drv883x

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/input/input.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/types.h>
#include <stddef.h>
#include <zephyr/drivers/pinctrl.h>

#include <zmk/drivers/drv883x.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(drv883x, CONFIG_DRV883X_LOG_LEVEL);

/* device data structure */
struct drv883x_data {
    const struct device *dev;
    bool ready; // whether init is finished successfully    
    uint32_t velocity;
    bool inverse;
};

/* device config data structure */
struct drv883x_config {
    bool has_enable;
    const struct gpio_dt_spec enable;

    bool has_pwms;
    struct pwm_dt_spec pwm1;
    struct pwm_dt_spec pwm2;

    bool has_in1;
    const struct gpio_dt_spec in1;
    bool has_in2;
    const struct gpio_dt_spec in2;
};

static int enable_set_value(const struct device *dev, uint8_t value) {
    const struct drv883x_config *config = dev->config;
    // struct drv883x_data *data = dev->data;

    if (config->has_enable) {
        if (gpio_pin_set_dt(&config->enable, value ? 1 : 0)) {
            LOG_WRN("Failed to set enable pin");
            return -EIO;
        }
    }

    return 0;
}

static int pwm_set_value(struct pwm_dt_spec pwm, uint8_t value) {
    if (!pwm_is_ready_dt(&pwm)) {
        LOG_WRN("PWM is not ready");
        return -EIO;
    }
    uint32_t period = pwm.period;
    if (value) {
        uint32_t pulse = period / 256.0 * value;
        LOG_DBG("set pwm: chan %d val %d period %d pulse %d", pwm.channel, value, period, pulse);
        pwm_set_dt(&pwm, period, pulse);
        // const struct pwm_dt_spec *spec = &pwm;
        // pwm_set(spec->dev, spec->channel, period, pulse, spec->flags);
    }
    else {
        LOG_DBG("unset pwm: chan %d", pwm.channel);
        pwm_set_dt(&pwm, period, 0);
    }
    return 0;
}

static int velocity_set_value(const struct device *dev, uint16_t vel, bool inv) {
    // const struct drv883x_config *config = dev->config;
    struct drv883x_data *data = dev->data;

    LOG_DBG("set velocity: %d  direction: [%s]", vel, inv ? "-" : "+");
    data->velocity = vel;
    data->inverse = inv;

    return 0;
}

static int sync_set_value(const struct device *dev, uint16_t chan) {
    const struct drv883x_config *config = dev->config;
    struct drv883x_data *data = dev->data;

    uint8_t in1;
    uint8_t in2;
    if (!data->inverse) {
        in1 = data->velocity;
        in2 = 0;
    } else {
        in1 = 0;
        in2 = data->velocity;
    }
    LOG_DBG("sync input: %d / %d", in1, in2);

    if (config->has_pwms) {
        if (pwm_set_value(config->pwm1, in1)) {
            LOG_WRN("Failed to set PWM1 pin");
            return -EIO;
        }
        if (pwm_set_value(config->pwm2, in2)) {
            LOG_WRN("Failed to set PWM2 pin");
            return -EIO;
        }
    }

    if (config->has_in1) {
        if (gpio_pin_set_dt(&config->in1, in1)) {
            LOG_WRN("Failed to set IN1 pin");
            return -EIO;
        }
    }
    if (config->has_in2) {
        if (gpio_pin_set_dt(&config->in2, in2)) {
            LOG_WRN("Failed to set IN2 pin");
            return -EIO;
        }
    }

    return 0;
}

static int drv883x_init(const struct device *dev) {
    struct drv883x_data *data = dev->data;
    const struct drv883x_config *config = dev->config;
    int err;

    // init device pointer
    data->dev = dev;

    if (config->has_enable) {
        if (gpio_pin_configure_dt(&config->enable, GPIO_OUTPUT_INACTIVE)) {
            LOG_ERR("Failed to configure output enable pin");
            return -EIO;
        }
    }

    if (config->has_in1) {
        if (gpio_pin_configure_dt(&config->in1, GPIO_OUTPUT_INACTIVE)) {
            LOG_ERR("Failed to configure output in1 pin");
            return -EIO;
        }
    }

    if (config->has_in2) {
        if (gpio_pin_configure_dt(&config->in2, GPIO_OUTPUT_INACTIVE)) {
            LOG_ERR("Failed to configure output in2 pin");
            return -EIO;
        }
    }

    enable_set_value(dev, 0);
    velocity_set_value(dev, 0, true);

    data->ready = true;
    return err;
}

static int drv883x_attr_set(const struct device *dev, enum sensor_channel chan,
                            enum sensor_attribute attr, const struct sensor_value *val) {
    struct drv883x_data *data = dev->data;
    int err;

    if (unlikely(!data->ready)) {
        LOG_DBG("Device is not initialized yet");
        return -EBUSY;
    }

    switch ((uint32_t)attr) {
    case DRV883X_ATTR_ENABLE:
        err = enable_set_value(dev, DRV883X_SVALUE_TO_ENABLE(*val));
        break;

	case DRV883X_ATTR_VELOCITY:
        err = velocity_set_value(dev, DRV883X_SVALUE_TO_VELOCITY_VEL(*val),
                                      DRV883X_SVALUE_TO_VELOCITY_INV(*val));
        break;

	case DRV883X_ATTR_SYNC:
        err = sync_set_value(dev, DRV883X_SVALUE_TO_SYNC(*val));
        break;

    default:
        LOG_ERR("Unknown attribute");
        err = -ENOTSUP;
    }

    return err;
}

static const struct sensor_driver_api drv883x_driver_api = {
    .attr_set = drv883x_attr_set,
};


#define DRV883X_DEFINE(n)                                                      \
    static struct drv883x_data data##n;                                        \
    static const struct drv883x_config config##n = {                           \
        .has_enable = COND_CODE_1(                                             \
                        DT_INST_NODE_HAS_PROP(n, enable_gpios),                \
                        (true), (false)),                                      \
        .enable = COND_CODE_1(                                                 \
                    DT_INST_NODE_HAS_PROP(n, enable_gpios),                    \
                    (GPIO_DT_SPEC_INST_GET(n, enable_gpios)),                  \
                    (NULL)),                                                   \
        .has_pwms = COND_CODE_1(                                               \
                      DT_INST_NODE_HAS_PROP(n, pwms),                          \
                      (true), (false)),                                        \
        .pwm1 = COND_CODE_1(                                                   \
                  DT_INST_NODE_HAS_PROP(n, pwms),                              \
                  (PWM_DT_SPEC_GET_BY_IDX(DT_DRV_INST(n), 0)),                 \
                  (NULL)),                                                     \
        .pwm2 = COND_CODE_1(                                                   \
                  DT_INST_NODE_HAS_PROP(n, pwms),                              \
                  (PWM_DT_SPEC_GET_BY_IDX(DT_DRV_INST(n), 1)),                 \
                  (NULL)),                                                     \
        .has_in1 = COND_CODE_1(                                                \
                     DT_INST_NODE_HAS_PROP(n, in1_gpios),                      \
                     (true), (false)),                                         \
        .in1 = COND_CODE_1(                                                    \
                 DT_INST_NODE_HAS_PROP(n, in1_gpios),                          \
                 (GPIO_DT_SPEC_INST_GET(n, in1_gpios)),                        \
                 (NULL)),                                                      \
        .has_in2 = COND_CODE_1(                                                \
                     DT_INST_NODE_HAS_PROP(n, in2_gpios),                      \
                     (true), (false)),                                         \
        .in2 = COND_CODE_1(                                                    \
                 DT_INST_NODE_HAS_PROP(n, in2_gpios),                          \
                 (GPIO_DT_SPEC_INST_GET(n, in2_gpios)),                        \
                 (NULL)),                                                      \
    };                                                                         \
    DEVICE_DT_INST_DEFINE(n, drv883x_init, DEVICE_DT_INST_GET(n),              \
                          &data##n, &config##n,                                \
                          POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,            \
                          &drv883x_driver_api);

DT_INST_FOREACH_STATUS_OKAY(DRV883X_DEFINE)
