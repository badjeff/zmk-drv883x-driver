#pragma once

#include <zephyr/drivers/sensor.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Helper macros used to convert sensor values. */
#define DRV883X_SVALUE_TO_ENABLE(svalue) ((uint32_t)(svalue).val1)
#define DRV883X_SVALUE_TO_VELOCITY_VEL(svalue) ((uint32_t)(svalue).val1)
#define DRV883X_SVALUE_TO_VELOCITY_INV(svalue) ((bool)(svalue).val2)
#define DRV883X_SVALUE_TO_SYNC(svalue) ((uint32_t)(svalue).val1)

/** @brief Sensor specific attributes of DRV883X. */
enum drv883x_attribute {
	DRV883X_ATTR_ENABLE,
	DRV883X_ATTR_VELOCITY,
	DRV883X_ATTR_SYNC,
};

#ifdef __cplusplus
}
#endif
