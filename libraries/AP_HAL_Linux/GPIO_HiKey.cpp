#include <AP_Common/AP_Common.h>

#include "GPIO_HiKey.h"

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_YATAGARASU

const unsigned Linux::GPIO_Sysfs::pin_table[] = {
    [HIKEY_GPIO_A] =    488,
    [HIKEY_GPIO_B] =    489,
    [HIKEY_GPIO_C] =    490,
    [HIKEY_GPIO_D] =    491,
    [HIKEY_GPIO_E] =    492,
    [HIKEY_GPIO_F] =    415,
    [HIKEY_GPIO_G] =    463,
    [HIKEY_GPIO_H] =    495,
    [HIKEY_GPIO_I] =    426,
    [HIKEY_GPIO_J] =    433,
    [HIKEY_GPIO_K] =    427,
    [HIKEY_GPIO_L] =    434,
};

const uint8_t Linux::GPIO_Sysfs::n_pins = _HIKEY_GPIO_MAX;

static_assert(ARRAY_SIZE(Linux::GPIO_Sysfs::pin_table) == _HIKEY_GPIO_MAX,
              "GPIO pin_table must have the same size of entries in enum gpio_hikey");

#endif
