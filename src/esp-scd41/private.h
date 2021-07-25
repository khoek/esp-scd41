#ifndef __LIB__SCD41_PRIVATE_H
#define __LIB__SCD41_PRIVATE_H

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <libi2c.h>

#include "device/scd41.h"

typedef struct scd41_cmd_def {
    uint16_t code;
    uint32_t delay_ms;
} scd41_cmd_def_t;

struct scd41_cmd_read {
    scd41_cmd_def_t def;
};

struct scd41_cmd_write {
    scd41_cmd_def_t def;
};

struct scd41_cmd_send {
    scd41_cmd_def_t def;
};

struct scd41_cmd_send_and_fetch_result {
    scd41_cmd_def_t def;
};

struct scd41 {
    i2c_7bit_handle_t i2c;

    SemaphoreHandle_t lock;
};

#endif
