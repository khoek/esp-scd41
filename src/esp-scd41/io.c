#include <driver/i2c.h>
#include <esp_log.h>
#include <libcrc.h>
#include <libi2c.h>
#include <math.h>

#include "private.h"

static const char* TAG = "scd41";

scd41_cmd_send_t SCD41_CMD_START_PERIODIC_MEASUREMENT = {.def = {.code = 0x21B1, .delay_ms = 0}};
scd41_cmd_read_t SCD41_CMD_READ_MEASUREMENT = {.def = {.code = 0xEC05, .delay_ms = 1}};
scd41_cmd_send_t SCD41_CMD_STOP_PERIODIC_MEASUREMENT = {.def = {.code = 0x3F86, .delay_ms = 500}};

scd41_cmd_write_t SCD41_CMD_SET_TEMPERATURE_OFFSET = {.def = {.code = 0x241D, .delay_ms = 1}};
scd41_cmd_read_t SCD41_CMD_GET_TEMPERATURE_OFFSET = {.def = {.code = 0x2318, .delay_ms = 1}};
scd41_cmd_write_t SCD41_CMD_SET_SENSOR_ALTITUDE = {.def = {.code = 0x2427, .delay_ms = 1}};
scd41_cmd_read_t SCD41_CMD_GET_SENSOR_ALTITUDE = {.def = {.code = 0x2322, .delay_ms = 1}};
scd41_cmd_write_t SCD41_CMD_SET_AMBIENT_PRESSURE = {.def = {.code = 0xE000, .delay_ms = 1}};

scd41_cmd_send_and_fetch_result_t SCD41_CMD_PERFORMED_FORCED_RECALIBRATION = {.def = {.code = 0x362F, .delay_ms = 400}};
scd41_cmd_write_t SCD41_CMD_SET_AUTOMATIC_SELF_CALIBRATION_ENABLED = {.def = {.code = 0x2416, .delay_ms = 1}};
scd41_cmd_read_t SCD41_CMD_GET_AUTOMATIC_SELF_CALIBRATION_ENABLED = {.def = {.code = 0x2313, .delay_ms = 1}};

scd41_cmd_send_t SCD41_CMD_START_LOW_POWER_PERIODIC_MEASUREMENT = {.def = {.code = 0x21AC, .delay_ms = 0}};
scd41_cmd_read_t SCD41_CMD_GET_DATA_READY_STATUS = {.def = {.code = 0xE4B8, .delay_ms = 1}};

scd41_cmd_send_t SCD41_CMD_PERSIST_SETTINGS = {.def = {.code = 0x3615, .delay_ms = 800}};
scd41_cmd_read_t SCD41_CMD_GET_SERIAL_NUMBER = {.def = {.code = 0x3682, .delay_ms = 1}};
scd41_cmd_read_t SCD41_CMD_PERFORM_SELF_TEST = {.def = {.code = 0x3639, .delay_ms = 10000}};
scd41_cmd_send_t SCD41_CMD_PERFORM_FACTORY_RESET = {.def = {.code = 0x3632, .delay_ms = 1200}};
scd41_cmd_send_t SCD41_CMD_REINIT = {.def = {.code = 0x3646, .delay_ms = 20}};

scd41_cmd_send_t SCD41_CMD_MEASURE_SINGLE_SHOT = {.def = {.code = 0x219D, .delay_ms = 5000}};
scd41_cmd_send_t SCD41_CMD_MEASURE_SINGLE_SHOT_RHT_ONLY = {.def = {.code = 0x2196, .delay_ms = 50}};

esp_err_t scd41_init(i2c_port_t port, uint8_t addr, scd41_handle_t* out_dev) {
    esp_err_t ret;

    scd41_handle_t dev = malloc(sizeof(scd41_t));
    i2c_7bit_init(port, addr, &dev->i2c);
    dev->lock = xSemaphoreCreateMutex();

    // As per spec start up time is 1000ms.
    vTaskDelay(1 + (1000 / portTICK_PERIOD_MS));

    // If the sensor is currently in period measurement mode then it will NAK the
    // CMD_GET_SERIAL_NUMBER, and so we must issue a stop command first (this does
    // no harm if the sensor is not currently performing periodic measurement).
    ret = scd41_cmd_send(dev, &SCD41_CMD_STOP_PERIODIC_MEASUREMENT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C read failed (0x%X), are I2C pin numbers/address correct?", ret);
        goto scd41_init_fail;
    }

    uint16_t serial[3];
    ESP_ERROR_CHECK(scd41_cmd_read(dev, &SCD41_CMD_GET_SERIAL_NUMBER, serial, 3));
    ESP_LOGD(TAG, "serial=0x%04X%04X%04X", serial[0], serial[1], serial[2]);

    scd41_reset(dev);

    *out_dev = dev;
    return ESP_OK;

scd41_init_fail:
    scd41_destroy(dev);
    return ret;
}

void scd41_destroy(scd41_handle_t dev) {
    i2c_7bit_destroy(dev->i2c);
    vSemaphoreDelete(dev->lock);
    free(dev);
}

void scd41_reset(scd41_handle_t dev) {
    // As per spec: "Before sending the reinit command, the stop measurement
    //               command must be issued."
    scd41_cmd_send(dev, &SCD41_CMD_STOP_PERIODIC_MEASUREMENT);
    scd41_cmd_send(dev, &SCD41_CMD_REINIT);

    // As per spec start up time is 1000ms.
    vTaskDelay(1 + (1000 / portTICK_PERIOD_MS));
}

static __always_inline void convert_1b16_to_2b8(uint8_t v[2], uint16_t o) {
    v[0] = (o & 0xFF00) >> 8;
    v[1] = (o & 0x00FF) >> 0;
}

static __always_inline void convert_2b8_to_1b16(uint16_t* v, const uint8_t o[2]) {
    *v = (((uint16_t) o[0]) << 8) | (((uint16_t) o[1]) << 0);
}

static inline esp_err_t cmd_transact_out(scd41_handle_t dev, uint16_t code, const uint16_t* out_data, size_t out_count) {
    esp_err_t ret;

    uint8_t reg[2];
    convert_1b16_to_2b8(reg, code);

    // First decompose the 16-bit words to send as pairs of bytes plus a CRC.
    uint8_t bytes[3 * out_count];
    for (size_t i = 0; i < out_count; i++) {
        convert_1b16_to_2b8(bytes + (3 * i), out_data[i]);
        bytes[(3 * i) + 2] = crc8_calc_sensirion(bytes + (3 * i), 2);
    }

    // Then write 3 byte chunks representing 16 bits (MSB, LSB, CRC) over I2C.
    ret = i2c_7bit_reg_write(dev->i2c, reg, 2, bytes, 3 * out_count);
    if (ret != ESP_OK) {
        return ret;
    }

    return ESP_OK;
}

static inline esp_err_t cmd_transact_in(scd41_handle_t dev, uint16_t* in_data, size_t in_count) {
    esp_err_t ret;

    // First read 3 byte chunks representing 16 bits (MSB, LSB, CRC) over I2C.
    uint8_t bytes[3 * in_count];
    ret = i2c_7bit_reg_read(dev->i2c, NULL, 0, bytes, 3 * in_count);
    if (ret != ESP_OK) {
        return ret;
    }

    // Then assemble the response bytes into 16-bit words and check their CRCs.
    for (size_t i = 0; i < in_count; i++) {
        convert_2b8_to_1b16(in_data + i, bytes + (3 * i));
        uint8_t crc8 = crc8_calc_sensirion(bytes + (3 * i), 2);

        if (bytes[(3 * i) + 2] != crc8) {
            ESP_LOGE(TAG, "crc8 error: read=%04X, crc=%02X vs %02X", in_data[i], bytes[(3 * i) + 2], crc8_calc_sensirion(bytes + (3 * 2), 2));
            return ESP_FAIL;
        }
    }

    return ESP_OK;
}

static esp_err_t cmd_transact(scd41_handle_t dev, scd41_cmd_def_t* def,
                              const uint16_t* out_data, size_t out_count, uint16_t* in_data, size_t in_count) {
    esp_err_t ret;

    while (xSemaphoreTake(dev->lock, portMAX_DELAY) != pdTRUE)
        ;

    // First transmit the command code/address, and any payload data which is supposed
    // to be sent after it.
    ret = cmd_transact_out(dev, def->code, out_data, out_count);
    if (ret != ESP_OK) {
        goto cmd_transact_out;
    }

    // Wait the datasheet-specified duration.
    vTaskDelay(1 + (def->delay_ms / portTICK_PERIOD_MS));

    // If there is no data to recieve, we are now done.
    if (!in_count) {
        goto cmd_transact_out;
    }

    // Otherwise, recieve the data which we are supposed to get back,
    // noting that we have now waited the neccesary `delay_ms`.
    ret = cmd_transact_in(dev, in_data, in_count);
    if (ret != ESP_OK) {
        goto cmd_transact_out;
    }

cmd_transact_out:
    xSemaphoreGive(dev->lock);
    return ret;
}

esp_err_t scd41_cmd_send(scd41_handle_t dev, scd41_cmd_send_t* cmd) {
    return cmd_transact(dev, &cmd->def, NULL, 0, NULL, 0);
}

esp_err_t scd41_cmd_send_and_fetch_result(scd41_handle_t dev, scd41_cmd_send_and_fetch_result_t* cmd,
                                          const uint16_t* out_data, size_t out_count,
                                          uint16_t* in_data, size_t in_count) {
    return cmd_transact(dev, &cmd->def, out_data, out_count, in_data, in_count);
}

esp_err_t scd41_cmd_read(scd41_handle_t dev, scd41_cmd_read_t* cmd, uint16_t* in_data, size_t in_count) {
    return cmd_transact(dev, &cmd->def, NULL, 0, in_data, in_count);
}

esp_err_t scd41_cmd_write(scd41_handle_t dev, scd41_cmd_write_t* cmd, const uint16_t* out_data, size_t out_count) {
    return cmd_transact(dev, &cmd->def, out_data, out_count, NULL, 0);
}

void scd41_decode_temp(uint32_t raw_temp, double* temp_c) {
    *temp_c = -45.0 + (175 * ((double) raw_temp) / 65536.0);
}

void scd41_decode_hum(uint32_t raw_hum, double* rel_humidity) {
    *rel_humidity = 100 * ((double) raw_hum) / 65536.0;
}
