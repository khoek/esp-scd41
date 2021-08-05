#include <device/sensirion.h>
#include <driver/i2c.h>
#include <esp_log.h>
#include <libesp.h>
#include <math.h>

#include "device/scd41.h"

static const char* TAG = "scd41";

struct scd41_cmd_exec {
    sensirion_cmd_def_t def;
};

struct scd41_cmd_read {
    sensirion_cmd_def_t def;
};

struct scd41_cmd_write {
    sensirion_cmd_def_t def;
};

struct scd41_cmd_readwrite {
    sensirion_cmd_def_t def;
};

const scd41_cmd_exec_t SCD41_CMD_START_PERIODIC_MEASUREMENT = {
    .def = {.code = 0x21B1, .delay_ms = 0}};
const scd41_cmd_read_t SCD41_CMD_READ_MEASUREMENT = {
    .def = {.code = 0xEC05, .delay_ms = 1}};
const scd41_cmd_exec_t SCD41_CMD_STOP_PERIODIC_MEASUREMENT = {
    .def = {.code = 0x3F86, .delay_ms = 500}};

const scd41_cmd_write_t SCD41_CMD_SET_TEMPERATURE_OFFSET = {
    .def = {.code = 0x241D, .delay_ms = 1}};
const scd41_cmd_read_t SCD41_CMD_GET_TEMPERATURE_OFFSET = {
    .def = {.code = 0x2318, .delay_ms = 1}};
const scd41_cmd_write_t SCD41_CMD_SET_SENSOR_ALTITUDE = {
    .def = {.code = 0x2427, .delay_ms = 1}};
const scd41_cmd_read_t SCD41_CMD_GET_SENSOR_ALTITUDE = {
    .def = {.code = 0x2322, .delay_ms = 1}};
const scd41_cmd_write_t SCD41_CMD_SET_AMBIENT_PRESSURE = {
    .def = {.code = 0xE000, .delay_ms = 1}};

const scd41_cmd_readwrite_t SCD41_CMD_PERFORMED_FORCED_RECALIBRATION = {
    .def = {.code = 0x362F, .delay_ms = 400}};
const scd41_cmd_write_t SCD41_CMD_SET_AUTOMATIC_SELF_CALIBRATION_ENABLED = {
    .def = {.code = 0x2416, .delay_ms = 1}};
const scd41_cmd_read_t SCD41_CMD_GET_AUTOMATIC_SELF_CALIBRATION_ENABLED = {
    .def = {.code = 0x2313, .delay_ms = 1}};

const scd41_cmd_exec_t SCD41_CMD_START_LOW_POWER_PERIODIC_MEASUREMENT = {
    .def = {.code = 0x21AC, .delay_ms = 0}};
const scd41_cmd_read_t SCD41_CMD_GET_DATA_READY_STATUS = {
    .def = {.code = 0xE4B8, .delay_ms = 1}};

const scd41_cmd_exec_t SCD41_CMD_PERSIST_SETTINGS = {
    .def = {.code = 0x3615, .delay_ms = 800}};
const scd41_cmd_read_t SCD41_CMD_GET_SERIAL_NUMBER = {
    .def = {.code = 0x3682, .delay_ms = 1}};
const scd41_cmd_read_t SCD41_CMD_PERFORM_SELF_TEST = {
    .def = {.code = 0x3639, .delay_ms = 10000}};
const scd41_cmd_exec_t SCD41_CMD_PERFORM_FACTORY_RESET = {
    .def = {.code = 0x3632, .delay_ms = 1200}};
const scd41_cmd_exec_t SCD41_CMD_REINIT = {
    .def = {.code = 0x3646, .delay_ms = 20}};

const scd41_cmd_exec_t SCD41_CMD_MEASURE_SINGLE_SHOT = {
    .def = {.code = 0x219D, .delay_ms = 5000}};
const scd41_cmd_exec_t SCD41_CMD_MEASURE_SINGLE_SHOT_RHT_ONLY = {
    .def = {.code = 0x2196, .delay_ms = 50}};

esp_err_t scd41_init(i2c_port_t port, uint8_t addr, scd41_handle_t* out_dev) {
    esp_err_t ret;

    scd41_handle_t dev;
    sensirion_init(port, addr, &dev);

    // As per spec start up time is 1000ms.
    vTaskDelay(1 + (1000 / portTICK_PERIOD_MS));

    // If the sensor is currently in period measurement mode then it will NAK
    // the CMD_GET_SERIAL_NUMBER, and so we must issue a stop command first
    // (this does no harm if the sensor is not currently performing periodic
    // measurement).
    ret = scd41_cmd_exec(dev, &SCD41_CMD_STOP_PERIODIC_MEASUREMENT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG,
                 "I2C read failed (0x%X), are I2C pin numbers/address correct?",
                 ret);
        goto scd41_init_fail;
    }

    uint16_t serial[3];
    ESP_ERROR_CHECK(
        scd41_cmd_read(dev, &SCD41_CMD_GET_SERIAL_NUMBER, serial, 3));
    ESP_LOGD(TAG, "serial={0x%04X,0x%04X,0x%04X}", serial[0], serial[1],
             serial[2]);

    ret = scd41_reset(dev);
    if (ret != ESP_OK) {
        goto scd41_init_fail;
    }

    *out_dev = dev;
    return ESP_OK;

scd41_init_fail:
    scd41_destroy(dev);
    return ret;
}

void scd41_destroy(scd41_handle_t dev) {
    ESP_ERROR_DISCARD(
        scd41_cmd_exec(dev, &SCD41_CMD_STOP_PERIODIC_MEASUREMENT));
    ESP_ERROR_DISCARD(scd41_cmd_exec(dev, &SCD41_CMD_REINIT));
    sensirion_destroy(dev);
}

esp_err_t scd41_reset(scd41_handle_t dev) {
    esp_err_t ret;

    // As per spec: "Before sending the reinit command, the stop measurement
    //               command must be issued."
    ret = scd41_cmd_exec(dev, &SCD41_CMD_STOP_PERIODIC_MEASUREMENT);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = scd41_cmd_exec(dev, &SCD41_CMD_REINIT);
    if (ret != ESP_OK) {
        return ret;
    }

    // As per spec start up time is 1000ms.
    vTaskDelay(1 + (1000 / portTICK_PERIOD_MS));

    return ESP_OK;
}

esp_err_t scd41_cmd_exec(scd41_handle_t dev, const scd41_cmd_exec_t* cmd) {
    return sensirion_cmd_perform(dev, &cmd->def, NULL, 0, NULL, 0);
}

esp_err_t scd41_cmd_read(scd41_handle_t dev, const scd41_cmd_read_t* cmd,
                         uint16_t* in_data, size_t in_count) {
    return sensirion_cmd_perform(dev, &cmd->def, NULL, 0, in_data, in_count);
}

esp_err_t scd41_cmd_write(scd41_handle_t dev, const scd41_cmd_write_t* cmd,
                          const uint16_t* out_data, size_t out_count) {
    return sensirion_cmd_perform(dev, &cmd->def, out_data, out_count, NULL, 0);
}

esp_err_t scd41_cmd_readwrite(scd41_handle_t dev,
                              const scd41_cmd_readwrite_t* cmd,
                              const uint16_t* out_data, size_t out_count,
                              uint16_t* in_data, size_t in_count) {
    return sensirion_cmd_perform(dev, &cmd->def, out_data, out_count, in_data,
                                 in_count);
}

void scd41_decode_temp(uint32_t raw_temp, double* temp_c) {
    *temp_c = -45.0 + (175 * ((double) raw_temp) / 65536.0);
}

void scd41_decode_hum(uint32_t raw_hum, double* rel_humidity) {
    *rel_humidity = 100 * ((double) raw_hum) / 65536.0;
}
