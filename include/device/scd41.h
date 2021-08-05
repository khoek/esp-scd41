#pragma once

#include <device/sensirion.h>
#include <driver/i2c.h>

typedef struct scd41_cmd_exec scd41_cmd_exec_t;
typedef struct scd41_cmd_read scd41_cmd_read_t;
typedef struct scd41_cmd_write scd41_cmd_write_t;
typedef struct scd41_cmd_readwrite scd41_cmd_readwrite_t;

extern const scd41_cmd_exec_t SCD41_CMD_START_PERIODIC_MEASUREMENT;
extern const scd41_cmd_read_t SCD41_CMD_READ_MEASUREMENT;
extern const scd41_cmd_exec_t SCD41_CMD_STOP_PERIODIC_MEASUREMENT;

extern const scd41_cmd_write_t SCD41_CMD_SET_TEMPERATURE_OFFSET;
extern const scd41_cmd_read_t SCD41_CMD_GET_TEMPERATURE_OFFSET;
extern const scd41_cmd_write_t SCD41_CMD_SET_SENSOR_ALTITUDE;
extern const scd41_cmd_read_t SCD41_CMD_GET_SENSOR_ALTITUDE;
extern const scd41_cmd_write_t SCD41_CMD_SET_AMBIENT_PRESSURE;

extern const scd41_cmd_readwrite_t SCD41_CMD_PERFORMED_FORCED_RECALIBRATION;
extern const scd41_cmd_write_t SCD41_CMD_SET_AUTOMATIC_SELF_CALIBRATION_ENABLED;
extern const scd41_cmd_read_t SCD41_CMD_GET_AUTOMATIC_SELF_CALIBRATION_ENABLED;

extern const scd41_cmd_exec_t SCD41_CMD_START_LOW_POWER_PERIODIC_MEASUREMENT;
extern const scd41_cmd_read_t SCD41_CMD_GET_DATA_READY_STATUS;

extern const scd41_cmd_exec_t SCD41_CMD_PERSIST_SETTINGS;
extern const scd41_cmd_read_t SCD41_CMD_GET_SERIAL_NUMBER;
extern const scd41_cmd_read_t SCD41_CMD_PERFORM_SELF_TEST;
extern const scd41_cmd_exec_t SCD41_CMD_PERFORM_FACTORY_RESET;
extern const scd41_cmd_exec_t SCD41_CMD_REINIT;

extern const scd41_cmd_exec_t SCD41_CMD_MEASURE_SINGLE_SHOT;
extern const scd41_cmd_exec_t SCD41_CMD_MEASURE_SINGLE_SHOT_RHT_ONLY;

#define MASK_SCD41_GET_DATA_READY_STATUS_READY 0x07FF

#define SCD41_SET_AUTOMATIC_SELF_CALIBRATION_ENABLED_FALSE 0x0000
#define SCD41_SET_AUTOMATIC_SELF_CALIBRATION_ENABLED_TRUE 0x0001

typedef sensirion_dev_handle_t scd41_handle_t;

// Register the SCD41 on the given I2C bus.
__result_use_check esp_err_t scd41_init(i2c_port_t port, uint8_t addr,
                                        scd41_handle_t* out_dev);

// Release the given handle.
void scd41_destroy(scd41_handle_t dev);

// Reset the device.
__result_use_check esp_err_t scd41_reset(scd41_handle_t dev);

// Perform a command over I2C. Use of these functions is thread-safe.
__result_use_check esp_err_t scd41_cmd_exec(scd41_handle_t dev,
                                            const scd41_cmd_exec_t* cmd);
__result_use_check esp_err_t scd41_cmd_read(scd41_handle_t dev,
                                            const scd41_cmd_read_t* cmd,
                                            uint16_t* in_data, size_t in_count);
__result_use_check esp_err_t scd41_cmd_write(scd41_handle_t dev,
                                             const scd41_cmd_write_t* cmd,
                                             const uint16_t* out_data,
                                             size_t out_count);
__result_use_check esp_err_t
scd41_cmd_readwrite(scd41_handle_t dev, const scd41_cmd_readwrite_t* cmd,
                    const uint16_t* out_data, size_t out_count,
                    uint16_t* in_data, size_t in_count);

// Decode temperature values returned by the SCD41.
void scd41_decode_temp(uint32_t raw_temp, double* temp_c);

// Decode humidity values returned by the SCD41.
void scd41_decode_hum(uint32_t raw_hum, double* rel_humidity);
