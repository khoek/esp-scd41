#ifndef __LIB_SCD41_H
#define __LIB_SCD41_H

#include <driver/i2c.h>
#include <libi2c.h>

typedef struct scd41_cmd_read scd41_cmd_read_t;
typedef struct scd41_cmd_write scd41_cmd_write_t;
typedef struct scd41_cmd_send scd41_cmd_send_t;
typedef struct scd41_cmd_send_and_fetch_result scd41_cmd_send_and_fetch_result_t;

extern scd41_cmd_send_t SCD41_CMD_START_PERIODIC_MEASUREMENT;
extern scd41_cmd_read_t SCD41_CMD_READ_MEASUREMENT;
extern scd41_cmd_send_t SCD41_CMD_STOP_PERIODIC_MEASUREMENT;

extern scd41_cmd_write_t SCD41_CMD_SET_TEMPERATURE_OFFSET;
extern scd41_cmd_read_t SCD41_CMD_GET_TEMPERATURE_OFFSET;
extern scd41_cmd_write_t SCD41_CMD_SET_SENSOR_ALTITUDE;
extern scd41_cmd_read_t SCD41_CMD_GET_SENSOR_ALTITUDE;
extern scd41_cmd_write_t SCD41_CMD_SET_AMBIENT_PRESSURE;

extern scd41_cmd_send_and_fetch_result_t SCD41_CMD_PERFORMED_FORCED_RECALIBRATION;
extern scd41_cmd_write_t SCD41_CMD_SET_AUTOMATIC_SELF_CALIBRATION_ENABLED;
extern scd41_cmd_read_t SCD41_CMD_GET_AUTOMATIC_SELF_CALIBRATION_ENABLED;

extern scd41_cmd_send_t SCD41_CMD_START_LOW_POWER_PERIODIC_MEASUREMENT;
extern scd41_cmd_read_t SCD41_CMD_GET_DATA_READY_STATUS;

extern scd41_cmd_send_t SCD41_CMD_PERSIST_SETTINGS;
extern scd41_cmd_read_t SCD41_CMD_GET_SERIAL_NUMBER;
extern scd41_cmd_read_t SCD41_CMD_PERFORM_SELF_TEST;
extern scd41_cmd_send_t SCD41_CMD_PERFORM_FACTORY_RESET;
extern scd41_cmd_send_t SCD41_CMD_REINIT;

extern scd41_cmd_send_t SCD41_CMD_MEASURE_SINGLE_SHOT;
extern scd41_cmd_send_t SCD41_CMD_MEASURE_SINGLE_SHOT_RHT_ONLY;

#define MASK_SCD41_GET_DATA_READY_STATUS_READY 0x07FF

#define SCD41_SET_AUTOMATIC_SELF_CALIBRATION_ENABLED_FALSE 0x0000
#define SCD41_SET_AUTOMATIC_SELF_CALIBRATION_ENABLED_TRUE 0x0001

typedef struct scd41 scd41_t;
typedef scd41_t* scd41_handle_t;

// Register the SCD41 on the given I2C bus.
esp_err_t scd41_init(i2c_port_t port, uint8_t addr, scd41_handle_t* out_dev);

// Release the given handle.
void scd41_destroy(scd41_handle_t dev);

// Reset the device.
void scd41_reset(scd41_handle_t dev);

// Perform a command over I2C. Use of these functions is thread-safe.
esp_err_t scd41_cmd_send(scd41_handle_t dev, scd41_cmd_send_t* cmd);
esp_err_t scd41_cmd_send_and_fetch_result(scd41_handle_t dev, scd41_cmd_send_and_fetch_result_t* cmd,
                                          const uint16_t* out_data, size_t out_count,
                                          uint16_t* in_data, size_t in_count);
esp_err_t scd41_cmd_read(scd41_handle_t dev, scd41_cmd_read_t* cmd, uint16_t* data, size_t data_count);
esp_err_t scd41_cmd_write(scd41_handle_t dev, scd41_cmd_write_t* cmd, const uint16_t* out_data, size_t out_count);

// Decode temperature values returned by the SCD41.
void scd41_decode_temp(uint32_t raw_temp, double* temp_c);

// Decode humidity values returned by the SCD41.
void scd41_decode_hum(uint32_t raw_hum, double* rel_humidity);

#endif
