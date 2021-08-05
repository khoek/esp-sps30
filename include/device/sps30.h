#pragma once

#include <device/sensirion.h>
#include <driver/i2c.h>
#include <sys/cdefs.h>

extern const uint16_t SPS30_PRODUCT_TYPE_DRIVER_SUPPORTED[4];

typedef struct sps30_cmd_exec sps30_cmd_exec_t;
typedef struct sps30_cmd_read sps30_cmd_read_t;
typedef struct sps30_cmd_write sps30_cmd_write_t;
typedef struct sps30_cmd_readwrite sps30_cmd_readwrite_t;

extern const sps30_cmd_write_t SPS30_CMD_START_MEASUREMENT;
extern const sps30_cmd_exec_t SPS30_CMD_STOP_MEASUREMENT;

extern const sps30_cmd_read_t SPS30_CMD_READ_DATAREADY_FLAG;
extern const sps30_cmd_read_t SPS30_CMD_READ_MEASURED_VALUES;

extern const sps30_cmd_exec_t SPS30_CMD_SLEEP;
extern const sps30_cmd_exec_t SPS30_CMD_WAKEUP;
extern const sps30_cmd_exec_t SPS30_CMD_START_FAN_CLEANING;

// WARNING: For FW Version < 2.2 after writing a new interval, this will be
// activated immediately. However, if the interval register is read out after
// setting the new value, the previous value is returned until the next
// start/reset of the sensor module.
extern const sps30_cmd_readwrite_t SPS30_CMD_READ_WRITE_AUTO_CLEANING_INTERVAL;

extern const sps30_cmd_read_t SPS30_CMD_READ_PRODUCT_TYPE;
extern const sps30_cmd_read_t SPS30_CMD_READ_SERIAL_NUMBER;
extern const sps30_cmd_read_t SPS30_CMD_READ_VERSION;
extern const sps30_cmd_read_t SPS30_CMD_READ_DEVICE_STATUS_REGISTER;
extern const sps30_cmd_exec_t SPS30_CMD_CLEAR_DEVICE_STATUS_REGISTER;
extern const sps30_cmd_exec_t SPS30_CMD_RESET;

#define SPS30_START_MEASUREMENT_OUTPUT_32BIT_FLOAT 0x0300
#define SPS30_START_MEASUREMENT_OUTPUT_16BIT_INT 0x0500

#define SPS30_DEVICE_STATUS_REGISTER_WORD0_LASER (1ULL << 5)
#define SPS30_DEVICE_STATUS_REGISTER_WORD0_FAN (1ULL << 4)
#define MASK_SPS30_DEVICE_STATUS_REGISTER_WORD0 0x30

#define SPS30_DEVICE_STATUS_REGISTER_WORD1_SPEED (1ULL << 5)
#define MASK_SPS30_DEVICE_STATUS_REGISTER_WORD1 0x20

#define SPS30_DATAREADY_FLAG_ENABLED 0x0001

typedef sensirion_dev_handle_t sps30_handle_t;

typedef struct sps30_measured_values_32bit_float {
    float mass_conc_pm1_0_ug_per_m3;
    float mass_conc_pm2_5_ug_per_m3;
    float mass_conc_pm4_0_ug_per_m3;
    float mass_conc_pm10_ug_per_m3;

    float numb_conc_pm0_5_per_cm3;
    float numb_conc_pm1_0_per_cm3;
    float numb_conc_pm2_5_per_cm3;
    float numb_conc_pm4_0_per_cm3;
    float numb_conc_pm10_per_cm3;

    float typical_particle_size_um;
} sps30_measured_values_32bit_float_t;

// Register the SPS30 on the given I2C bus.
__result_use_check esp_err_t sps30_init(i2c_port_t port, uint8_t addr,
                                        sps30_handle_t* out_dev);

// Release the given handle.
void sps30_destroy(sps30_handle_t dev);

// Reset the device.
__result_use_check esp_err_t sps30_reset(sps30_handle_t dev);

// Perform a command over I2C. Use of these functions is thread-safe.
__result_use_check esp_err_t sps30_cmd_exec(sps30_handle_t dev,
                                            const sps30_cmd_exec_t* cmd);
__result_use_check esp_err_t sps30_cmd_read(sps30_handle_t dev,
                                            const sps30_cmd_read_t* cmd,
                                            uint16_t* in_data, size_t in_count);
__result_use_check esp_err_t sps30_cmd_write(sps30_handle_t dev,
                                             const sps30_cmd_write_t* cmd,
                                             const uint16_t* out_data,
                                             size_t out_count);
__result_use_check esp_err_t
sps30_cmd_readwrite(sps30_handle_t dev, const sps30_cmd_readwrite_t* cmd,
                    const uint16_t* out_data, size_t out_count,
                    uint16_t* in_data, size_t in_count);

__result_use_check esp_err_t sps30_cmd_read_measured_values_32bit_float(
    sps30_handle_t dev, sps30_measured_values_32bit_float_t* values);
