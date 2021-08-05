#include <device/sensirion.h>
#include <driver/i2c.h>
#include <esp_log.h>
#include <libesp.h>
#include <libesp/marshall.h>
#include <string.h>

#include "device/sps30.h"

static const char* TAG = "sps30";

// This is the ASCII string "00080000", as per spec.
const uint16_t SPS30_PRODUCT_TYPE_DRIVER_SUPPORTED[4] = {0x3030, 0x3038, 0x3030,
                                                         0x3030};

struct sps30_cmd_exec {
    sensirion_cmd_def_t def;
};

struct sps30_cmd_read {
    sensirion_cmd_def_t def;
};

struct sps30_cmd_write {
    sensirion_cmd_def_t def;
};

struct sps30_cmd_readwrite {
    sensirion_cmd_def_t def;
};

const sps30_cmd_write_t SPS30_CMD_START_MEASUREMENT = {
    .def = {.code = 0x0010, .delay_ms = 20}};
const sps30_cmd_exec_t SPS30_CMD_STOP_MEASUREMENT = {
    .def = {.code = 0x0104, .delay_ms = 20}};

const sps30_cmd_read_t SPS30_CMD_READ_DATAREADY_FLAG = {
    .def = {.code = 0x0202, .delay_ms = 0}};
const sps30_cmd_read_t SPS30_CMD_READ_MEASURED_VALUES = {
    .def = {.code = 0x0300, .delay_ms = 0}};

const sps30_cmd_exec_t SPS30_CMD_SLEEP = {
    .def = {.code = 0x1001, .delay_ms = 5}};
const sps30_cmd_exec_t SPS30_CMD_WAKEUP = {
    .def = {.code = 0x1103, .delay_ms = 5}};
const sps30_cmd_exec_t SPS30_CMD_START_FAN_CLEANING = {
    .def = {.code = 0x5607, .delay_ms = 5}};

// WARNING: For FW Version < 2.2 after writing a new interval, this will be
// activated immediately. However, if the interval register is read out after
// setting the new value, the previous value is returned until the next
// start/reset of the sensor module.
const sps30_cmd_readwrite_t SPS30_CMD_READ_WRITE_AUTO_CLEANING_INTERVAL = {
    .def = {.code = 0x8004, .delay_ms = 20}};

const sps30_cmd_read_t SPS30_CMD_READ_PRODUCT_TYPE = {
    .def = {.code = 0xD002, .delay_ms = 0}};
const sps30_cmd_read_t SPS30_CMD_READ_SERIAL_NUMBER = {
    .def = {.code = 0xD033, .delay_ms = 0}};
const sps30_cmd_read_t SPS30_CMD_READ_VERSION = {
    .def = {.code = 0xD100, .delay_ms = 0}};
const sps30_cmd_read_t SPS30_CMD_READ_DEVICE_STATUS_REGISTER = {
    .def = {.code = 0xD206, .delay_ms = 0}};
const sps30_cmd_exec_t SPS30_CMD_CLEAR_DEVICE_STATUS_REGISTER = {
    .def = {.code = 0xD210, .delay_ms = 5}};
const sps30_cmd_exec_t SPS30_CMD_RESET = {
    .def = {.code = 0xD304, .delay_ms = 100}};

#define POLLING_INTERVAL_MS 250
#define POLLING_MAX_ITERS (3 * 4)

// Poll the device `dev` for its product type, retrying some number of times if
// we get a failure, in order to allow the device to start up. (Unfortunately,
// the spec does not give a maximum startup time.)
static esp_err_t polling_read_product_type(sps30_handle_t dev,
                                           uint16_t product_type[4]) {
    esp_err_t ret;

    for (size_t i = 0; i < POLLING_MAX_ITERS; i++) {
        ret =
            sps30_cmd_read(dev, &SPS30_CMD_READ_PRODUCT_TYPE, product_type, 4);
        if (ret == ESP_OK) {
            return ESP_OK;
        }
    }

    return ret;
}

esp_err_t sps30_init(i2c_port_t port, uint8_t addr, sps30_handle_t* out_dev) {
    esp_err_t ret;

    sps30_handle_t dev;
    sensirion_init(port, addr, &dev);

    // Two wakeup commands within 100ms are required to bring the device out of
    // sleep mode, if the device was in this mode. We ignore the possible errors
    // generated if the device is not currently in sleep mode.
    ESP_ERROR_DISCARD(sps30_cmd_exec(dev, &SPS30_CMD_WAKEUP));
    ESP_ERROR_DISCARD(sps30_cmd_exec(dev, &SPS30_CMD_WAKEUP));

    // Try to contact the device, retrying (up to a maximum number of times) if
    // we get a failure, in order to allow the device time to start up.
    uint16_t product_type[4];
    ret = polling_read_product_type(dev, product_type);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG,
                 "I2C read failed (0x%X), are I2C pin numbers/address correct?",
                 ret);
        goto sps30_init_fail;
    }

    if (memcmp(product_type, SPS30_PRODUCT_TYPE_DRIVER_SUPPORTED,
               sizeof(SPS30_PRODUCT_TYPE_DRIVER_SUPPORTED))) {
        ESP_LOGE(TAG,
                 "unknown device id (0x%04X,0x%04X,0x%04X,0x%04X), have you "
                 "specified the address of another device?",
                 product_type[0], product_type[1], product_type[2],
                 product_type[3]);
        goto sps30_init_fail;
    }

    uint8_t serial[33];
    ret = sps30_cmd_read(dev, &SPS30_CMD_READ_SERIAL_NUMBER, (uint16_t*) serial,
                         16);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "couldn't read serial number");
        goto sps30_init_fail;
    }
    serial[32] = '\0';
    for (size_t j = 0; j < 16; j++) {
        uint8_t tmp = serial[2 * j];
        serial[2 * j] = serial[(2 * j) + 1];
        serial[(2 * j) + 1] = tmp;
    }

    ESP_LOGD(TAG, "serial=<str>: %s", (char*) serial);
    ESP_LOGD(TAG, "   hex= [00]: %02X %02X %02X %02X %02X %02X %02X %02X",
             serial[0x00], serial[0x01], serial[0x02], serial[0x03],
             serial[0x04], serial[0x05], serial[0x06], serial[0x07]);
    ESP_LOGD(TAG, "        [08]: %02X %02X %02X %02X %02X %02X %02X %02X",
             serial[0x08], serial[0x09], serial[0x0A], serial[0x0B],
             serial[0x0C], serial[0x0D], serial[0x0E], serial[0x0F]);
    ESP_LOGD(TAG, "        [10]: %02X %02X %02X %02X %02X %02X %02X %02X",
             serial[0x10], serial[0x11], serial[0x12], serial[0x13],
             serial[0x14], serial[0x15], serial[0x16], serial[0x17]);
    ESP_LOGD(TAG, "        [18]: %02X %02X %02X %02X %02X %02X %02X %02X",
             serial[0x18], serial[0x19], serial[0x1A], serial[0x1B],
             serial[0x1C], serial[0x1D], serial[0x1E], serial[0x1F]);

    uint16_t version;
    ret = sps30_cmd_read(dev, &SPS30_CMD_READ_VERSION, &version, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "couldn't read version");
        goto sps30_init_fail;
    }

    ESP_LOGD(TAG, "version=0x%04X", version);

    ret = sps30_reset(dev);
    if (ret != ESP_OK) {
        goto sps30_init_fail;
    }

    *out_dev = dev;
    return ESP_OK;

sps30_init_fail:
    sps30_destroy(dev);
    return ret;
}

void sps30_destroy(sps30_handle_t dev) {
    ESP_ERROR_DISCARD(sps30_cmd_exec(dev, &SPS30_CMD_RESET));
    sensirion_destroy(dev);
}

esp_err_t sps30_reset(sps30_handle_t dev) {
    esp_err_t ret;

    // Two wakeup commands within 100ms are required to bring the device out of
    // sleep mode, if the device was in this mode. We ignore the possible errors
    // generated if the device is not currently in sleep mode.
    for (size_t i = 0; i < 2; i++) {
        ret = sps30_cmd_exec(dev, &SPS30_CMD_WAKEUP);
        if (ret != ESP_OK) {
            return ret;
        }
    }

    // Actually reset the device.
    ret = sps30_cmd_exec(dev, &SPS30_CMD_RESET);
    if (ret != ESP_OK) {
        return ret;
    }

    // Try to contact the device, retrying (up to a maximum number of times) if
    // we get a failure, in order to allow the device time to start up. (Spec
    // does not give a maximum startup time.)
    uint16_t product_type[4];
    return polling_read_product_type(dev, product_type);
}

esp_err_t sps30_cmd_exec(sps30_handle_t dev, const sps30_cmd_exec_t* cmd) {
    return sensirion_cmd_perform(dev, &cmd->def, NULL, 0, NULL, 0);
}

esp_err_t sps30_cmd_read(sps30_handle_t dev, const sps30_cmd_read_t* cmd,
                         uint16_t* in_data, size_t in_count) {
    return sensirion_cmd_perform(dev, &cmd->def, NULL, 0, in_data, in_count);
}

esp_err_t sps30_cmd_write(sps30_handle_t dev, const sps30_cmd_write_t* cmd,
                          const uint16_t* out_data, size_t out_count) {
    return sensirion_cmd_perform(dev, &cmd->def, out_data, out_count, NULL, 0);
}

esp_err_t sps30_cmd_readwrite(sps30_handle_t dev,
                              const sps30_cmd_readwrite_t* cmd,
                              const uint16_t* out_data, size_t out_count,
                              uint16_t* in_data, size_t in_count) {
    return sensirion_cmd_perform(dev, &cmd->def, out_data, out_count, in_data,
                                 in_count);
}

esp_err_t sps30_cmd_read_measured_values_32bit_float(
    sps30_handle_t dev, sps30_measured_values_32bit_float_t* values) {
    uint16_t buff[10][2];

    esp_err_t ret =
        sps30_cmd_read(dev, &SPS30_CMD_READ_MEASURED_VALUES, &buff[0][0],
                       sizeof(buff) / sizeof(uint16_t));
    if (ret != ESP_OK) {
        return ret;
    }

    // Need to turn pairs of 16-bit uints into 32-bit floats, big endian.

    marshall_2u16_to_1f32_be(&values->mass_conc_pm1_0_ug_per_m3, buff[0]);
    marshall_2u16_to_1f32_be(&values->mass_conc_pm2_5_ug_per_m3, buff[1]);
    marshall_2u16_to_1f32_be(&values->mass_conc_pm4_0_ug_per_m3, buff[2]);
    marshall_2u16_to_1f32_be(&values->mass_conc_pm10_ug_per_m3, buff[3]);

    marshall_2u16_to_1f32_be(&values->numb_conc_pm0_5_per_cm3, buff[4]);
    marshall_2u16_to_1f32_be(&values->numb_conc_pm1_0_per_cm3, buff[5]);
    marshall_2u16_to_1f32_be(&values->numb_conc_pm2_5_per_cm3, buff[6]);
    marshall_2u16_to_1f32_be(&values->numb_conc_pm4_0_per_cm3, buff[7]);
    marshall_2u16_to_1f32_be(&values->numb_conc_pm10_per_cm3, buff[8]);

    marshall_2u16_to_1f32_be(&values->typical_particle_size_um, buff[9]);

    return ESP_OK;
}
