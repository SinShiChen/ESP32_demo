#include <string.h>
#include <stdio.h>
#include "sdkconfig.h"
#include "esp_types.h"
#include "esp_log.h"
#include "esp_check.h"
#include "driver/i2c_master.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2c_ICM20602.h"

#define I2C_EEPROM_MAX_TRANS_UNIT (48)
static const char TAG[] = "i2c-icm";

esp_err_t i2c_icm20602_init(i2c_master_bus_handle_t bus_handle, const i2c_icm_config_t *icm_config, i2c_icm_handle_t *icm_handle)
{
    esp_err_t ret = ESP_OK;
    i2c_icm_handle_t out_handle;
    out_handle = (i2c_icm_handle_t)calloc(1, sizeof(i2c_icm_handle_t));
    ESP_GOTO_ON_FALSE(out_handle, ESP_ERR_NO_MEM, err, TAG, "no memory for i2c eeprom device");

    i2c_device_config_t i2c_dev_conf = {
        .scl_speed_hz = icm_config->icm_device.scl_speed_hz,
        .device_address = icm_config->icm_device.device_address,
    };

    if (out_handle->i2c_dev == NULL) {
        ESP_GOTO_ON_ERROR(i2c_master_bus_add_device(bus_handle, &i2c_dev_conf, &out_handle->i2c_dev), err, TAG, "i2c new bus failed");
    }

    out_handle->buffer = (uint8_t*)calloc(1,  I2C_EEPROM_MAX_TRANS_UNIT);
    ESP_GOTO_ON_FALSE(out_handle->buffer, ESP_ERR_NO_MEM, err, TAG, "no memory for i2c eeprom device buffer");

    out_handle->write_time_ms = icm_config->write_time_ms;
    *icm_handle = out_handle;

    return ESP_OK;

    err:
    if (out_handle && out_handle->i2c_dev) {
        i2c_master_bus_rm_device(out_handle->i2c_dev);
    }
    free(out_handle);
    return ret;
}

esp_err_t i2c_icm20602_init_reg(i2c_icm_handle_t icm_handle)
{
    printf("i2c_icm20602_init_reg");
    uint8_t reg[] = {0x6B,0x19,0x6B,0X1A,0X1B,0X1C};
    uint8_t val[] = {0x80,0x02,0x03,0x03,0x10,0x09};
    int times = sizeof(reg)/sizeof(uint8_t);
    // for (int i = 0; i < times; i++)
    // {

    //     icm_handle->buffer[0] = reg[i];
    //     icm_handle->buffer[1] = val[i];

    //     ESP_ERROR_CHECK(i2c_master_transmit(icm_handle->i2c_dev, icm_handle->buffer, 2, -1));
    //     if (i == 0)
    //     {
    //         vTaskDelay(50);
    //     }
    // }


    uint8_t data[5];
    uint8_t size = 1;
    icm_handle->buffer[0] = 0x1e;
    i2c_master_transmit_receive(icm_handle->i2c_dev, icm_handle->buffer, 1, data, size, -1);
    printf("read_data = %#x\r\n",data[0]);
    vTaskDelay(50);
    icm_handle->buffer[0] = 0x1e;
    icm_handle->buffer[1] = 0x34;
    ESP_ERROR_CHECK(i2c_master_transmit(icm_handle->i2c_dev, icm_handle->buffer, 2, -1));
    vTaskDelay(50);
    icm_handle->buffer[0] = 0x1e;
    i2c_master_transmit_receive(icm_handle->i2c_dev, icm_handle->buffer, 1, data, size, -1);
    printf("read_data = %#x\r\n",data[0]);
    return ESP_OK;

}


esp_err_t i2c_icm20602_read(i2c_icm_handle_t icm_handle)
{
    printf("i2c_icm20602_read");
    return ESP_OK;
}
