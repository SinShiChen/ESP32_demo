/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/i2s_std.h"
#include "driver/i2s_tdm.h"
#include "soc/soc_caps.h"
#include "esp_codec_dev.h"
#include "esp_codec_dev_defaults.h"
#include "driver/i2c_master.h"
#include "unity.h"

#include "PCA9557.h"
#include "sdmount.h"
#include "codec_dev.h"




#define LCD_CS_GPIO                 BIT(0)    // PCA9557_GPIO_NUM_1
#define PA_EN_GPIO                  BIT(1)    // PCA9557_GPIO_NUM_2
#define DVP_PWDN_GPIO               BIT(2)    // PCA9557_GPIO_NUM_3

#define PCA9557_SENSOR_ADDR             0x19        /*!< Slave address of the MPU9250 sensor */
#define SET_BITS(_m, _s, _v)  ((_v) ? (_m)|((_s)) : (_m)&~((_s)))
#define BSP_I2C_FREQ_HZ       100000         // 100kHz


i2c_master_bus_handle_t i2c_bus_handle;


void app_main(void)
{
    // Need install driver (i2c and i2s) firstly
    // int ret = ut_i2c_init(0);
    esp_err_t ret =  bsp_i2c_init(&i2c_bus_handle);
    TEST_ESP_OK(ret);
    
    pca9557_init(i2c_bus_handle,&i2c_pca9557_handle);
    pa_en(&i2c_pca9557_handle,1);
    // Do initialize of related interface: data_if, ctrl_if and gpio_if
    code_dev_init(&i2c_bus_handle);
    
    sdmmc_card_t *sdmmc_card = mount_sdcard();

    // bool not_break = true;
    // static  uint8_t buffer[4096];
    // size_t read_bytes;
    // size_t len =0;
    // while ((read_bytes = fread(buffer, 1, sizeof(buffer), fp)) > 0) {
    //     _audio_player_write_fn(buffer, read_bytes,&len,500);
    // }
    mp3_player_init();

    // esp_err_t err = esp_vfs_fat_sdcard_unmount(EXAMPLE_SD_MOUNT_POINT, sdmmc_card);
    // if (err == ESP_OK) {
    //     ESP_LOGI("sd", "unmount ok ");
    // } else {
    //     ESP_LOGE("sd", "unmount NG");
    // }

    while (1)
    {
        vTaskDelay(1000);
    }
}
