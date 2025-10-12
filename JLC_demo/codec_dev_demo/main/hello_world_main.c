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

#define TEST_BOARD_I2C_SDA_PIN      (GPIO_NUM_1)
#define TEST_BOARD_I2C_SCL_PIN      (GPIO_NUM_2)

#define TEST_BOARD_I2S_BCK_PIN      (GPIO_NUM_14)
#define TEST_BOARD_I2S_MCK_PIN      (GPIO_NUM_38)
#define TEST_BOARD_I2S_DATA_IN_PIN  (-1)
#define TEST_BOARD_I2S_DATA_OUT_PIN (GPIO_NUM_45)
#define TEST_BOARD_I2S_DATA_WS_PIN  (GPIO_NUM_13)

#define PCA9557_INPUT_PORT              0x00
#define PCA9557_OUTPUT_PORT             0x01
#define PCA9557_POLARITY_INVERSION_PORT 0x02
#define PCA9557_CONFIGURATION_PORT      0x03

#define LCD_CS_GPIO                 BIT(0)    // PCA9557_GPIO_NUM_1
#define PA_EN_GPIO                  BIT(1)    // PCA9557_GPIO_NUM_2
#define DVP_PWDN_GPIO               BIT(2)    // PCA9557_GPIO_NUM_3

#define PCA9557_SENSOR_ADDR             0x19        /*!< Slave address of the MPU9250 sensor */
#define SET_BITS(_m, _s, _v)  ((_v) ? (_m)|((_s)) : (_m)&~((_s)))
#define BSP_I2C_FREQ_HZ       100000         // 100kHz

/* Import music file as buffer */
extern const uint8_t music_pcm_start[] asm("_binary_canon_pcm_start");
extern const uint8_t music_pcm_end[]   asm("_binary_canon_pcm_end");

static i2c_master_bus_handle_t i2c_bus_handle;



esp_err_t bsp_i2c_init(i2c_master_bus_handle_t *bus_handle)
{
    i2c_master_bus_config_t i2c_bus_config = {
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .i2c_port = 0,
    .scl_io_num = TEST_BOARD_I2C_SCL_PIN,
    .sda_io_num = TEST_BOARD_I2C_SDA_PIN,
    .glitch_ignore_cnt = 7,
    .flags.enable_internal_pullup = true,
    };
    return i2c_new_master_bus(&i2c_bus_config, bus_handle);
}

// 读取PCA9557寄存器的值
esp_err_t pca9557_register_read(i2c_master_dev_handle_t *handle,uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(*handle, &reg_addr, 1, data, len, -1);
}

// 给PCA9557的寄存器写值
esp_err_t pca9557_register_write_byte(i2c_master_dev_handle_t *handle,uint8_t reg_addr, uint8_t data)
{
    uint8_t write_buf[2] = {reg_addr, data};
    return i2c_master_transmit(*handle, write_buf, 2,-1);
}


// 初始化PCA9557 IO扩展芯片
esp_err_t pca9557_init(i2c_master_bus_handle_t bus_handle,i2c_master_dev_handle_t *handle)
{

    // CHECK_ARG(bus_handle);
    esp_err_t ret = ESP_OK;

    i2c_device_config_t i2c_dev_conf = {
        .scl_speed_hz = BSP_I2C_FREQ_HZ,
        .device_address = PCA9557_SENSOR_ADDR,
    };

    ret = i2c_master_bus_add_device(bus_handle, &i2c_dev_conf, handle);
    if (ret != ESP_OK)
        return ret;
    
    // 写入控制引脚默认值 DVP_PWDN=1  PA_EN = 0  LCD_CS = 1
    ret = pca9557_register_write_byte(handle,PCA9557_OUTPUT_PORT, 0x05);  
    if (ret != ESP_OK)
        return ret;
    // 把PCA9557芯片的IO1 IO1 IO2设置为输出 其它引脚保持默认的输入
    ret = pca9557_register_write_byte(handle,PCA9557_CONFIGURATION_PORT, 0xf8); 
    return ret;
    
}

// 设置PCA9557芯片的某个IO引脚输出高低电平
esp_err_t pca9557_set_output_state(i2c_master_dev_handle_t *handle,uint8_t gpio_bit, uint8_t level)
{
    uint8_t data;
    esp_err_t res = ESP_FAIL;

    pca9557_register_read(handle,PCA9557_OUTPUT_PORT, &data, 1);
    res = pca9557_register_write_byte(handle,PCA9557_OUTPUT_PORT, SET_BITS(data, gpio_bit, level));

    return res;
}

// 控制 PCA9557_PA_EN 引脚输出高低电平 参数0输出低电平 参数1输出高电平 
void pa_en(i2c_master_dev_handle_t *handle,uint8_t level)
{
    pca9557_set_output_state(handle,PA_EN_GPIO, level);
}

static int ut_i2c_init(uint8_t port)
{
    i2c_master_bus_config_t i2c_bus_config = {0};
    i2c_bus_config.clk_source = I2C_CLK_SRC_DEFAULT;
    i2c_bus_config.i2c_port = port;
    i2c_bus_config.scl_io_num = TEST_BOARD_I2C_SCL_PIN;
    i2c_bus_config.sda_io_num = TEST_BOARD_I2C_SDA_PIN;
    i2c_bus_config.glitch_ignore_cnt = 7;
    i2c_bus_config.flags.enable_internal_pullup = true;
    return i2c_new_master_bus(&i2c_bus_config, &i2c_bus_handle);
}

static int ut_i2c_deinit(uint8_t port)
{
   if (i2c_bus_handle) {
       i2c_del_master_bus(i2c_bus_handle);
   }
   i2c_bus_handle = NULL;
   return 0;
}


#define I2S_MAX_KEEP SOC_I2S_NUM

typedef struct {
    i2s_chan_handle_t tx_handle;
    i2s_chan_handle_t rx_handle;
} i2s_keep_t;

static i2s_comm_mode_t i2s_in_mode = I2S_COMM_MODE_STD;
static i2s_comm_mode_t i2s_out_mode = I2S_COMM_MODE_STD;
static i2s_keep_t *i2s_keep[I2S_MAX_KEEP];


static void ut_set_i2s_mode(i2s_comm_mode_t out_mode, i2s_comm_mode_t in_mode)
{
    i2s_in_mode = in_mode;
    i2s_out_mode = out_mode;
}

static void ut_clr_i2s_mode(void)
{
    i2s_in_mode = I2S_COMM_MODE_STD;
    i2s_out_mode = I2S_COMM_MODE_STD;
}

static int ut_i2s_init(uint8_t port)
{
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    if (port >= I2S_MAX_KEEP) {
        return -1;
    }
    // Already installed
    if (i2s_keep[port]) {
        return 0;
    }
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(16000),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(16, I2S_SLOT_MODE_STEREO),
        .gpio_cfg ={
            .mclk = TEST_BOARD_I2S_MCK_PIN,
            .bclk = TEST_BOARD_I2S_BCK_PIN,
            .ws = TEST_BOARD_I2S_DATA_WS_PIN,
            .dout = TEST_BOARD_I2S_DATA_OUT_PIN,
            .din = TEST_BOARD_I2S_DATA_IN_PIN,
        },
    };
    i2s_keep[port] = (i2s_keep_t *) calloc(1, sizeof(i2s_keep_t));
    if (i2s_keep[port] == NULL) {
        return -1;
    }
#if SOC_I2S_SUPPORTS_TDM 
    i2s_tdm_slot_mask_t slot_mask = I2S_TDM_SLOT0 | I2S_TDM_SLOT1 | I2S_TDM_SLOT2 | I2S_TDM_SLOT3;
    i2s_tdm_config_t tdm_cfg = {
        .slot_cfg = I2S_TDM_PHILIPS_SLOT_DEFAULT_CONFIG(16, I2S_SLOT_MODE_STEREO, slot_mask),
        .clk_cfg  = I2S_TDM_CLK_DEFAULT_CONFIG(16000),
        .gpio_cfg = {
            .mclk = TEST_BOARD_I2S_MCK_PIN,
            .bclk = TEST_BOARD_I2S_BCK_PIN,
            .ws = TEST_BOARD_I2S_DATA_WS_PIN,
            .dout = TEST_BOARD_I2S_DATA_OUT_PIN,
            .din = TEST_BOARD_I2S_DATA_IN_PIN,
        },
    };
    tdm_cfg.slot_cfg.total_slot = 4;
#endif
    int ret = i2s_new_channel(&chan_cfg, &i2s_keep[port]->tx_handle, &i2s_keep[port]->rx_handle);
    TEST_ESP_OK(ret);
    if (i2s_out_mode == I2S_COMM_MODE_STD) {
        ret = i2s_channel_init_std_mode(i2s_keep[port]->tx_handle, &std_cfg);
    }
#if SOC_I2S_SUPPORTS_TDM 
    else if (i2s_out_mode == I2S_COMM_MODE_TDM) {
        ret = i2s_channel_init_tdm_mode(i2s_keep[port]->tx_handle, &tdm_cfg);
    }
#endif
    TEST_ESP_OK(ret);
    if (i2s_in_mode == I2S_COMM_MODE_STD) {
        ret = i2s_channel_init_std_mode(i2s_keep[port]->rx_handle, &std_cfg);
    } 
#if SOC_I2S_SUPPORTS_TDM 
    else if (i2s_in_mode == I2S_COMM_MODE_TDM) {
        ret = i2s_channel_init_tdm_mode(i2s_keep[port]->rx_handle, &tdm_cfg);
    }
#endif
    TEST_ESP_OK(ret);
    // For tx master using duplex mode
    i2s_channel_enable(i2s_keep[port]->tx_handle);
#else
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t) (I2S_MODE_TX | I2S_MODE_RX | I2S_MODE_MASTER),
        .sample_rate = 44100,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL2 | ESP_INTR_FLAG_IRAM,
        .dma_buf_count = 2,
        .dma_buf_len = 128,
        .use_apll = true,
        .tx_desc_auto_clear = true,
    };
    int ret = i2s_driver_install(port, &i2s_config, 0, NULL);
    i2s_pin_config_t i2s_pin_cfg = {
        .mck_io_num = TEST_BOARD_I2S_MCK_PIN,
        .bck_io_num = TEST_BOARD_I2S_BCK_PIN,
        .ws_io_num = TEST_BOARD_I2S_DATA_WS_PIN,
        .data_out_num = TEST_BOARD_I2S_DATA_OUT_PIN,
        .data_in_num = TEST_BOARD_I2S_DATA_IN_PIN,
    };
    i2s_set_pin(port, &i2s_pin_cfg);
#endif
    return ret;
}

static int ut_i2s_deinit(uint8_t port)
{
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    if (port >= I2S_MAX_KEEP) {
        return -1;
    }
    // already installed
    if (i2s_keep[port] == NULL) {
        return 0;
    }
    i2s_channel_disable(i2s_keep[port]->tx_handle);
    i2s_channel_disable(i2s_keep[port]->rx_handle);
    i2s_del_channel(i2s_keep[port]->tx_handle);
    i2s_del_channel(i2s_keep[port]->rx_handle);
    free(i2s_keep[port]);
    i2s_keep[port] = NULL;
#else
    i2s_driver_uninstall(port);
#endif
    return 0;
}

static void codec_max_sample(uint8_t *data, int size, int *max_value, int *min_value)
{
    int16_t *s = (int16_t *) data;
    size >>= 1;
    int i = 1, max, min;
    max = min = s[0];
    while (i < size) {
        if (s[i] > max) {
            max = s[i];
        } else if (s[i] < min) {
            min = s[i];
        }
        i++;
    }
    *max_value = max;
    *min_value = min;
}

void app_main(void)
{
    // Need install driver (i2c and i2s) firstly
    int ret = ut_i2c_init(0);
    TEST_ESP_OK(ret);
    ret = ut_i2s_init(0);
    TEST_ESP_OK(ret);

    i2c_master_dev_handle_t i2c_pca9557_handle ;
    pca9557_init(i2c_bus_handle,&i2c_pca9557_handle);
    pa_en(&i2c_pca9557_handle,1);
    // Do initialize of related interface: data_if, ctrl_if and gpio_if
    audio_codec_i2s_cfg_t i2s_cfg = {
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
        .rx_handle = i2s_keep[0]->rx_handle,
        .tx_handle = i2s_keep[0]->tx_handle,
#endif
    };
    const audio_codec_data_if_t *data_if = audio_codec_new_i2s_data(&i2s_cfg);
    TEST_ASSERT_NOT_NULL(data_if);

    audio_codec_i2c_cfg_t i2c_cfg = {.addr = ES8311_CODEC_DEFAULT_ADDR};

    i2c_cfg.bus_handle = i2c_bus_handle;

    const audio_codec_ctrl_if_t *out_ctrl_if = audio_codec_new_i2c_ctrl(&i2c_cfg);
    TEST_ASSERT_NOT_NULL(out_ctrl_if);

    const audio_codec_gpio_if_t *gpio_if = audio_codec_new_gpio();
    TEST_ASSERT_NOT_NULL(gpio_if);
    // New output codec interface
    es8311_codec_cfg_t es8311_cfg = {
        .codec_mode = ESP_CODEC_DEV_WORK_MODE_DAC,
        .ctrl_if = out_ctrl_if,
        .gpio_if = gpio_if,
        .pa_pin = GPIO_NUM_NC,
        .use_mclk = true,
    };
    const audio_codec_if_t *out_codec_if = es8311_codec_new(&es8311_cfg);
    TEST_ASSERT_NOT_NULL(out_codec_if);

    // New output codec device
    esp_codec_dev_cfg_t dev_cfg = {
        .codec_if = out_codec_if,
        .data_if = data_if,
        .dev_type = ESP_CODEC_DEV_TYPE_OUT,
    };
    esp_codec_dev_handle_t play_dev = esp_codec_dev_new(&dev_cfg);
    TEST_ASSERT_NOT_NULL(play_dev);

    ret = esp_codec_dev_set_out_vol(play_dev, 60.0);
    TEST_ESP_OK(ret);

    esp_codec_dev_sample_info_t fs = {
        .sample_rate = 16000,
        .channel = 2,
        .bits_per_sample = 16,
    };

    ret = esp_codec_dev_open(play_dev, &fs);
    TEST_ESP_OK(ret);
    uint8_t *p = music_pcm_start;

    for (int i = 0; i < ((music_pcm_end-music_pcm_start));)
    {
        ret = esp_codec_dev_write(play_dev, p+i, 256);
        TEST_ESP_OK(ret);
        i += 256;
        ESP_LOGI("e","%d",i);
        if(i == music_pcm_end-music_pcm_start)
            i = 0;
    }
    while (1)
    {
        vTaskDelay(1000);
    }
    
    
}
