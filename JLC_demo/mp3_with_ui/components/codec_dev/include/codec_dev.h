#pragma once
#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/i2s_std.h"
#include "driver/i2s_tdm.h"
#include "soc/soc_caps.h"
#include "esp_codec_dev.h"
#include "esp_codec_dev_defaults.h"
#include "driver/i2c_master.h"


#ifdef __cplusplus
extern "C" {
#endif

#define TEST_BOARD_I2S_BCK_PIN      (GPIO_NUM_14)
#define TEST_BOARD_I2S_MCK_PIN      (GPIO_NUM_38)
#define TEST_BOARD_I2S_DATA_IN_PIN  (-1)
#define TEST_BOARD_I2S_DATA_OUT_PIN (GPIO_NUM_45)
#define TEST_BOARD_I2S_DATA_WS_PIN  (GPIO_NUM_13)

#define EVENT_BIT_PAUSE  0x01
#define EVENT_BIT_PLAY   0x02
#define EVENT_BIT_NEXT   0x04
#define EVENT_BIT_PREVIOUS   0x8

#define I2S_MAX_KEEP SOC_I2S_NUM

typedef enum {
    CMD_PLAY=0,
    CMD_PAUSE,
    CMD_NEXT,
    CMD_PREV,
    CMD_PLAY_SELECTED,
    CMD_VOL
} player_command_t;

// 消息结构体
typedef struct {
    player_command_t cmd;
    char file_path[256]; // 选中的歌曲路径
} player_message_t;

extern EventGroupHandle_t mp3_play_event;
extern QueueHandle_t mp3_queue_handle;
esp_err_t code_dev_init(i2c_master_bus_handle_t *i2c_bus_handle);
esp_err_t _audio_player_write_fn(void *audio_buffer, size_t len, size_t *bytes_written, uint32_t timeout_ms);
int codec_devplayer_write_db(int db);
void mp3_player_init(void);


#ifdef __cplusplus
}
#endif