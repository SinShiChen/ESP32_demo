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

#define I2S_MAX_KEEP SOC_I2S_NUM

esp_err_t code_dev_init(i2c_master_bus_handle_t *i2c_bus_handle);
esp_err_t _audio_player_write_fn(void *audio_buffer, size_t len, size_t *bytes_written, uint32_t timeout_ms);

void mp3_player_init(void);
#ifdef __cplusplus
}
#endif