#include "codec_dev.h"
#include "file_iterator.h"
#include "audio_player.h"
#include "sdmount.h"
#include "PCA9557.h"
static const char *TAG = "player";

typedef struct {
    i2s_chan_handle_t tx_handle;
    i2s_chan_handle_t rx_handle;
} i2s_keep_t;

static i2s_comm_mode_t i2s_in_mode = I2S_COMM_MODE_STD;
static i2s_comm_mode_t i2s_out_mode = I2S_COMM_MODE_STD;
static i2s_keep_t *i2s_keep[I2S_MAX_KEEP];
static esp_codec_dev_handle_t play_dev;
static file_iterator_instance_t *file_iterator = NULL;

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
    
    if (i2s_out_mode == I2S_COMM_MODE_STD) {
        ret = i2s_channel_init_std_mode(i2s_keep[port]->tx_handle, &std_cfg);
    }
#if SOC_I2S_SUPPORTS_TDM 
    else if (i2s_out_mode == I2S_COMM_MODE_TDM) {
        ret = i2s_channel_init_tdm_mode(i2s_keep[port]->tx_handle, &tdm_cfg);
    }
#endif
    
    if (i2s_in_mode == I2S_COMM_MODE_STD) {
        ret = i2s_channel_init_std_mode(i2s_keep[port]->rx_handle, &std_cfg);
    } 
#if SOC_I2S_SUPPORTS_TDM 
    else if (i2s_in_mode == I2S_COMM_MODE_TDM) {
        ret = i2s_channel_init_tdm_mode(i2s_keep[port]->rx_handle, &tdm_cfg);
    }
#endif
    
    // For tx master using duplex mode
    i2s_channel_enable(i2s_keep[port]->tx_handle);

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

esp_err_t code_dev_init(i2c_master_bus_handle_t *i2c_bus_handle)
{
    ut_i2s_init(0);
    audio_codec_i2s_cfg_t i2s_cfg = {
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
        .rx_handle = i2s_keep[0]->rx_handle,
        .tx_handle = i2s_keep[0]->tx_handle,
#endif
    };
    const audio_codec_data_if_t *data_if = audio_codec_new_i2s_data(&i2s_cfg);
    audio_codec_i2c_cfg_t i2c_cfg = {.addr = ES8311_CODEC_DEFAULT_ADDR};
    i2c_cfg.bus_handle = *i2c_bus_handle;
    const audio_codec_ctrl_if_t *out_ctrl_if = audio_codec_new_i2c_ctrl(&i2c_cfg);
    const audio_codec_gpio_if_t *gpio_if = audio_codec_new_gpio();

    es8311_codec_cfg_t es8311_cfg = {
        .codec_mode = ESP_CODEC_DEV_WORK_MODE_DAC,
        .ctrl_if = out_ctrl_if,
        .gpio_if = gpio_if,
        .pa_pin = GPIO_NUM_NC,
        .use_mclk = true,
    };
    const audio_codec_if_t *out_codec_if = es8311_codec_new(&es8311_cfg);

    esp_codec_dev_cfg_t dev_cfg = {
        .codec_if = out_codec_if,
        .data_if = data_if,
        .dev_type = ESP_CODEC_DEV_TYPE_OUT,
    };
    play_dev = esp_codec_dev_new(&dev_cfg);
    esp_codec_dev_set_out_vol(play_dev, 6.0);

    esp_codec_dev_sample_info_t fs = {
        .sample_rate = 48000,
        .channel = 2,
        .bits_per_sample = 16,
    };

    esp_codec_dev_open(play_dev, &fs);
    return ESP_OK;
}

// 播放音乐函数 播放音乐的时候 会不断进入
esp_err_t _audio_player_write_fn(void *audio_buffer, size_t len, size_t *bytes_written, uint32_t timeout_ms)
{
    esp_err_t ret = ESP_OK;
    ret = esp_codec_dev_write(play_dev, audio_buffer, len);
    *bytes_written = len;
    return ret;
}

// 设置采样率 播放的时候进入一次
static esp_err_t _audio_player_std_clock(uint32_t rate, uint32_t bits_cfg, i2s_slot_mode_t ch)
{
    esp_err_t ret = ESP_OK;
    esp_codec_dev_close(play_dev);
    esp_codec_dev_sample_info_t fs = {
        .sample_rate = rate,
        .channel = ch,
        .bits_per_sample = bits_cfg,
    };

    ret = esp_codec_dev_open(play_dev, &fs);
    return ret;
}

// 播放指定序号的音乐
static void play_index(int index)
{
    ESP_LOGI(TAG, "play_index(%d)", index);

    static char filename[128];
    // int retval = file_iterator_get_full_path_from_index(file_iterator, index, filename, sizeof(filename));
    // if (retval == 0) {
    //     ESP_LOGE(TAG, "unable to retrieve filename");
    //     return;
    // }
    int file_count = file_iterator_get_count(file_iterator);
    for (size_t i = index; i < file_count; i++)
    {
        int retval = file_iterator_get_full_path_from_index(file_iterator, i, filename, sizeof(filename));
        ESP_LOGI(TAG, "name '%s'", filename);

        if (retval == 0) {
            ESP_LOGE(TAG, "unable to retrieve filename");
            return;
        }
        if (strcasestr(filename, ".mp3") != NULL || strcasestr(filename, ".wav") != NULL) {
            break;
        }
        file_iterator_next(file_iterator);
    }
    

    FILE *fp = fopen(filename, "rb");
    if (fp) {
        ESP_LOGI(TAG, "Playing '%s'", filename);
        audio_player_play(fp);
    } else {
        ESP_LOGE(TAG, "unable to open index %d, filename '%s'", index, filename);
    }
}
static esp_err_t _audio_player_mute_fn(AUDIO_PLAYER_MUTE_SETTING setting)
{
    return ESP_OK;
}
// 回调函数 播放器每次动作都会进入
static void _audio_player_callback(audio_player_cb_ctx_t *ctx)
{
    ESP_LOGI(TAG, "ctx->audio_event = %d", ctx->audio_event);
     ESP_LOGI(TAG, "event %d",ctx->audio_event);
    switch (ctx->audio_event) {
    case AUDIO_PLAYER_CALLBACK_EVENT_IDLE: {  // 播放完一首歌 进入这个case
        pa_en(&i2c_pca9557_handle,0); // 关闭音频功放
        ESP_LOGI(TAG, "AUDIO_PLAYER_REQUEST_IDLE");
        // 指向下一首歌
        file_iterator_next(file_iterator);
        int index = file_iterator_get_index(file_iterator);
        ESP_LOGI(TAG, "playing index '%d'", index);
        play_index(index);
        // 修改当前播放的音乐名称
        break;
    }
    case AUDIO_PLAYER_CALLBACK_EVENT_PLAYING: // 正在播放音乐
        ESP_LOGI(TAG, "AUDIO_PLAYER_REQUEST_PLAY");
        pa_en(&i2c_pca9557_handle,1); // 打开音频功放
        break;
    case AUDIO_PLAYER_CALLBACK_EVENT_PAUSE: // 正在暂停音乐
        ESP_LOGI(TAG, "AUDIO_PLAYER_REQUEST_PAUSE");
        pa_en(&i2c_pca9557_handle,0); // 关闭音频功放
        break;
    default:
        break;
    }
}
static audio_player_config_t player_config = {0};
// mp3播放器初始化
void mp3_player_init(void)
{
    // 获取文件信息
    file_iterator = file_iterator_new(EXAMPLE_SD_MOUNT_POINT);
    assert(file_iterator != NULL);

    // 初始化音频播放
    player_config.mute_fn = _audio_player_mute_fn;
    player_config.write_fn = _audio_player_write_fn;
    player_config.clk_set_fn = _audio_player_std_clock;
    player_config.priority = 5;

    ESP_ERROR_CHECK(audio_player_new(player_config));
    ESP_ERROR_CHECK(audio_player_callback_register(_audio_player_callback, NULL));
    // file_iterator_next(file_iterator);
    int index = file_iterator_get_index(file_iterator);
    play_index(index);
    audio_player_resume();
}