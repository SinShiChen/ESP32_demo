#include "app_ui.h"
// #include "audio_player.h"
// #include "esp32_s3_szp.h"
#include "file_iterator.h"
#include "string.h"
#include <dirent.h>
#include "codec_dev.h"
#include "sdmount.h"
#include "esp_log.h"
#include "codec_dev.h"

static const char *TAG = "app_ui";
#include "GenShinGothic_ExtraLight_2.h"

// static audio_player_config_t player_config = {0};
static uint8_t g_sys_volume = 80;
static file_iterator_instance_t *file_iterator = NULL;

lv_obj_t *music_list;
_lock_t lvgl_api_lock;

// 按钮样式相关定义
typedef struct {
    lv_style_t style_bg;
    lv_style_t style_focus_no_outline;
} button_style_t;

static button_style_t g_btn_styles;

button_style_t *ui_button_styles(void)
{
    return &g_btn_styles;
}

// 按钮样式初始化
static void ui_button_style_init(void)
{
    /*Init the style for the default state*/
    lv_style_init(&g_btn_styles.style_focus_no_outline);
    lv_style_set_outline_width(&g_btn_styles.style_focus_no_outline, 0);

    lv_style_init(&g_btn_styles.style_bg);
    lv_style_set_bg_opa(&g_btn_styles.style_bg, LV_OPA_100);
    lv_style_set_bg_color(&g_btn_styles.style_bg, lv_color_make(255, 255, 255));
    lv_style_set_shadow_width(&g_btn_styles.style_bg, 0);
}

// 播放暂停按钮 事件处理函数
static void btn_play_pause_cb(lv_event_t *event)
{
    lv_obj_t *btn = lv_event_get_target(event);
    lv_obj_t *lab = lv_obj_get_user_data(btn);
    player_message_t msg={CMD_PLAY,""};
    // _lock_acquire(&lvgl_api_lock);
    const char * text = lv_label_get_text(lab);
    if (memcmp(text,LV_SYMBOL_PAUSE,3)==0)
    {
        lv_label_set_text_static(lab, LV_SYMBOL_PLAY);
        msg.cmd = CMD_PLAY;
    }
    else if (memcmp(text,LV_SYMBOL_PLAY,3)==0)
    {
        lv_label_set_text_static(lab, LV_SYMBOL_PAUSE);
        msg.cmd = CMD_PAUSE;
    }
    xQueueSend(mp3_queue_handle, &msg, portMAX_DELAY);
    // _lock_release(&lvgl_api_lock);
}

// 上一首 下一首 按键事件处理函数
static void btn_prev_next_cb(lv_event_t *event)
{
    bool is_next = lv_event_get_user_data(event);
    player_message_t msg;
    if (is_next) {
        ESP_LOGI(TAG, "btn next");
    } else {
        ESP_LOGI(TAG, "btn prev");
    }
    int index = file_iterator_get_index(file_iterator);
    msg.cmd = CMD_PLAY_SELECTED;
    memset(msg.file_path,0,256);
    file_iterator_get_full_path_from_index(file_iterator, index,msg.file_path,256);
    xQueueSend(mp3_queue_handle, &msg, portMAX_DELAY);
    file_iterator_prev(file_iterator);

    // _lock_acquire(&lvgl_api_lock);
    lv_dropdown_set_selected(music_list, index);
    lv_obj_t *label_title = lv_obj_get_user_data(music_list);
    lv_label_set_text_static(label_title, file_iterator_get_name_from_index(file_iterator, index));
    // _lock_release(&lvgl_api_lock);
}

// 音量调节滑动条 事件处理函数
static void volume_slider_cb(lv_event_t *event)
{
    lv_obj_t *slider = lv_event_get_target(event);
    int volume = lv_slider_get_value(slider); // 获取slider的值
    codec_devplayer_write_db(volume);
    g_sys_volume = volume; // 把声音赋值给g_sys_volume保存
    ESP_LOGI(TAG, "volume '%d'", volume);

}

// 音乐列表 点击事件处理函数
static void music_list_cb(lv_event_t *event)
{

}

// 音乐名称加入列表
static void build_file_list(lv_obj_t *music_list)
{
    // lv_obj_t *label_title = (lv_obj_t *) music_list->user_data;
    lv_obj_t *label_title = lv_obj_get_user_data(music_list);


    lv_dropdown_clear_options(music_list);
    for(size_t i = 0; i<file_iterator->count; i++)
    {
        ESP_LOGI("LVGL","cnt %d",i);
        const char *file_name = file_iterator_get_name_from_index(file_iterator, i);
        ESP_LOGI("LVGL","name %s",file_name);
        if (NULL != file_name) {
            lv_dropdown_add_option(music_list, file_name, i); // 添加音乐名称到列表中
        }
    }
    lv_dropdown_set_selected(music_list, 0); // 选择列表中的第一个
    lv_label_set_text_static(label_title, file_iterator_get_name_from_index(file_iterator, 0)); // 显示list中第一个音乐的名称
}

// 播放器界面初始化
void music_ui(void)
{
    file_iterator = file_iterator_new(EXAMPLE_SD_MOUNT_POINT);
    ui_button_style_init();// 初始化按键风格

    /* 创建播放暂停控制按键 */
    lv_obj_t *btn_play_pause = lv_btn_create(lv_scr_act());
    lv_obj_align(btn_play_pause, LV_ALIGN_CENTER, 0, 40);
    lv_obj_set_size(btn_play_pause, 50, 50);
    lv_obj_set_style_radius(btn_play_pause, 25, LV_STATE_DEFAULT);
    lv_obj_add_flag(btn_play_pause, LV_OBJ_FLAG_CHECKABLE);

    lv_obj_add_style(btn_play_pause, &ui_button_styles()->style_focus_no_outline, LV_STATE_FOCUS_KEY);
    lv_obj_add_style(btn_play_pause, &ui_button_styles()->style_focus_no_outline, LV_STATE_FOCUSED);

    lv_obj_t *label_play_pause = lv_label_create(btn_play_pause);
    lv_label_set_text_static(label_play_pause, LV_SYMBOL_PLAY);
    lv_obj_center(label_play_pause);

    lv_obj_set_user_data(btn_play_pause, (void *) label_play_pause);
    lv_obj_add_event_cb(btn_play_pause, btn_play_pause_cb, LV_EVENT_VALUE_CHANGED, NULL);

    /* 创建上一首控制按键 */
    lv_obj_t *btn_play_prev = lv_btn_create(lv_scr_act());
    lv_obj_set_size(btn_play_prev, 50, 50);
    lv_obj_set_style_radius(btn_play_prev, 25, LV_STATE_DEFAULT);
    lv_obj_clear_flag(btn_play_prev, LV_OBJ_FLAG_CHECKABLE);
    lv_obj_align_to(btn_play_prev, btn_play_pause, LV_ALIGN_OUT_LEFT_MID, -40, 0); 

    lv_obj_add_style(btn_play_prev, &ui_button_styles()->style_focus_no_outline, LV_STATE_FOCUS_KEY);
    lv_obj_add_style(btn_play_prev, &ui_button_styles()->style_focus_no_outline, LV_STATE_FOCUSED);
    lv_obj_add_style(btn_play_prev, &ui_button_styles()->style_bg, LV_STATE_FOCUS_KEY);
    lv_obj_add_style(btn_play_prev, &ui_button_styles()->style_bg, LV_STATE_FOCUSED);
    lv_obj_add_style(btn_play_prev, &ui_button_styles()->style_bg, LV_STATE_DEFAULT);

    lv_obj_t *label_prev = lv_label_create(btn_play_prev);
    lv_label_set_text_static(label_prev, LV_SYMBOL_PREV);
    // lv_obj_set_style_text_font(label_prev, &lv_font_montserrat_24, LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(label_prev, lv_color_make(0, 0, 0), LV_STATE_DEFAULT);
    lv_obj_center(label_prev);
    lv_obj_set_user_data(btn_play_prev, (void *) label_prev);
    lv_obj_add_event_cb(btn_play_prev, btn_prev_next_cb, LV_EVENT_CLICKED, (void *) false);

    /* 创建下一首控制按键 */
    lv_obj_t *btn_play_next = lv_btn_create(lv_scr_act());
    lv_obj_set_size(btn_play_next, 50, 50);
    lv_obj_set_style_radius(btn_play_next, 25, LV_STATE_DEFAULT);
    lv_obj_clear_flag(btn_play_next, LV_OBJ_FLAG_CHECKABLE);
    lv_obj_align_to(btn_play_next, btn_play_pause, LV_ALIGN_OUT_RIGHT_MID, 40, 0);

    lv_obj_add_style(btn_play_next, &ui_button_styles()->style_focus_no_outline, LV_STATE_FOCUS_KEY);
    lv_obj_add_style(btn_play_next, &ui_button_styles()->style_focus_no_outline, LV_STATE_FOCUSED);
    lv_obj_add_style(btn_play_next, &ui_button_styles()->style_bg, LV_STATE_FOCUS_KEY);
    lv_obj_add_style(btn_play_next, &ui_button_styles()->style_bg, LV_STATE_FOCUSED);
    lv_obj_add_style(btn_play_next, &ui_button_styles()->style_bg, LV_STATE_DEFAULT);

    lv_obj_t *label_next = lv_label_create(btn_play_next);
    lv_label_set_text_static(label_next, LV_SYMBOL_NEXT);
    // lv_obj_set_style_text_font(label_next, &lv_font_montserrat_24, LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(label_next, lv_color_make(0, 0, 0), LV_STATE_DEFAULT);
    lv_obj_center(label_next);
    lv_obj_set_user_data(btn_play_next, (void *) label_next);
    lv_obj_add_event_cb(btn_play_next, btn_prev_next_cb, LV_EVENT_CLICKED, (void *) true);

    /* 创建声音调节滑动条 */
    lv_obj_t *volume_slider = lv_slider_create(lv_scr_act());
    lv_obj_set_size(volume_slider, 200, 10);
    lv_obj_set_ext_click_area(volume_slider, 15);
    lv_obj_align(volume_slider, LV_ALIGN_BOTTOM_MID, 0, -20);
    lv_slider_set_range(volume_slider, 0, 100);
    lv_slider_set_value(volume_slider, g_sys_volume, LV_ANIM_ON);
    lv_obj_add_event_cb(volume_slider, volume_slider_cb, LV_EVENT_VALUE_CHANGED, NULL);

    lv_obj_t *lab_vol_min = lv_label_create(lv_scr_act());
    lv_label_set_text_static(lab_vol_min, LV_SYMBOL_VOLUME_MID);
    lv_obj_set_style_text_font(lab_vol_min, &lv_font_montserrat_20, LV_STATE_DEFAULT);
    lv_obj_align_to(lab_vol_min, volume_slider, LV_ALIGN_OUT_LEFT_MID, -10, 0);

    lv_obj_t *lab_vol_max = lv_label_create(lv_scr_act());
    lv_label_set_text_static(lab_vol_max, LV_SYMBOL_VOLUME_MAX);
    lv_obj_set_style_text_font(lab_vol_max, &lv_font_montserrat_20, LV_STATE_DEFAULT);
    lv_obj_align_to(lab_vol_max, volume_slider, LV_ALIGN_OUT_RIGHT_MID, 10, 0);

    /* 创建音乐标题 */
    lv_obj_t *lab_title = lv_label_create(lv_scr_act());
    lv_obj_set_user_data(lab_title, (void *) btn_play_pause);
    lv_label_set_text_static(lab_title, "Scanning Files...");
    lv_obj_set_style_text_font(lab_title, &GenShinGothic_ExtraLight_2, LV_STATE_ANY);
    lv_obj_align(lab_title, LV_ALIGN_TOP_MID, 0, 20);

    /* 创建音乐列表 */ 
    music_list = lv_dropdown_create(lv_scr_act());
    lv_dropdown_clear_options(music_list);
    lv_dropdown_set_options_static(music_list, "Scanning...");
    lv_obj_set_style_text_font(music_list, &lv_font_montserrat_20, LV_STATE_ANY);
    lv_obj_set_width(music_list, 300);
    lv_obj_align_to(music_list, lab_title, LV_ALIGN_OUT_BOTTOM_MID, 0, 20);
    lv_obj_set_user_data(music_list, (void *) lab_title);
    lv_obj_add_event_cb(music_list, music_list_cb, LV_EVENT_VALUE_CHANGED, NULL);


    lv_obj_t *list = lv_dropdown_get_list(music_list);
    lv_obj_set_style_text_font(music_list,&GenShinGothic_ExtraLight_2,0);
    lv_obj_set_style_text_font(music_list, &lv_font_montserrat_20, LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(list,&GenShinGothic_ExtraLight_2,0);


    build_file_list(music_list);

}








