#pragma once
#include "lvgl.h"
// LVGL library is not thread-safe, this example will call LVGL APIs from different tasks, so use a mutex to protect it
extern _lock_t lvgl_api_lock;
void mp3_player_init(void);
void music_ui(void);




