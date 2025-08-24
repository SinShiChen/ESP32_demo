#include <time.h>
#include <stdbool.h>
#include "esp_sntp.h"
#include "esp_log.h"
#include "esp_system.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_sntp.h"
#ifdef __cplusplus
extern "C" {
#endif

#define CONFIG_BLINK_PERIOD 1000


extern SemaphoreHandle_t i2c_mutex;


void initialize_sntp();
void print_current_time();
void time_sync_notification_cb(struct timeval *tv);
#ifdef __cplusplus
}
#endif
