#include "my_ntp.h"
#include "i2c_ds3231.h"
static const char TAG[] = "my_ntp";
SemaphoreHandle_t i2c_mutex;

void time_sync_notification_cb(struct timeval *tv) {
    ESP_LOGI("SNTP", "Time synchronized");
    time_t now;
    struct tm timeinfo;
   // 获取当前时间戳
    time(&now);
    // 将时间戳转换为本地时间
    localtime_r(&now, &timeinfo);
    ds3231_set_time(&i2c_ds3231_handle,&timeinfo);
    ESP_LOGI(TAG, "write time to ds3231");

}



// SNTP 初始化
void initialize_sntp() {
    ESP_LOGI(TAG, "Initializing SNTP");

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
        // 设置时间服务器（默认使用 pool.ntp.org）
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    // 添加 NTP 服务器
    esp_sntp_setservername(0, "cn.pool.ntp.org"); // 中国 NTP 服务器
    esp_sntp_setservername(1, "ntp1.aliyun.com"); //阿里云 NTP 服务器

    sntp_set_sync_interval(30*60*1000);
    sntp_set_time_sync_notification_cb(time_sync_notification_cb);

    // 初始化 SNTP
    esp_sntp_init();
#else
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "pool.ntp.org");
    sntp_setservername(1, "cn.pool.ntp.org");
    sntp_setservername(2, "ntp1.aliyun.com");


    sntp_init();// 初始化 SNTP
#endif
    // 设置时区（例如：北京时间 UTC+8）
    setenv("TZ", "CST-8", 1);
    tzset();
}

// 打印当前时间
void print_current_time() {
    time_t now;
    struct tm timeinfo;
   // 获取当前时间戳
    time(&now);
    // 将时间戳转换为本地时间
    localtime_r(&now, &timeinfo);

    // 使用 asctime 打印时间
    char *time_str = asctime(&timeinfo);
    if (time_str != NULL) {
        // 去掉 asctime 输出的换行符
        time_str[strlen(time_str) - 1] = '\0';
        ESP_LOGI(TAG, "Current time: %s", time_str);
    } else {
        ESP_LOGE(TAG, "Failed to convert time to string");
    }
}


