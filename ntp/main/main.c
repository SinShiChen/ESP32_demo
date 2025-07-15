#include <stdio.h>
#include "esp_err.h"
#include "esp_sntp.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

#define WIFI_SSID "lb"	//注意替换wifi信息
#define WIFI_PASS "asdewq0608"
#define CONFIG_BLINK_PERIOD 1000
static const char *TAG = "NTP_TIME";
static void ntp_task(void* pram);
static void show_time_task(void* pram);
SemaphoreHandle_t i2c_mutex;


void initialize_nvs() {
    esp_err_t ret = nvs_flash_init();// 初始化NVS, 并检查是否需要擦除NVS
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
}

void time_sync_notification_cb(struct timeval *tv) {
    ESP_LOGI("SNTP", "Time synchronized");
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
   // char buffer[64];

   // 获取当前时间戳
    time(&now);
    // 将时间戳转换为本地时间
    localtime_r(&now, &timeinfo);
    // 格式化时间
 // 使用 strftime 格式化时间
    // strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", &timeinfo);
    // ESP_LOGI(TAG, "Formatted time: %s", buffer);

    // strftime(buffer, sizeof(buffer), "%A, %B %d, %Y %I:%M:%S %p", &timeinfo);
    // ESP_LOGI(TAG, "Formatted time: %s", buffer);

    // strftime(buffer, sizeof(buffer), "Today is %A, %B %d, %Y. The time is %I:%M %p.", &timeinfo);
    // ESP_LOGI(TAG, "Formatted time: %s", buffer);
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
//事件回调
void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI("WIFI", "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        ESP_LOGI(TAG, "Wi-Fi connected, initializing SNTP...");
        initialize_sntp();
    }
}

void initialize_wifi() {
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));// 注册WiFi事件处理程序
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));// 注册IP事件处理程序

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));// 设置为STA模式
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));// 设置WiFi配置
    ESP_ERROR_CHECK(esp_wifi_start());// 启动WiFi
}

// 打印 Wi-Fi 信息
void print_wifi_info() {
    wifi_config_t wifi_config;
    esp_wifi_get_config(ESP_IF_WIFI_STA, &wifi_config);

    esp_netif_ip_info_t ip_info;
    esp_netif_t* netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (netif && esp_netif_get_ip_info(netif, &ip_info) == ESP_OK) {
        ESP_LOGI(TAG, "Wi-Fi SSID: %s", (char*)wifi_config.sta.ssid);
        ESP_LOGI(TAG, "Wi-Fi Password: %s", (char*)wifi_config.sta.password);
        ESP_LOGI(TAG, "IP Address: " IPSTR, IP2STR(&ip_info.ip));
    } else {
        ESP_LOGE(TAG, "Failed to get IP information");
    }
}



static void ntp_task(void* pram)
{
    while (1)
    {
        // 检查时间是否已同步
        time_t now;
        struct tm timeinfo;
        time(&now);
        localtime_r(&now, &timeinfo);//将时间戳转换为本地时间。
         // 打印当前时间的详细信息
        ESP_LOGI(TAG, "Current time: %04d-%02d-%02d %02d:%02d:%02d",
        timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
        timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
        // 如果时间已同步（年份大于 2020）
        if (timeinfo.tm_year > (2020 - 1900)) {
            if (xSemaphoreTake(i2c_mutex, portMAX_DELAY))
            {
                print_current_time();
                vTaskDelay(500);
                ESP_LOGI(TAG, "remove");
                xSemaphoreGive(i2c_mutex);
                vTaskDelete(NULL);
            }
            // vTaskDelay(5000);

        } else {
            ESP_LOGI(TAG, "Waiting for time synchronization...");
        }
        vTaskDelay(CONFIG_BLINK_PERIOD / portTICK_PERIOD_MS*5);
        // print_wifi_info() ;

    }
}

static void show_time_task(void* pram)
{
    time_t now;
    struct tm timeinfo;
    while (1)
    {
        ESP_LOGI("show_time_task", "while");
        if (xSemaphoreTake(i2c_mutex, portMAX_DELAY))
        {
            // 获取当前时间戳
            time(&now);
            // 将时间戳转换为本地时间
            localtime_r(&now, &timeinfo);
            char *time_str = asctime(&timeinfo);
            if (time_str != NULL) {
                // 去掉 asctime 输出的换行符
                time_str[strlen(time_str) - 1] = '\0';
                ESP_LOGI("show_time_task", "Current time: %s", time_str);
            } else {
                ESP_LOGE("show_time_task", "Failed to convert time to string");
            }
            xSemaphoreGive(i2c_mutex);
        }
        vTaskDelay(500);
    }
}


void app_main(void)
{
    i2c_mutex = xSemaphoreCreateMutex();
    initialize_nvs();// 初始化NVS
    initialize_wifi();  // 初始化Wi-Fi
    xTaskCreate(ntp_task,"ntp_task",4096,NULL,2,NULL);
    xTaskCreate(show_time_task,"show_time_task",4096,NULL,3,NULL);
    while (1)
    {
        vTaskDelay(1000);
    }
}

