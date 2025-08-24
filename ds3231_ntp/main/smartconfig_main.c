/* Esptouch example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_eap_client.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_smartconfig.h"
#include "esp_mac.h"
#include "esp_sntp.h"
#include "nvs.h"
#include "driver/i2c_master.h"

#include "i2c_ds3231.h"
#include "nvs_time.h"
#include "my_ntp.h"
#define CONFIG_BLINK_PERIOD 1000
#define STORAGE_NAMESPACE "NVS"
#define SCL_IO_PIN 6
#define SDA_IO_PIN 7
#define MASTER_FREQUENCY 400000

/* FreeRTOS event group to signal when we are connected & ready to make a request */
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event,
   but we only care about one event - are we connected
   to the AP with an IP? */
static const int CONNECTED_BIT = BIT0;
static const int ESPTOUCH_DONE_BIT = BIT1;
static const char *TAG = "smartconfig_example";

static void ntp_task(void* pram);
static void show_time_task(void* pram);
static void smartconfig_example_task(void * parm);



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


static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        xTaskCreate(smartconfig_example_task, "smartconfig_example_task", 4096, NULL, 3, NULL);
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();
        xEventGroupClearBits(s_wifi_event_group, CONNECTED_BIT);
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        xEventGroupSetBits(s_wifi_event_group, CONNECTED_BIT);
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI("WIFI", "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        ESP_LOGI(TAG, "Wi-Fi connected, initializing SNTP...");
        initialize_sntp();
    } else if (event_base == SC_EVENT && event_id == SC_EVENT_SCAN_DONE) {
        ESP_LOGI(TAG, "Scan done");
    } else if (event_base == SC_EVENT && event_id == SC_EVENT_FOUND_CHANNEL) {
        ESP_LOGI(TAG, "Found channel");
    } else if (event_base == SC_EVENT && event_id == SC_EVENT_GOT_SSID_PSWD) {
        ESP_LOGI(TAG, "Got SSID and password");

        smartconfig_event_got_ssid_pswd_t *evt = (smartconfig_event_got_ssid_pswd_t *)event_data;
        wifi_config_t wifi_config;
        uint8_t ssid[33] = { 0 };
        uint8_t password[65] = { 0 };
        uint8_t rvd_data[33] = { 0 };

        bzero(&wifi_config, sizeof(wifi_config_t));
        memcpy(wifi_config.sta.ssid, evt->ssid, sizeof(wifi_config.sta.ssid));
        memcpy(wifi_config.sta.password, evt->password, sizeof(wifi_config.sta.password));

#ifdef CONFIG_SET_MAC_ADDRESS_OF_TARGET_AP
        wifi_config.sta.bssid_set = evt->bssid_set;
        if (wifi_config.sta.bssid_set == true) {
            ESP_LOGI(TAG, "Set MAC address of target AP: "MACSTR" ", MAC2STR(evt->bssid));
            memcpy(wifi_config.sta.bssid, evt->bssid, sizeof(wifi_config.sta.bssid));
        }
#endif

        memcpy(ssid, evt->ssid, sizeof(evt->ssid));
        memcpy(password, evt->password, sizeof(evt->password));
        ESP_LOGI(TAG, "SSID:%s", ssid);
        ESP_LOGI(TAG, "PASSWORD:%s", password);
        esp_err_t res = nvs_write(&password,&ssid,sizeof(evt->password),sizeof(evt->ssid));
        if (res == ESP_OK)
            ESP_LOGI("NVS_write", "OK");
        else
            ESP_LOGI("NVS_write", "NG");

        if (evt->type == SC_TYPE_ESPTOUCH_V2) {
            ESP_ERROR_CHECK( esp_smartconfig_get_rvd_data(rvd_data, sizeof(rvd_data)) );
            ESP_LOGI(TAG, "RVD_DATA:");
            for (int i=0; i<33; i++) {
                printf("%02x ", rvd_data[i]);
            }
            printf("\n");
        }

        ESP_ERROR_CHECK( esp_wifi_disconnect() );
        ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
        esp_wifi_connect();
    } else if (event_base == SC_EVENT && event_id == SC_EVENT_SEND_ACK_DONE) {
        xEventGroupSetBits(s_wifi_event_group, ESPTOUCH_DONE_BIT);
    }
}

void initialize_wifi(char* ssid,char* pass,int ssid_len,int pass_len) {
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    // ESP_LOGI("WIFI_set", "WIFI_SSID: %s len %d", ssid,ssid_len);
    // ESP_LOGI("WIFI_set", "wifi_pass: %s len %d", pass,pass_len);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));// 注册WiFi事件处理程序
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));// 注册IP事件处理程序

    // wifi_config_t wifi_config = {
    //     .sta = {
    //         .ssid = "lb",
    //         .password = "asdewq0608",
    //     },
    // };
    wifi_config_t wifi_config;
    bzero(&wifi_config, sizeof(wifi_config_t));
    memcpy(wifi_config.sta.ssid, ssid, ssid_len);
    memcpy(wifi_config.sta.password, pass, pass_len);
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));// 设置为STA模式
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));// 设置WiFi配置
    ESP_ERROR_CHECK(esp_wifi_start());// 启动WiFi
}

static void initialise_wifi_sc(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    s_wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    assert(sta_netif);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );

    ESP_ERROR_CHECK( esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL) );
    ESP_ERROR_CHECK( esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL) );
    ESP_ERROR_CHECK( esp_event_handler_register(SC_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL) );

    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK( esp_wifi_start() );
}

static void smartconfig_example_task(void * parm)
{
    EventBits_t uxBits;
    ESP_ERROR_CHECK( esp_smartconfig_set_type(SC_TYPE_ESPTOUCH) );
    smartconfig_start_config_t cfg = SMARTCONFIG_START_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_smartconfig_start(&cfg) );
    while (1) {
        uxBits = xEventGroupWaitBits(s_wifi_event_group, CONNECTED_BIT | ESPTOUCH_DONE_BIT, true, false, portMAX_DELAY);
        if(uxBits & CONNECTED_BIT) {
            ESP_LOGI(TAG, "WiFi Connected to ap");
        }
        if(uxBits & ESPTOUCH_DONE_BIT) {
            ESP_LOGI(TAG, "smartconfig over");
            esp_smartconfig_stop();
            vTaskDelete(NULL);
        }
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

    i2c_master_bus_config_t i2c_bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = 0,
        .scl_io_num = SCL_IO_PIN,
        .sda_io_num = SDA_IO_PIN,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, &bus_handle));

    ESP_ERROR_CHECK(ds3231_init_desc(bus_handle,&i2c_ds3231_handle));
    struct tm time;
    ds3231_get_time(&i2c_ds3231_handle, &time);

    setenv("TZ", "CST-8", 1);
    tzset();
    time_t t = mktime(&time); // struct tm -> time_t
    struct timeval now = {
        .tv_sec = t,
        .tv_usec = 0
    };
    settimeofday(&now, NULL);
    print_current_time();

    char *wifi_id,*wifi_pass;
    // nvs_flash_erase();
    esp_err_t err = nvs_flash_init();

    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK( err );


    i2c_mutex = xSemaphoreCreateMutex();
    if (nvs_check_have_write() != ESP_OK)
    {
        ESP_LOGI("WIFI_get_fail", "NO wifi ");
        initialise_wifi_sc();
    }
    else
    {
        nvs_get_PASS(&wifi_pass);
        nvs_get_SSID(&wifi_id);
        ESP_LOGI("WIFI_get", "WIFI_SSID: %s", wifi_id);
        ESP_LOGI("WIFI_get", "wifi_pass: %s", wifi_pass);
        initialize_wifi(wifi_id,wifi_pass,strlen(wifi_id),strlen(wifi_pass));
    }

    xTaskCreate(ntp_task,"ntp_task",4096,NULL,2,NULL);
    xTaskCreate(show_time_task,"show_time_task",4096,NULL,3,NULL);
}
