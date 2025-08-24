#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"

#include "esp_mac.h"
#include "esp_sntp.h"
#include "nvs.h"

#ifdef __cplusplus
extern "C" {
#endif



esp_err_t nvs_check_have_write();
esp_err_t nvs_write(char* WIFI_PASS,char* WIFI_SSID,int size_WIFI_PASS,int size_WIFI_SSID);
esp_err_t nvs_get_PASS(char** WIFI_PASS);
esp_err_t nvs_get_SSID(char** WIFI_SSID);

#ifdef __cplusplus
}
#endif
