
#include "nvs_time.h"

static char *str ="NVS";

esp_err_t nvs_check_have_write()
{
    esp_err_t err;
    nvs_handle_t my_handle;
    // Open
    err = nvs_open(str, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) return err;

    // Read the size of memory space required for blob
    size_t required_size = 0;  // value will default to 0, if not set yet in NVS
    err = nvs_get_blob(my_handle, "WIFI_PASS", NULL, &required_size);

    if (err != ESP_OK) return err;

   required_size = 0;  // value will default to 0, if not set yet in NVS
    err = nvs_get_blob(my_handle, "WIFI_SSID", NULL, &required_size);

    // Close
    nvs_close(my_handle);
    return err;
}


esp_err_t nvs_write(char* WIFI_PASS,char* WIFI_SSID,int size_WIFI_PASS,int size_WIFI_SSID)
{
    esp_err_t err;
    nvs_handle_t my_handle;
    // Open
    err = nvs_open(str, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) return err;

    err = nvs_set_blob(my_handle, "WIFI_PASS", WIFI_PASS, size_WIFI_PASS);
    if (err != ESP_OK) return err;
    err = nvs_set_blob(my_handle, "WIFI_SSID", WIFI_SSID, size_WIFI_SSID);
    if (err != ESP_OK) return err;
    // Commit
    err = nvs_commit(my_handle);
    if (err != ESP_OK) return err;
    // Close
    nvs_close(my_handle);
    return ESP_OK;
}

esp_err_t nvs_get_PASS(char** WIFI_PASS)
{
    esp_err_t err;
    nvs_handle_t my_handle;

    // Open
    err = nvs_open(str, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) return err;

    // Read the size of memory space required for blob
    size_t size = 0;  // value will default to 0, if not set yet in NVS
    err = nvs_get_blob(my_handle, "WIFI_PASS", NULL, &size);

    if (err != ESP_OK) return err;
    *WIFI_PASS = malloc(size);
    // Read previously saved blob if available
    if (size > 0) {
        err = nvs_get_blob(my_handle, "WIFI_PASS", *WIFI_PASS, &size);
        if (err != ESP_OK) {
            return err;
        }
    }
    ESP_LOGI(str, "WIFI_PASS: %s", *WIFI_PASS);
    return err;
}

esp_err_t nvs_get_SSID(char** WIFI_SSID)
{
    esp_err_t err;
    nvs_handle_t my_handle;

    // Open
    err = nvs_open(str, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) return err;

    // Read the size of memory space required for blob
    size_t size = 0;  // value will default to 0, if not set yet in NVS
    err = nvs_get_blob(my_handle, "WIFI_SSID", NULL, &size);

    if (err != ESP_OK) return err;
    *WIFI_SSID = malloc(size);
    // Read previously saved blob if available
    if (size > 0) {
        err = nvs_get_blob(my_handle, "WIFI_SSID", *WIFI_SSID, &size);
        if (err != ESP_OK) {
            return err;
        }
    }
    ESP_LOGI(str, "WIFI_SSID: %s", *WIFI_SSID);
    return err;
}
