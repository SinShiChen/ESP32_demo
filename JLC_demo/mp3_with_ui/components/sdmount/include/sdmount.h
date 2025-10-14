#include <stdio.h>
#include <string.h>
#include <dirent.h>
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"

#ifdef __cplusplus
extern "C" {
#endif

/* SD card GPIOs */
#define EXAMPLE_SD_CMD_IO      (48) 
#define EXAMPLE_SD_CLK_IO      (47)
#define EXAMPLE_SD_DAT0_IO     (21)

#define EXAMPLE_SD_MOUNT_POINT     "/sdcard"

sdmmc_card_t * mount_sdcard(void);
static esp_err_t s_example_write_file(const char *path, char *data);
static esp_err_t s_example_read_file(const char *path);
void list_files(const char *path);

#ifdef __cplusplus
}
#endif
