#include <time.h>
#include <stdbool.h>
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif


#define DS3231_ADDR 0x68 //!< I2C address

#define DS3231_STAT_OSCILLATOR 0x80
#define DS3231_STAT_32KHZ      0x08
#define DS3231_STAT_BUSY       0x04
#define DS3231_STAT_ALARM_2    0x02
#define DS3231_STAT_ALARM_1    0x01

#define DS3231_CTRL_OSCILLATOR    0x80
#define DS3231_CTRL_SQUAREWAVE_BB 0x40
#define DS3231_CTRL_TEMPCONV      0x20
#define DS3231_CTRL_ALARM_INTS    0x04
#define DS3231_CTRL_ALARM2_INT    0x02
#define DS3231_CTRL_ALARM1_INT    0x01

#define DS3231_ALARM_WDAY   0x40
#define DS3231_ALARM_NOTSET 0x80

#define DS3231_ADDR_TIME    0x00
#define DS3231_ADDR_ALARM1  0x07
#define DS3231_ADDR_ALARM2  0x0b
#define DS3231_ADDR_CONTROL 0x0e
#define DS3231_ADDR_STATUS  0x0f
#define DS3231_ADDR_AGING   0x10
#define DS3231_ADDR_TEMP    0x11

#define DS3231_12HOUR_FLAG  0x40
#define DS3231_12HOUR_MASK  0x1f
#define DS3231_PM_FLAG      0x20
#define DS3231_MONTH_MASK   0x1f

#define I2C_FREQ_HZ 400000
#define I2CDEV_TIMEOUT 1000


esp_err_t i2c_dev_read_reg(const i2c_master_dev_handle_t *dev, uint8_t reg, void *in_data, size_t in_size);
esp_err_t i2c_dev_write_reg(const i2c_master_dev_handle_t *dev, uint8_t reg, const void *out_data, size_t out_size);


uint8_t bcd2dec(uint8_t val);
uint8_t dec2bcd(uint8_t val);

esp_err_t ds3231_init_desc(i2c_master_bus_handle_t bus_handle,i2c_master_dev_handle_t *handle);
esp_err_t ds3231_set_time(i2c_master_dev_handle_t *dev, struct tm *time);
esp_err_t ds3231_get_raw_temp(i2c_master_dev_handle_t *dev, int16_t *temp);
esp_err_t ds3231_get_temp_integer(i2c_master_dev_handle_t *dev, int8_t *temp);
esp_err_t ds3231_get_temp_float(i2c_master_dev_handle_t *dev, float *temp);
esp_err_t ds3231_get_time(i2c_master_dev_handle_t *dev, struct tm *time);


#ifdef __cplusplus
}
#endif
