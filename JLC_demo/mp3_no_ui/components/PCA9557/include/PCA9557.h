#include "esp_system.h"
#include "esp_log.h"
#include "driver/i2c_master.h"


#ifdef __cplusplus
extern "C" {
#endif

#define TEST_BOARD_I2C_SDA_PIN      (GPIO_NUM_1)
#define TEST_BOARD_I2C_SCL_PIN      (GPIO_NUM_2)

#define PCA9557_INPUT_PORT              0x00
#define PCA9557_OUTPUT_PORT             0x01
#define PCA9557_POLARITY_INVERSION_PORT 0x02
#define PCA9557_CONFIGURATION_PORT      0x03

#define PCA9557_SENSOR_ADDR             0x19        /*!< Slave address of the MPU9250 sensor */
#define SET_BITS(_m, _s, _v)  ((_v) ? (_m)|((_s)) : (_m)&~((_s)))
#define BSP_I2C_FREQ_HZ       100000         // 100kHz

#define LCD_CS_GPIO                 BIT(0)    // PCA9557_GPIO_NUM_1
#define PA_EN_GPIO                  BIT(1)    // PCA9557_GPIO_NUM_2
#define DVP_PWDN_GPIO               BIT(2)    // PCA9557_GPIO_NUM_3

esp_err_t bsp_i2c_init(i2c_master_bus_handle_t *bus_handle);
esp_err_t bsp_i2c_deinit(i2c_master_bus_handle_t *bus_handle);
esp_err_t pca9557_register_read(i2c_master_dev_handle_t *handle,uint8_t reg_addr, uint8_t *data, size_t len);
esp_err_t pca9557_register_write_byte(i2c_master_dev_handle_t *handle,uint8_t reg_addr, uint8_t data);
esp_err_t pca9557_init(i2c_master_bus_handle_t bus_handle,i2c_master_dev_handle_t *handle);
esp_err_t pca9557_set_output_state(i2c_master_dev_handle_t *handle,uint8_t gpio_bit, uint8_t level);
void pa_en(i2c_master_dev_handle_t *handle,uint8_t level);

extern i2c_master_dev_handle_t i2c_pca9557_handle;


#ifdef __cplusplus
}
#endif