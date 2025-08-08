#include <stdint.h>
#include "driver/i2c_master.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif
#define MPU6050_SENSOR_ADDR                 0x68        /*!< Slave address of the MPU9250 sensor */
#define MPU6050_WHO_AM_I_REG_ADDR           0x75        /*!< Register addresses of the "who am I" register */
#define MPU6050_ACC_X_ADDR              0x3B
#define MPU6050_ACC_Y_ADDR              0X3D
#define MPU6050_ACC_Z_ADDR              0X3F
#define MPU6050_GYR_X_ADDR              0X43
#define MPU6050_GYR_Y_ADDR              0X45
#define MPU6050_GYR_Z_ADDR              0X47
#define MPU6050_TEMP                    0x41

#define MPU6050_PWR_MGMT_1_REG_ADDR         0x6B        /*!< Register addresses of the power managment register */
#define MPU6050_RESET_BIT                   7
#define MPU6050_SLEEP_BIT                   6

#define MPU6050_GYR_FS_ADDR             0x1B
#define MPU6050_ACC_FS_ADDR             0x1C
#define MPU6050_FS_BIT                  3

#define	SMPLRT_DIV		0x19	//陀螺仪采样率，典型值：0x07(125Hz)
#define	CONFIG			0x1A	//低通滤波频率，典型值：0x06(5Hz)
#define	GYRO_CONFIG		0x1B	//陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
#define	ACCEL_CONFIG	0x1C	//加速计自检、测量范围及高通滤波频率，典型值：0x01(不自检，2G，5Hz)

typedef struct {
i2c_device_config_t icm_device;  /*!< Configuration for eeprom device */
uint8_t write_time_ms;              /*!< eeprom write time, typically 10ms*/
} i2c_icm_config_t;


typedef struct {
    i2c_master_dev_handle_t i2c_dev;      /*!< I2C device handle */
    uint8_t *buffer;                      /*!< I2C transaction buffer */
    uint8_t write_time_ms;                /*!< I2C eeprom write time(ms)*/
}i2c_icm_t;

typedef i2c_icm_t *i2c_icm_handle_t;

esp_err_t i2c_icm20602_init(i2c_master_bus_handle_t bus_handle, const i2c_icm_config_t *icm_config, i2c_icm_handle_t *icm_handle);
esp_err_t i2c_icm20602_init_reg(i2c_icm_handle_t icm_handle);
esp_err_t i2c_icm20602_read(i2c_icm_handle_t icm_handle);

#ifdef __cplusplus
}
#endif
