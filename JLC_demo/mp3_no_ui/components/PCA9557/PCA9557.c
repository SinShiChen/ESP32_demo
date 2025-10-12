#include "PCA9557.h"



i2c_master_dev_handle_t i2c_pca9557_handle ;
esp_err_t bsp_i2c_init(i2c_master_bus_handle_t *bus_handle)
{
    i2c_master_bus_config_t i2c_bus_config = {
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .i2c_port = 0,
    .scl_io_num = TEST_BOARD_I2C_SCL_PIN,
    .sda_io_num = TEST_BOARD_I2C_SDA_PIN,
    .glitch_ignore_cnt = 7,
    .flags.enable_internal_pullup = true,
    };
    return i2c_new_master_bus(&i2c_bus_config, bus_handle);
}

esp_err_t bsp_i2c_deinit(i2c_master_bus_handle_t *bus_handle)
{
   if (*bus_handle) {
       i2c_del_master_bus(*bus_handle);
   }
   *bus_handle = NULL;
   return 0;
}

// 读取PCA9557寄存器的值
esp_err_t pca9557_register_read(i2c_master_dev_handle_t *handle,uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(*handle, &reg_addr, 1, data, len, -1);
}

// 给PCA9557的寄存器写值
esp_err_t pca9557_register_write_byte(i2c_master_dev_handle_t *handle,uint8_t reg_addr, uint8_t data)
{
    uint8_t write_buf[2] = {reg_addr, data};
    return i2c_master_transmit(*handle, write_buf, 2,-1);
}


// 初始化PCA9557 IO扩展芯片
esp_err_t pca9557_init(i2c_master_bus_handle_t bus_handle,i2c_master_dev_handle_t *handle)
{

    // CHECK_ARG(bus_handle);
    esp_err_t ret = ESP_OK;

    i2c_device_config_t i2c_dev_conf = {
        .scl_speed_hz = BSP_I2C_FREQ_HZ,
        .device_address = PCA9557_SENSOR_ADDR,
    };

    ret = i2c_master_bus_add_device(bus_handle, &i2c_dev_conf, handle);
    if (ret != ESP_OK)
        return ret;
    
    // 写入控制引脚默认值 DVP_PWDN=1  PA_EN = 0  LCD_CS = 1
    ret = pca9557_register_write_byte(handle,PCA9557_OUTPUT_PORT, 0x05);  
    if (ret != ESP_OK)
        return ret;
    // 把PCA9557芯片的IO1 IO1 IO2设置为输出 其它引脚保持默认的输入
    ret = pca9557_register_write_byte(handle,PCA9557_CONFIGURATION_PORT, 0xf8); 
    return ret;
    
}

// 设置PCA9557芯片的某个IO引脚输出高低电平
esp_err_t pca9557_set_output_state(i2c_master_dev_handle_t *handle,uint8_t gpio_bit, uint8_t level)
{
    uint8_t data;
    esp_err_t res = ESP_FAIL;

    pca9557_register_read(handle,PCA9557_OUTPUT_PORT, &data, 1);
    res = pca9557_register_write_byte(handle,PCA9557_OUTPUT_PORT, SET_BITS(data, gpio_bit, level));

    return res;
}

// 控制 PCA9557_PA_EN 引脚输出高低电平 参数0输出低电平 参数1输出高电平 
void pa_en(i2c_master_dev_handle_t *handle,uint8_t level)
{
    pca9557_set_output_state(handle,PA_EN_GPIO, level);
}
