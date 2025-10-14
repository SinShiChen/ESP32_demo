idf_component_register(SRCS "siyuan.c" "spi_lcd_touch_example_main.c" "lvgl_demo_ui.c" "app_ui.c"
                    PRIV_REQUIRES spi_flash   
                    INCLUDE_DIRS "."
                        REQUIRES 
                        driver 
                        unity 
                        PCA9557 
                        sdmount
                        codec_dev)
