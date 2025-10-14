idf_component_register(SRCS "spi_lcd_touch_example_main.c" "GenShinGothic_ExtraLight_2.c" "app_ui.c"
                    PRIV_REQUIRES spi_flash   
                    INCLUDE_DIRS "."
                        REQUIRES 
                        driver 
                        unity 
                        PCA9557 
                        sdmount
                        codec_dev)
