idf_component_register(SRCS "sdmount.c"
                    PRIV_REQUIRES driver
                    INCLUDE_DIRS "include"
                    REQUIRES fatfs)
