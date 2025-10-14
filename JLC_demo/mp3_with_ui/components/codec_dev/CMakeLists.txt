idf_component_register(SRCS "codec_dev.c"
                    PRIV_REQUIRES driver
                    INCLUDE_DIRS "include"
                    REQUIRES 
                        esp_codec_dev
                        chmorgan__esp-file-iterator 
                        chmorgan__esp-audio-player
                        sdmount
                        PCA9557
                    )
