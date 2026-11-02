idf_component_register(SRCS "hub_board.cpp" INCLUDE_DIRS "include"
                       REQUIRES esp_driver_gpio esp_hw_support nvs_flash attiny_isp fan_bus hub_button hub_led
                                hub_mode probes)
