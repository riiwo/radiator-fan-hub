idf_component_register(SRCS "xiao_alone.cpp" "xiao_with_attiny.cpp" INCLUDE_DIRS "include"
                       REQUIRES attiny_link fan_bus fan_curve hub_board hub_button hub_ha hub_led hub_mode probes)
