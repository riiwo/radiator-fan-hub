idf_component_register(SRCS "hub_console.cpp" INCLUDE_DIRS "include"
                       REQUIRES esp_driver_usb_serial_jtag fan_bus hub_mode probes)
