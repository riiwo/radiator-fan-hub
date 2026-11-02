idf_component_register(SRCS "ha_link.cpp" INCLUDE_DIRS "include"
                       REQUIRES esp_wifi esp_netif esp_event esp_http_client app_update esp_timer)
