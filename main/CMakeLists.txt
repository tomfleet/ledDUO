idf_component_register(SRCS "espnow_example_main.c"
                    PRIV_REQUIRES driver esp_timer esp_event esp_netif esp_wifi esp_http_server nvs_flash
                    INCLUDE_DIRS ".")
