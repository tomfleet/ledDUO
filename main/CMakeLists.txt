idf_component_register(SRCS "espnow_example_main.c" "led_ws2812.c" "espnow_link.c" "game_logic.c" "imu_driver.c" "web_dashboard.c" "captive_portal.c"
                    PRIV_REQUIRES driver esp_timer esp_event esp_netif esp_wifi esp_http_server spiffs nvs_flash
                    INCLUDE_DIRS ".")

spiffs_create_partition_image(spiffs ../spiffs FLASH_IN_PROJECT)
