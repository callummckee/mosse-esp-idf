idf_component_register(SRCS "main.cpp" "mosse.cpp" "server.cpp" 
                    INCLUDE_DIRS "."
                    EMBED_FILES "front_end/index.html" "front_end/app.js"
                    REQUIRES esp32-camera nvs_flash bootloader_support esp_http_server esp_wifi esp_netif driver esp_timer)
