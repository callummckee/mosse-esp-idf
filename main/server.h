#pragma once

#include "freertos/idf_additions.h"
#include "esp_http_server.h"
#include "esp_dsp.h"
#include "mosse.h"
#include "config.h"

void start_mdns();

class Server {
    public:
    Server();
    ~Server();
    void wifi_init_sta();
    void server_init(uint16_t ctrl_port, uint16_t port);
    void update_transformations_billboard(const Image* affine_transformations);

    static void event_handler_tramp(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
    static esp_err_t app_js_handler_tramp(httpd_req_t* req);
    static esp_err_t socket_handler_tramp(httpd_req_t* req); 
    static esp_err_t page_handler_tramp(httpd_req_t* req);

    esp_err_t register_handler(const char* uri, httpd_method_t method, esp_err_t (*handler_trampoline)(httpd_req_t*), bool websocket = false);

    int client_fd = -1;
    private:
    uint8_t* m_pixel_blob = NULL;
    Image transformations_billboard[9] = {};
    SemaphoreHandle_t frame_lock;
    httpd_handle_t serverhandle = nullptr;
    
    static const int WIFI_CONNECTED_BIT = BIT0;
    static const int WIFI_FAIL_BIT = BIT1;
    EventGroupHandle_t s_wifi_event_group;
    
    void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
    esp_err_t app_js_handler(httpd_req_t *req);
    esp_err_t socket_handler(httpd_req_t* req);
    esp_err_t page_handler(httpd_req_t* req);

    static void ws_send_images(void *arg);
};
