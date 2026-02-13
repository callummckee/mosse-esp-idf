#pragma once

#include "esp_jpeg_common.h"
#include "esp_jpeg_enc.h"
#include "freertos/idf_additions.h"
#include "esp_http_server.h"
#include "esp_dsp.h"
#include "esp_camera.h"

#include "tracker.h"
#include "config.h"

void start_mdns();

struct PingPong {
    uint8_t* buffers[2];
    int write_index = 0;
    int read_index = 1;
    SemaphoreHandle_t pingpong_lock;
    bool reader_busy = false;
};

class JPEGEnc {
    private:
        jpeg_enc_config_t cfg = DEFAULT_JPEG_ENC_CONFIG();
        jpeg_enc_handle_t jpeg_enc = NULL;
        size_t image_size = FRAME_HEIGHT * FRAME_WIDTH;

    public:
        JPEGEnc();
        ~JPEGEnc();
        jpeg_error_t encode(uint8_t* inbuf, uint8_t* outbuf);
        int out_len;
};

class Server {
    public:
        Server(Tracker* t);
        ~Server();

        TaskHandle_t* xCameraTaskHandle = NULL;
        
        Tracker* tracker;

        void wifi_init_sta();
        void server_init(uint16_t ctrl_port, uint16_t port);
        void update_frame(camera_fb_t* fb);
        void send_target();
        void send_frame();

        volatile bool frame_processing = false;

        static void event_handler_tramp(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
        static esp_err_t app_js_handler_tramp(httpd_req_t* req);
        static esp_err_t stream_socket_handler_tramp(httpd_req_t* req); 
        static esp_err_t target_socket_handler_tramp(httpd_req_t* req); 
        static esp_err_t page_handler_tramp(httpd_req_t* req);
        static esp_err_t receive_handler_tramp(httpd_req_t* req);

        esp_err_t register_handler(const char* uri, httpd_method_t method, esp_err_t (*handler_trampoline)(httpd_req_t*), bool websocket = false);

        PingPong pp = {};

        void updateTargetPOS(Coord tp);

        int stream_fd = -1;
        int target_fd = -1;
    private:
        JPEGEnc jpegenc = {};
        SemaphoreHandle_t target_lock;
        httpd_handle_t serverhandle = nullptr;

        static const int WIFI_CONNECTED_BIT = BIT0;
        static const int WIFI_FAIL_BIT = BIT1;
        EventGroupHandle_t s_wifi_event_group;

        uint8_t* ws_stream_buf = NULL;

        uint8_t* ws_target_buf = NULL;
        Coord target_pos = {0, 0};
        SemaphoreHandle_t target_pos_lock;

        void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
        esp_err_t app_js_handler(httpd_req_t *req);
        esp_err_t stream_socket_handler(httpd_req_t* req);
        esp_err_t target_socket_handler(httpd_req_t* req);
        esp_err_t page_handler(httpd_req_t* req);
        esp_err_t receive_handler(httpd_req_t* req);

        static void ws_send_stream(void *arg);
        static void ws_send_target(void* arg);
};
