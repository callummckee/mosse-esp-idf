#include "server.h"
#include "config.h"
#include "wifi_credentials.h"
#include "mosse.h"

#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/task.h"
#include "img_converters.h"
#include "esp_netif_types.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_wifi_default.h"
#include "esp_wifi_types_generic.h"
#include "mdns.h"
#include <cstdio>

#define PART_BOUNDARY "123456789000000000000987654321"
const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

static const char* TAG = "server.cpp";

extern const uint8_t index_html_start[] asm("_binary_index_html_start");
extern const uint8_t index_html_end[] asm("_binary_index_html_end");
extern const uint8_t app_js_start[] asm("_binary_app_js_start");
extern const uint8_t app_js_end[] asm("_binary_app_js_end");

Server::Server() {
    frame_lock = xSemaphoreCreateMutex();
    if (frame_lock == NULL) {
        ESP_LOGE("server", "failed to create frame_lock semaphore");
    }

    size_t img_bytes = FRAME_WIDTH * FRAME_HEIGHT;
    m_pixel_blob = (uint8_t*)heap_caps_malloc(img_bytes * 9, MALLOC_CAP_SPIRAM);
    if (!m_pixel_blob) {
        ESP_LOGE("server", "couldn't allocate m_pixel_blob aborting");
        return;
    }
    for (int i = 0; i < 9; i++) {
        transformations_billboard[i].data = m_pixel_blob + (i * img_bytes);
        transformations_billboard[i].rows = FRAME_HEIGHT;
        transformations_billboard[i].cols = FRAME_WIDTH;
    }

}

Server::~Server() {
    free(m_pixel_blob);
}


void Server::wifi_init_sta() {
    this->s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler_tramp, this, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler_tramp, this, &instance_got_ip));

    wifi_config_t wifi_config = {};

    strcpy((char*)wifi_config.sta.ssid, WIFI_SSID);
    strcpy((char*)wifi_config.sta.password, WIFI_PASS);

    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    wifi_config.sta.pmf_cfg.capable = true;
    wifi_config.sta.pmf_cfg.required = false;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_sta finished");

    xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
}

void start_mdns() {
    esp_err_t err = mdns_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "mdns init failed: %d", err);
        return;
    }

    mdns_hostname_set("mosse");

    ESP_LOGI(TAG, "mdns started, access http://mosse.local\n");
}


void Server::event_handler_tramp(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    Server* self = static_cast<Server*>(arg);
    if (self) {
        self->event_handler(arg, event_base, event_id, event_data);
    }
}


void Server::event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();
        ESP_LOGI(TAG, "retry connection");
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

esp_err_t Server::register_handler(const char* uri, httpd_method_t method, esp_err_t (*handler_trampoline)(httpd_req_t*), bool websocket /*= false*/) 
{
    const httpd_uri_t handler = {
        .uri = uri,
        .method = method,
        .handler = handler_trampoline,
        .user_ctx = this,
        .is_websocket = websocket,
        .handle_ws_control_frames = false,
        .supported_subprotocol = NULL
    };
    if (httpd_register_uri_handler(this->serverhandle, &handler) == ESP_OK) {
        return ESP_OK;
    }
    else {
        ESP_LOGE(TAG, "handler registration %s failed", uri);
        return ESP_FAIL;
    }

}

void Server::server_init(uint16_t ctrl_port, uint16_t port)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = port;
    config.lru_purge_enable = true;
    config.ctrl_port = ctrl_port;
    config.max_req_hdr_len = 4096;

    ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {
        ESP_LOGI(TAG, "server online");
        this->serverhandle = server;
    }
    else {
        ESP_LOGE(TAG, "error starting server with port %d", port);
        this->serverhandle = server;
    }
}


esp_err_t Server::page_handler_tramp(httpd_req_t* req) {
    Server* self = static_cast<Server*>(req->user_ctx);

    if(!self) {
        return ESP_FAIL;
    }
    else {
        return self->page_handler(req);
    }

}

esp_err_t Server::page_handler(httpd_req_t* req) {
    const size_t html_len = index_html_end - index_html_start;

    httpd_resp_send(req, (const char*)index_html_start, html_len);
    return ESP_OK;
}

void Server::update_transformations_billboard(const Image* affines) {
    if (xSemaphoreTake(frame_lock, portMAX_DELAY)) {
        size_t img_size = FRAME_WIDTH * FRAME_HEIGHT;
        for (int i = 0; i < 9; i ++) {
            memcpy(transformations_billboard[i].data, affines[i].data, img_size);
            transformations_billboard[i].rows = affines[i].rows;
            transformations_billboard[i].cols = affines[i].cols;

        }
        xSemaphoreGive(frame_lock);
    }
    if (this->client_fd != -1) {
        httpd_queue_work(this->serverhandle, this->ws_send_images, this);
    }
}


esp_err_t Server::app_js_handler_tramp(httpd_req_t* req)
{
    Server* self = static_cast<Server*>(req->user_ctx);
    if(!self) {
        return ESP_FAIL;
    }
    else {
        return self->app_js_handler(req);
    }
}

esp_err_t Server::app_js_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/javascript");
    const size_t js_len = app_js_end - app_js_start;
    httpd_resp_send(req, (const char *)app_js_start, js_len);
    return ESP_OK;
}

esp_err_t Server::socket_handler_tramp(httpd_req_t* req) {
    Server* self = static_cast<Server*>(req->user_ctx);

    if(!self) {
        return ESP_FAIL;
    }
    return self->socket_handler(req);
}

esp_err_t Server::socket_handler(httpd_req_t* req) {
    if (req->method == HTTP_GET) {
        this->client_fd = httpd_req_to_sockfd(req);
        ESP_LOGI(TAG, "connected on fd: %d", this->client_fd);
        return ESP_OK;
    } 
    httpd_ws_frame_t ws_pkt;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;

    httpd_ws_recv_frame(req, &ws_pkt, 0);

    return ESP_OK;
}

void Server::ws_send_images(void *arg) {
    Server* self = (Server *)arg;
    Image* data;

    xSemaphoreTake(self->frame_lock, portMAX_DELAY);
    data = &(self->transformations_billboard)[0];
    xSemaphoreGive(self->frame_lock);
    size_t len = data[0].cols * data[0].rows;
    uint8_t* packet = (uint8_t*)heap_caps_malloc(len + 5, MALLOC_CAP_SPIRAM);

    if(!packet) {
        ESP_LOGE(TAG, "error allocating packet");
        return;
    }

    for (int i = 0; i < 9; i++) {
        packet[0] = i;
        packet[1] = (len >> 24) & 0xFF;
        packet[2] = (len >> 16) & 0xFF;
        packet[3] = (len >> 8) & 0xFF;
        packet[4] = len & 0xFF;

        memcpy(&packet[5], data[i].data, len); 
        httpd_ws_frame_t ws_pkt = {};
        ws_pkt.payload = packet;
        ws_pkt.len = len + 5; 
        ws_pkt.type = HTTPD_WS_TYPE_BINARY;

        if (self->client_fd != -1) {
            esp_err_t err = httpd_ws_send_frame_async(self->serverhandle, self->client_fd, &ws_pkt);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "ws fatal error: %s", esp_err_to_name(err));
                self->client_fd = -1;
            }
        }
    }
    free(packet);
}
