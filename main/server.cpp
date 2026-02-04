#include "server.h"
#include "config.h"
#include "wifi_credentials.h"
#include "mosse.h"

#include "esp_jpeg_common.h"
#include "esp_jpeg_enc.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include <cJSON.h>
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/task.h"
#include "img_converters.h"
#include "esp_netif_types.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_wifi_default.h"
#include "mdns.h"
#include <cstdint>
#include <cstdio>
#include "esp_camera.h"

#define MIN(i, j) (((i) < (j)) ? (i) : (j))

static const char* TAG = "server.cpp";

extern const uint8_t index_html_start[] asm("_binary_index_html_start");
extern const uint8_t index_html_end[] asm("_binary_index_html_end");
extern const uint8_t app_js_start[] asm("_binary_app_js_start");
extern const uint8_t app_js_end[] asm("_binary_app_js_end");

JPEGEnc::JPEGEnc() {
    // will have to throw exceptions to properly handle errors here, happy to fail loudly for now
    cfg.width = FRAME_WIDTH;
    cfg.height = FRAME_HEIGHT;
    cfg.src_type = JPEG_PIXEL_FORMAT_GRAY;
    cfg.subsampling = JPEG_SUBSAMPLE_GRAY;
    cfg.quality = 40;
    cfg.rotate = JPEG_ROTATE_0D;
    cfg.task_enable = false;
    cfg.hfm_task_priority = 13;
    cfg.hfm_task_core = 1;

    jpeg_error_t err = jpeg_enc_open(&cfg, &jpeg_enc);
    if (err != JPEG_ERR_OK) {
        ESP_LOGE(TAG, "jpeg_enc_open failed with err: %d", err);
    }

    outbuf = (uint8_t*)calloc(1, image_size);
    if (outbuf == NULL) {
        ESP_LOGE(TAG, "failed to calloc outbuf"); 
    }

}

JPEGEnc::~JPEGEnc() {
    jpeg_enc_close(jpeg_enc); 
    if (outbuf) {
        free(outbuf);
    }
}

jpeg_error_t JPEGEnc::encode(uint8_t* inbuf) {
    jpeg_error_t err = jpeg_enc_process(jpeg_enc, inbuf, image_size, outbuf, image_size, &out_len);
    return err;
}

Server::Server() {
    pp.pingpong_lock = xSemaphoreCreateMutex();
    if (pp.pingpong_lock == NULL) {
        ESP_LOGE("server", "failed to create frame_lock semaphore");
    }
    pp.buffers[0] = (uint8_t*)heap_caps_malloc(FRAME_WIDTH * FRAME_HEIGHT, MALLOC_CAP_INTERNAL);
    pp.buffers[1] = (uint8_t*)heap_caps_malloc(FRAME_WIDTH * FRAME_HEIGHT, MALLOC_CAP_INTERNAL);
    if (!pp.buffers[0] || !pp.buffers[1]) {
        ESP_LOGE(TAG, "failed to allocate pp buffer mem");
    }
}

Server::~Server() {
    free(pp.buffers[0]);
    free(pp.buffers[1]);
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
    //    config.max_req_hdr_len = 4096;

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


void Server::update_frame(camera_fb_t* fb) {
    memcpy(pp.buffers[pp.write_index], fb->buf, fb->len);
    if (!pp.reader_busy) {
        if (xSemaphoreTake(pp.pingpong_lock, portMAX_DELAY)) {
            int temp = pp.write_index;
            pp.write_index = pp.read_index;
            pp.read_index = temp;
            xSemaphoreGive(pp.pingpong_lock);
        }
    }
}

void Server::send_target() {
    if (this->target_fd != -1) {
        ESP_LOGI(TAG, "in send_target");
        httpd_queue_work(this->serverhandle, this->ws_send_target, this);
    }
}

void Server::send_frame() {
    if (this->stream_fd != -1) {
        pp.reader_busy = true;
        httpd_queue_work(this->serverhandle, this->ws_send_stream, this);
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

esp_err_t Server::stream_socket_handler_tramp(httpd_req_t* req) {
    Server* self = static_cast<Server*>(req->user_ctx);

    if(!self) {
        return ESP_FAIL;
    }
    return self->stream_socket_handler(req);
}

esp_err_t Server::target_socket_handler_tramp(httpd_req_t* req) {
    Server* self = static_cast<Server*>(req->user_ctx);

    if(!self) {
        return ESP_FAIL;
    }
    return self->target_socket_handler(req);
}

esp_err_t Server::stream_socket_handler(httpd_req_t* req) {
    if (req->method == HTTP_GET) {
        this->stream_fd = httpd_req_to_sockfd(req);
        ESP_LOGI(TAG, "connected on fd: %d", this->stream_fd);
    }     
    return ESP_OK;
}

esp_err_t Server::target_socket_handler(httpd_req_t* req) {
    if (req->method == HTTP_GET) {
        this->target_fd = httpd_req_to_sockfd(req);
        ESP_LOGI(TAG, "connected on fd: %d", this->target_fd);
        return ESP_OK;
    }     
    uint8_t *buf = NULL;
    httpd_ws_frame_t ws_pkt;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.type = HTTPD_WS_TYPE_BINARY;

    esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "httpd_ws_recv_frame failed to get frame len with %d", ret);
        return ret;
    }
    ESP_LOGI(TAG, "frame len is %d", ws_pkt.len);
    if (ws_pkt.len) {
        buf = (uint8_t*)heap_caps_malloc(ws_pkt.len, MALLOC_CAP_SPIRAM);
        if (!buf) {
            ESP_LOGE(TAG, "Failed to allocate memory for buf");
            return ESP_ERR_NO_MEM;
        }
        ws_pkt.payload = buf;
        /* Set max_len = ws_pkt.len to get the frame payload */
        ret = httpd_ws_recv_frame(req, &ws_pkt, ws_pkt.len);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "httpd_ws_recv_frame failed with %d", ret);
            free(buf);
            return ret;
        }
        tracker.target.rows = buf[0];
        ESP_LOGI(TAG, "setting target.rows: %d", tracker.target.rows);
        tracker.target.cols = buf[1];
        ESP_LOGI(TAG, "setting target.cols: %d", tracker.target.cols);
        tracker.updateTarget(&(buf[2]), ws_pkt.len - 2);
        xTaskNotifyGive(this->tracker.transformationTaskHandle);
        free(buf);
    }
    return ESP_OK;
}


void Server::ws_send_stream(void *arg) {
    Server* self = (Server *)arg;
    uint8_t* data = NULL;
    if (xSemaphoreTake(self->pp.pingpong_lock, portMAX_DELAY)) {
        data = self->pp.buffers[self->pp.read_index];
        xSemaphoreGive(self->pp.pingpong_lock);
    } //check to see if data is NULL?
    self->pp.reader_busy = false;
    httpd_ws_frame_t ws_pkt = {};
    if (self->jpegenc.encode(data) != JPEG_ERR_OK) {
        return;
    }
    ws_pkt.payload = self->jpegenc.outbuf;
    ws_pkt.len = self->jpegenc.out_len; 
    ws_pkt.type = HTTPD_WS_TYPE_BINARY;

    if (self->stream_fd != -1) {
        int64_t start = esp_timer_get_time();
        esp_err_t err = httpd_ws_send_frame_async(self->serverhandle, self->stream_fd, &ws_pkt);
        int64_t end = esp_timer_get_time();
        if (end - start > 10000) {
            ESP_LOGW(TAG, "slow send took %lld us", end - start);
        }
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "ws fatal error: %s", esp_err_to_name(err));
            self->stream_fd = -1;
        }
    }
}


void Server::ws_send_target(void* arg) {
    Server* self = (Server *)arg;
    Image* data;

    xSemaphoreTake(self->tracker.target_lock, portMAX_DELAY);
    data = &self->tracker.target;
    xSemaphoreGive(self->tracker.target_lock);
    size_t len = data->rows * data->cols;
    ESP_LOGI(TAG, "sending target of len: %d", len);
    uint8_t* packet = (uint8_t*)heap_caps_malloc(len + 5, MALLOC_CAP_SPIRAM);
    if(!packet) {
        ESP_LOGE(TAG, "error allocating packet");
        return;
    }

    packet[0] = data->rows;
    packet[1] = data->cols;

    memcpy(&packet[2], data->data, len); 
    httpd_ws_frame_t ws_pkt = {};
    ws_pkt.payload = packet;
    ws_pkt.len = len + 2; 
    ws_pkt.type = HTTPD_WS_TYPE_BINARY;

    if (self->target_fd != -1) {
        esp_err_t err = httpd_ws_send_frame_async(self->serverhandle, self->target_fd, &ws_pkt);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "ws fatal error: %s", esp_err_to_name(err));
            self->target_fd = -1;
        }
    }
}

