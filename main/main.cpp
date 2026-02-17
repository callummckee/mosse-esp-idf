#include <stdio.h>
#include "esp_heap_caps.h"
#include "esp_rom_sys.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "esp_dsp.h"
#include "http_parser.h"
#include "mat.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "esp_camera.h"
#include "nvs_flash.h"
#include "esp_timer.h"

#include "randomutils.h"
#include "sensor.h"
#include "tracker.h"
#include "config.h"
#include "server.h"
#include "pins.h"
#include "benchmark.h"

struct SystemContext {
    Server* server;
    Tracker* tracker;
};


static const char* TAG = "main.cpp";

static camera_config_t get_camera_config() {
    camera_config_t config = {};

    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_sccb_sda = SIOD_GPIO_NUM;
    config.pin_sccb_scl = SIOC_GPIO_NUM;

    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;

    config.xclk_freq_hz = 20000000; 

    config.ledc_timer = LEDC_TIMER_1; //servos use timer 1 as they have different clock frequency
    config.ledc_channel = LEDC_CHANNEL_2; //servos use channels 0 and 1

    config.pixel_format = PIXFORMAT_GRAYSCALE;
    config.fb_count = 3;
    config.frame_size = FRAMESIZE_QQVGA;

    config.grab_mode = CAMERA_GRAB_LATEST;
    config.fb_location = CAMERA_FB_IN_PSRAM;

    return config;
}

struct mainBenchmarkData {
    uint64_t updateFilter_cycles = 0;
    uint64_t update_frame_cycles = 0; 
    uint64_t updateTargetPOS_cycles = 0;
    uint64_t send_frame_cycles = 0;

    uint32_t avg_updateFilter = 0;
    uint32_t avg_update_frame = 0;
    uint32_t avg_updateTargetPOS = 0;
    uint32_t avg_send_frame = 0;
    uint32_t frame_count = 0;
};

void camera_task(void* pvParameters){
    mainBenchmarkData mbd;
    SystemContext* sysctx = (SystemContext*)pvParameters;
    Tracker* tracker = sysctx->tracker;
    Server* server = sysctx->server;

    ESP_LOGI(TAG, "starting loop");

    while(1) {
        int64_t start = esp_timer_get_time();
        camera_fb_t* fb = esp_camera_fb_get();
        if (!fb) {
            ESP_LOGE(TAG, "frame buffer could not be acquired");
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }
        if (tracker->isTracking) {
            {
                Benchmarker b(&mbd.updateFilter_cycles);
                tracker->updateFilter(fb->buf);
            }
            {
                Benchmarker b(&mbd.update_frame_cycles);
                server->update_frame(fb);
            }
            {
                Benchmarker b(&mbd.updateTargetPOS_cycles);
                server->updateTargetPOS(tracker->getTargetPOS());
            }
            if (!server->pp.reader_busy) {
                {
                    Benchmarker b(&mbd.send_frame_cycles);
                    server->send_frame();
                }
            }
        }
        else {

            {
                Benchmarker b(&mbd.update_frame_cycles);
                server->update_frame(fb);
            }
            if (!server->pp.reader_busy) {
                {
                    Benchmarker b(&mbd.send_frame_cycles);
                    server->send_frame();
                }
            }
        }
        esp_camera_fb_return(fb);
        int64_t end = esp_timer_get_time();
        float fps = 1.0/((end - start)/1000000.0);
        ESP_LOGI(TAG, "FPS: %f", fps);
        mbd.frame_count++;
        if (mbd.frame_count > 100) {
            mbd.avg_updateFilter = mbd.updateFilter_cycles/100;
            mbd.avg_update_frame = mbd.update_frame_cycles/100;
            mbd.avg_updateTargetPOS = mbd.updateTargetPOS_cycles/100;
            mbd.avg_send_frame = mbd.send_frame_cycles/100;

           mbd.updateFilter_cycles = 0;
           mbd.update_frame_cycles = 0;
           mbd.updateTargetPOS_cycles = 0;
           mbd.send_frame_cycles = 0;
           mbd.frame_count = 0;

           esp_rom_printf("updateFilter: %lu\nupdate_frame: %lu\nupdateTarget: %lu\nsend_frame: %lu\n", mbd.avg_updateFilter, mbd.avg_update_frame, mbd.avg_updateTargetPOS, mbd.avg_send_frame);
        }

        vTaskDelay(1);
    }
}

extern "C" void app_main(void) {
    //initialize nvs for wifi creds
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    camera_config_t config_cam = get_camera_config();
    esp_err_t err_cam = esp_camera_init(&config_cam);
    if (err_cam != ESP_OK) {
        ESP_LOGE(TAG, "Camera Init Failed: 0x%x", err_cam);
        return;
    }

    Tracker* tracker = new Tracker();
    Server* server = new Server(tracker);
    SystemContext* sysctx = new SystemContext;
    sysctx->server = server;
    sysctx->tracker = tracker;

    server->wifi_init_sta();
    start_mdns(); //move this inside a class? 
    server->server_init(32768, 80);
    server->register_handler("/", HTTP_GET, Server::page_handler_tramp);
    server->register_handler("/app.js", HTTP_GET, Server::app_js_handler_tramp); 
    server->register_handler("/stream_ws", HTTP_GET, Server::stream_socket_handler_tramp, true);
    server->register_handler("/target_ws", HTTP_GET, Server::target_socket_handler_tramp, true);

    ESP_LOGI(TAG, "creating camera task");
    xTaskCreatePinnedToCore(camera_task, "Camera Task", 1024 * 8, sysctx, 5, server->xCameraTaskHandle, 1);
    vTaskDelete(NULL); //frees mem assigned to main
}
