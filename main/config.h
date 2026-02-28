#pragma once


#include "idf_additions.h"
#define FRAME_WIDTH 320
#define FRAME_HEIGHT 240
#define MAX_TARGET_WIDTH 64
#define MAX_TARGET_HEIGHT 64

struct PIDConfig {
        float pan_kp;
        float pan_ki;
        float pan_kd;
        float tilt_kp;
        float tilt_ki;
        float tilt_kd;
};

class Config {
    public:
        Config() {
            configMutex = xSemaphoreCreateMutex();
        }
        esp_err_t loadConfig(); 
        void saveConfigToNVS();
        const PIDConfig dfltcfg = {0.05, 0.05, 0.2, 0.05, 0.05, 0.2};
        PIDConfig syscfg = {};
        SemaphoreHandle_t configMutex;
        bool newcfg = false;
};

