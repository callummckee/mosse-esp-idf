#include "config.h"

#include "esp_err.h"
#include "esp_log.h"
#include "nvs.h"

static const char *TAG = "config.cpp";

esp_err_t Config::loadConfig() {
    nvs_handle_t handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "error opening nvs handle: %s", esp_err_to_name(err));
        return err;
    }
    size_t required_size = 0;
    float pan_kp; 
    err = nvs_get_str(handle, "pan_kp", NULL, &required_size);
    if (err == ESP_OK) {
        char* pan_kp_str = (char*) malloc(required_size);
        err = nvs_get_str(handle, "pan_kp", pan_kp_str, &required_size);
        if (err == ESP_OK) {
            pan_kp = strtof(pan_kp_str, NULL);
            syscfg.pan_kp = pan_kp;
        }
        free(pan_kp_str);
    }
    else {
        ESP_LOGE(TAG, "error fetching pan_kp str from nvs, error: %s", esp_err_to_name(err));
    }
    float pan_kd; 
    err = nvs_get_str(handle, "pan_kd", NULL, &required_size);
    if (err == ESP_OK) {
        char* pan_kd_str = (char*) malloc(required_size);
        err = nvs_get_str(handle, "pan_kd", pan_kd_str, &required_size);
        if (err == ESP_OK) {
            pan_kd = strtof(pan_kd_str, NULL);
            syscfg.pan_kd = pan_kd;
        }
        free(pan_kd_str);
    }
    else {
        ESP_LOGE(TAG, "error fetching pan_kd str from nvs, error: %s", esp_err_to_name(err));
    }
    float pan_ki; 
    err = nvs_get_str(handle, "pan_ki", NULL, &required_size);
    if (err == ESP_OK) {
        char* pan_ki_str = (char*) malloc(required_size);
        err = nvs_get_str(handle, "pan_ki", pan_ki_str, &required_size);
        if (err == ESP_OK) {
            pan_ki = strtof(pan_ki_str, NULL);
            syscfg.pan_ki = pan_ki;
        }
        free(pan_ki_str);
    }
    else {
        ESP_LOGE(TAG, "error fetching pan_ki str from nvs, error: %s", esp_err_to_name(err));
    }
    float tilt_kd; 
    err = nvs_get_str(handle, "tilt_kd", NULL, &required_size);
    if (err == ESP_OK) {
        char* tilt_kd_str = (char*) malloc(required_size);
        err = nvs_get_str(handle, "tilt_kd", tilt_kd_str, &required_size);
        if (err == ESP_OK) {
            tilt_kd = strtof(tilt_kd_str, NULL);
            syscfg.tilt_kd = tilt_kd;
        }
        free(tilt_kd_str);
    }
    else {
        ESP_LOGE(TAG, "error fetching tilt_kd str from nvs, error: %s", esp_err_to_name(err));
    }
    float tilt_ki; 
    err = nvs_get_str(handle, "tilt_ki", NULL, &required_size);
    if (err == ESP_OK) {
        char* tilt_ki_str = (char*) malloc(required_size);
        err = nvs_get_str(handle, "tilt_ki", tilt_ki_str, &required_size);
        if (err == ESP_OK) {
            tilt_ki = strtof(tilt_ki_str, NULL);
            syscfg.tilt_ki = tilt_ki;
        }
        free(tilt_ki_str);
    }
    else {
        ESP_LOGE(TAG, "error fetching tilt_ki str from nvs, error: %s", esp_err_to_name(err));
    }
    float tilt_kp; 
    err = nvs_get_str(handle, "tilt_kp", NULL, &required_size);
    if (err == ESP_OK) {
        char* tilt_kp_str = (char*) malloc(required_size);
        err = nvs_get_str(handle, "tilt_kp", tilt_kp_str, &required_size);
        if (err == ESP_OK) {
            tilt_kp = strtof(tilt_kp_str, NULL);
            syscfg.tilt_kp = tilt_kp;
        }
        free(tilt_kp_str);
    }
    else {
        ESP_LOGE(TAG, "error fetching tilt_kp str from nvs, error: %s", esp_err_to_name(err));
    }
    nvs_close(handle);
    return err;
}

void Config::saveConfigToNVS() {
    nvs_handle_t handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error (%s) opening NVS handle", esp_err_to_name(err));
        return;
    }

    char buf[100];
    sprintf(buf, "%.4f", syscfg.pan_kp);
    ESP_LOGI(TAG, "saving %s to nvs", buf);
    err = nvs_set_str(handle, "pan_kp", buf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_set_str error: %s", esp_err_to_name(err));
    }

    sprintf(buf, "%.4f", syscfg.pan_kd);
    ESP_LOGI(TAG, "saving %s to nvs", buf);
    err = nvs_set_str(handle, "pan_kd", buf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_set_str error: %s", esp_err_to_name(err));
    }

    sprintf(buf, "%.4f", syscfg.tilt_kd);
    ESP_LOGI(TAG, "saving %s to nvs", buf);
    err = nvs_set_str(handle, "tilt_kd", buf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_set_str error: %s", esp_err_to_name(err));
    }

    sprintf(buf, "%.4f", syscfg.tilt_kp);
    ESP_LOGI(TAG, "saving %s to nvs", buf);
    err = nvs_set_str(handle, "tilt_kp", buf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_set_str error: %s", esp_err_to_name(err));
    }
    sprintf(buf, "%.4f", syscfg.pan_ki);
    ESP_LOGI(TAG, "saving %s to nvs", buf);
    err = nvs_set_str(handle, "pan_ki", buf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_set_str error: %s", esp_err_to_name(err));
    }

    sprintf(buf, "%.4f", syscfg.tilt_ki);
    ESP_LOGI(TAG, "saving %s to nvs", buf);
    err = nvs_set_str(handle, "tilt_ki", buf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_set_str error: %s", esp_err_to_name(err));
    }

    err = nvs_commit(handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Commit Error: %s", esp_err_to_name(err));
    }

    nvs_close(handle);
}
