#include "mosse.h"
#include "config.h"
#include "dspm_mult.h"
#include "esp_heap_caps.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "randomutils.h"
#include <cstdint>
#include <math.h>
#include "esp_log.h"
#include "bootloader_random.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "server.h"
#include "mat.h"

const int theta_mean = 0;
const int theta_sigma = 30;
const int lambda_mean = 1;
const int lambda_sigma = 0.5;
const float x_shift_sigma = 0.15;
const float y_shift_sigma = 0.15;

static const char* TAG = "mosse.cpp";

Tracker::Tracker() {
    target_data = (uint8_t*)heap_caps_malloc(FRAME_WIDTH*FRAME_HEIGHT, MALLOC_CAP_INTERNAL);
    if (!target_data) {
        ESP_LOGE(TAG, "couldn't allocate target_data mem");
    }
    target.data = target_data;
    target_lock = xSemaphoreCreateMutex();
    if (target_lock == NULL) {
        ESP_LOGE(TAG, "failed to create target_lock semaphore");
    }

    size_t img_bytes = FRAME_WIDTH * FRAME_HEIGHT;
    m_pixel_blob = (uint8_t*)heap_caps_malloc(img_bytes * NUM_TRANSFORMATIONS, MALLOC_CAP_SPIRAM);
    if (!m_pixel_blob) {
        ESP_LOGE("server", "couldn't allocate m_pixel_blob");
        return;
    }
    for (int i = 0; i < NUM_TRANSFORMATIONS; i++) {
        transformations[i].data = m_pixel_blob + (i * img_bytes);
        transformations[i].rows = FRAME_HEIGHT;
        transformations[i].cols = FRAME_WIDTH;
    }
    transformations_lock = xSemaphoreCreateMutex();
    if (transformations_lock == NULL) {
        ESP_LOGE(TAG, "failed to create transformations_lock semaphore");
    }
}

Tracker::~Tracker() {
    free(target_data);
    free(m_pixel_blob);
}

void Tracker::updateTarget(uint8_t* payload, size_t len) {
    ESP_LOGI(TAG, "updating target of size: %d", len);
    if(xSemaphoreTake(this->target_lock, portMAX_DELAY)) {
        memcpy(target.data, payload, len);
        xSemaphoreGive(target_lock);
    } 
}

void Tracker::transformationTask_tramp(void* _this) {
    Tracker* self = (Tracker*)_this;
    self->transformationTaskLoop();
}

void Tracker::transformationTaskLoop() {
    while(true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        uint8_t* temp_target_data = NULL; 
        int rows = 0;
        int cols = 0;
        if (xSemaphoreTake(target_lock, portMAX_DELAY)) {
            rows = target.rows;
            cols = target.cols;
            temp_target_data = (uint8_t*)heap_caps_malloc(rows*cols, MALLOC_CAP_INTERNAL);
            memcpy(temp_target_data, target.data, rows*cols);
            xSemaphoreGive(target_lock);
        }
        ESP_LOGI(TAG, "creating an affine transformation with rows: %d, cols:%d", rows, cols);
        if (xSemaphoreTake(transformations_lock, portMAX_DELAY)) {
            for (int i = 0; i < NUM_TRANSFORMATIONS; i++) {
                randomAffineTransformation(temp_target_data, transformations[i].data, rows, cols);
            }
            xSemaphoreGive(transformations_lock);
        }
        free(temp_target_data);
    }
}

void NOT_randomAffineTransformation(uint8_t* input, uint8_t* output, int rows, int cols, float theta_deg, float lambda, float x_shift, float y_shift) {
    float theta_rad = (theta_deg * M_PI)/180;
    float costheta = cos(theta_rad);
    float sintheta = sin(theta_rad);

    Mat<2, 2> inverse_transform = {0, 0, 0, 0};

    Mat<2, 2> transform = {costheta * lambda, -1 * sintheta * lambda, sintheta * lambda, costheta * lambda};
    transform.invert2x2(inverse_transform);  

    Mat<2, 1> origin_vec = {-cols/2.0f, -rows/2.0f};
    Mat<2, 1> t_origin_vec = {};

    inverse_transform.vecMult2D(origin_vec, t_origin_vec);
    t_origin_vec.m_data[0] += cols/2.0f;
    t_origin_vec.m_data[1] += rows/2.0f;
    int x_in = (int)round(t_origin_vec.m_data[0]);
    int y_in = (int)round(t_origin_vec.m_data[1]);
    int index = y_in * cols + x_in;

    if (x_in >= 0 && x_in < cols && y_in >= 0 && y_in < rows) {
        output[0] = input[index]; 
    }
    else {
        output[0] = 0;
    }


    for (int i = 1; i < cols * rows; i++) {
        int x_out = (i % cols);
        int y_out = i / cols;

        x_in = (int)round(t_origin_vec.m_data[0] + (inverse_transform.m_data[0] * x_out) + (inverse_transform.m_data[1] * y_out));
        y_in = (int)round(t_origin_vec.m_data[1] + (inverse_transform.m_data[2] * x_out) + (inverse_transform.m_data[3] * y_out));
        index = y_in * cols + x_in;
        if (x_in >= 0 && x_in < cols && y_in >= 0 && y_in < rows) {
            output[i] = input[index];
        }
        else {
            output[i] = 0;
        }

    }
}


void randomAffineTransformation(uint8_t* input, uint8_t* output, int rows, int cols) {
    /* Applies a random transformation that simulates small changes in 
     * rotation, scaling, translation, blur, noise
     * rotation, scaling and translation achieved through affine transformation
     * blur noise achieved through gaussian filter
     * randomly selected from normal distributions across these aspects such 
     * that 'normal' situations are weighted heavier in initial Window
     * calculations
     * implemented with backwards mapping, still need to add gaussian filter
     */
    Mat<2, 2> inverse_transform = {0, 0, 0, 0};
    float x_shift = generateRandom<float>(0.0, x_shift_sigma * cols);
    float y_shift = generateRandom<float>(0.0, y_shift_sigma * rows);
    float theta, x_lambda, y_lambda, costheta, sintheta;
    theta = x_lambda = y_lambda = 0; //gets around Werror maybe-unitialized
    while (inverse_transform.isZero()) {
        theta = generateRandom<float>(theta_mean, theta_sigma);
        x_lambda = generateRandom<float>(lambda_mean, lambda_sigma);
        y_lambda = generateRandom<float>(lambda_mean, lambda_sigma);
        costheta = cos((theta * M_PI)/180.0);
        sintheta = sin((theta * M_PI)/180.0);
        Mat<2, 2> transform = {costheta * x_lambda, -1 * sintheta, sintheta, costheta * y_lambda};
        transform.invert2x2(inverse_transform);  
    }

    Mat<2, 1> origin_vec = {-cols/2.0f, -rows/2.0f};
    Mat<2, 1> t_origin_vec = {x_shift, y_shift};

    inverse_transform.vecMult2D(origin_vec, t_origin_vec);
    t_origin_vec.m_data[0] += cols/2.0f;
    t_origin_vec.m_data[1] += rows/2.0f;
    int x_in = (int)round(t_origin_vec.m_data[0] + x_shift);
    int y_in = (int)round(t_origin_vec.m_data[1] + y_shift);
    int index = y_in * cols + x_in;

    if (x_in >= 0 && x_in < cols && y_in >= 0 && y_in < rows) {
        output[0] = input[index]; 
    }
    else {
        output[0] = 0;
    }


    for (int i = 1; i < cols * rows; i++) {
        int x_out = (i % cols);
        int y_out = i / cols;

        x_in = (int)round(t_origin_vec.m_data[0] + (inverse_transform.m_data[0] * x_out) + (inverse_transform.m_data[1] * y_out) + x_shift);
        y_in = (int)round(t_origin_vec.m_data[1] + (inverse_transform.m_data[2] * x_out) + (inverse_transform.m_data[3] * y_out) + y_shift);
        index = y_in * cols + x_in;
        if (x_in >= 0 && x_in < cols && y_in >= 0 && y_in < rows) {
            output[i] = input[index];
        }
        else {
            output[i] = 0;
        }

    }
}

