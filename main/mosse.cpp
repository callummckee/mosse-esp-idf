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
const float lambda_sigma = 0.5;
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

    //better to malloc these now with max size as otherwise repeat on demand mallocs in the hot path
    size_t img_bytes = FRAME_WIDTH * FRAME_HEIGHT;
    transformation_blob = (uint8_t*)heap_caps_malloc(img_bytes * NUM_TRANSFORMATIONS, MALLOC_CAP_SPIRAM);
    if (!transformation_blob) {
        ESP_LOGE("server", "couldn't allocate transformation_blob");
        return;
    }
    for (int i = 0; i < NUM_TRANSFORMATIONS; i++) {
        transformations[i].data = transformation_blob + (i * img_bytes);
    }
    gaussian_blob = (uint8_t*)heap_caps_malloc(img_bytes * (NUM_TRANSFORMATIONS + 1), MALLOC_CAP_SPIRAM);
    if (!gaussian_blob) {
        ESP_LOGE("server", "couldn't allocate gaussian_blob");
        return;
    }
    for (int i = 0; i < NUM_TRANSFORMATIONS + 1; i++) {
        gaussians[i].data = gaussian_blob + (i * img_bytes);
    }

    transformations_lock = xSemaphoreCreateMutex();
    if (transformations_lock == NULL) {
        ESP_LOGE(TAG, "failed to create transformations_lock semaphore");
    }
}

Tracker::~Tracker() {
    free(target_data);
    free(transformation_blob);
    free(gaussian_blob);
}

void Tracker::generateFG() {
    int x_shift_temp, y_shift_temp;
    int x_0 = (int)round(target.cols/2.0f) - 1;
    int y_0 = (int)round(target.rows/2.0f) - 1;
    generate2dGaussian(gaussians[0].data, target.cols, target.rows, x_0, y_0, 2, 2, 255);
    for (int i = 0; i < NUM_TRANSFORMATIONS; i++) {
       randomAffineTransformation(target.data, transformations[i].data, target.rows, target.cols, x_shift_temp, y_shift_temp);
       shiftGaussian(gaussians[0].data, gaussians[i + 1].data, target.rows, target.cols, x_shift_temp, y_shift_temp); 
    }
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
    //test function can probably be deleted soon
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


void randomAffineTransformation(uint8_t* input, uint8_t* output, int rows, int cols, int& x_shift_out, int& y_shift_out) {
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

    //generate a random transformation matrix and calculate its inverse
    float x_shift = generateRandom<float>(0.0, x_shift_sigma * cols);
    x_shift_out = x_shift;
    float y_shift = generateRandom<float>(0.0, y_shift_sigma * rows);
    y_shift_out = y_shift;
    float theta, x_lambda, y_lambda, costheta, sintheta;
    theta = x_lambda = y_lambda = 0; //gets around warning maybe-unitialized
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
    Mat<2, 1> t_origin_vec = {0};

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

void generate2dGaussian(uint8_t* output, int width, int height, int x_0, int y_0, int xsigma, int ysigma, float amplitude) {
    // x_0, y_0 centre of gaussian (must be 0 indexed!), output pre-allocated 
    for (int i = 0; i < width * height; i++) {
        int y = i/width;
        int x = i % width;
        output[i] = (amplitude*exp(-1 * (((x - x_0) * (x - x_0))/(2*xsigma*xsigma) + ((y - y_0) * (y - y_0))/(2*ysigma*ysigma))));
    }
}

//calculating shiftGaussian like this does lead to some off by one errors in centring the gaussian on the affine transform due to the backwards mapping rounding shit but it really shouldn't matter as long as the tracking window isn't tiny tiny
void shiftGaussian(uint8_t* input, uint8_t* output, int width, int height, int x_shift, int y_shift) {
    for (int i = 0; i < width * height; i++) {
        int y = i/width;
        int x = i % width;
        int x_in = x + x_shift;
        int y_in = y + y_shift;
        int in_index = y_in * width + x_in;
        if (x_in < 0 || x_in >= width || y_in < 0 || y_in >= height) {
            output[i] = 0;
        }
        else {
            output[i] = input[in_index];
        }
    }
}

