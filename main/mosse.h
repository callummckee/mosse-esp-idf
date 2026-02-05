#pragma once

#include "esp_dsp.h"
#include "randomutils.h"
#include "freertos/idf_additions.h"

#define NUM_TRANSFORMATIONS 8

struct Image{
    uint8_t* data;
    int rows;
    int cols;
};

class Tracker {
    public:
        Tracker();
        ~Tracker();
        void updateTarget(uint8_t* payload, size_t len);
        SemaphoreHandle_t target_lock;
        Image target;
        SemaphoreHandle_t transformations_lock;
        Image transformations[NUM_TRANSFORMATIONS] = {};
        Image gaussians[NUM_TRANSFORMATIONS + 1] = {};
        void update_transformations(const Image* affines); 
        void generateFG();
        TaskHandle_t transformationTaskHandle = NULL;
        static void transformationTask_tramp(void* _this);
        void transformationTaskLoop();
    private:
        uint8_t* transformation_blob= NULL;
        uint8_t* gaussian_blob = NULL;
        uint8_t* target_data;
};

void NOT_randomAffineTransformation(uint8_t* input, uint8_t* output, int rows, int cols, float theta, float lambda, float x_shift, float y_shift);

void randomAffineTransformation(uint8_t* input, uint8_t* output, int rows, int cols);

