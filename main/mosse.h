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
        void generateInitialFilter();
        TaskHandle_t transformationTaskHandle = NULL;
    private:
        uint8_t* transformation_blob= NULL;
        uint8_t* gaussian_blob = NULL;
        uint8_t* target_data;
        void generate2dGaussian(uint8_t* output, int width, int height, int x_0, int y_0, int xsigma, int ysigma, float amplitude);
        void shiftGaussian(uint8_t* input, uint8_t* output, int width, int height, int x_shift, int y_shift);
        void generateRandomAffine(uint8_t* input, uint8_t* output, int rows, int cols, int& x_shift_out, int& y_shift_out);
};

void NOT_randomAffineTransformation(uint8_t* input, uint8_t* output, int rows, int cols, float theta, float lambda, float x_shift, float y_shift);


