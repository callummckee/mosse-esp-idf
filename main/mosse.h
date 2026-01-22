#pragma once

#include "esp_dsp.h"
#include "randomutils.h"

struct Image{
    uint8_t* data;
    int rows;
    int cols;
};

void NOT_randomAffineTransformation(uint8_t* input, uint8_t* output, int rows, int cols, float theta, float lambda, float x_shift, float y_shift);

void randomAffineTransformation(uint8_t* input, uint8_t* output, int rows, int cols);

