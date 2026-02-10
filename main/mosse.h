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

struct floatImage{
    float* data;
    int rows;
    int cols;
};

struct Filter{
    float* data;
    int size;
};

class Tracker {
    public:
        Tracker();
        ~Tracker();

        void test();

        SemaphoreHandle_t target_lock;
        SemaphoreHandle_t transformations_lock;
        TaskHandle_t transformationTaskHandle = NULL;

        Image target;
        floatImage f_i[NUM_TRANSFORMATIONS + 1] = {}; //these can just be blobs! refactor later
        floatImage g_i[NUM_TRANSFORMATIONS + 1] = {};
        Filter filter;

        void updateTarget(uint8_t* payload, size_t len);
        void update_transformations(const Image* affines); 
        void generateFG();
        void generateInitialFilter();
        void updateFilter();
    private:
        Filter A; //I know these should be different types
        Filter B;

        float* f_i_blob = NULL;
        float* g_i_blob = NULL;

        void FFT2D(float* input, float* output, int M, int N, bool complexInput);
        void IFFT2D(float* input, float* output, int M, int N, bool complexInput);
        void generate2dGaussian(float* output, int width, int height, int x_0, int y_0, int xsigma, int ysigma, float amplitude);
        void shiftGaussian(float* input, float* output, int width, int height, int x_shift, int y_shift);
        void generateRandomAffine(uint8_t* input, float* output, int rows, int cols, int& x_shift_out, int& y_shift_out);
        void complexMult(float re1, float im1, float re2, float im2, float* out);
        void complexDiv(float re1, float im1, float re2, float im2, float* out);
        void elmWiseComplexDiv(float* in1, float* in2, int size, float* output);
        void elmWiseComplexMult(float* in1, float* in2, int size, float* output, bool conjin1, bool conjin2);
        void complexMatAdd(float* in1, float* in2, float* out, int size);
        void matScalarAdd(float* in, float scalar, int size);
};

void NOT_randomAffineTransformation(uint8_t* input, uint8_t* output, int rows, int cols, float theta, float lambda, float x_shift, float y_shift);


