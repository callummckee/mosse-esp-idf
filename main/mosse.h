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

struct Coord {
    int x;
    int y;
};

class Tracker {
    public:
        Tracker();
        ~Tracker();

        void test();

        SemaphoreHandle_t target_lock;
        SemaphoreHandle_t transformations_lock;
        TaskHandle_t transformationTaskHandle = NULL;
    private:
        //buffers to be allocated in constructor, 617kb with 96x96 image and 8 transformations with buffers target.data, filter, A, B, f_i, g_i, frame_conv_buf, frame_FFT_buf, gaussian_buf, gaussian_FFT_buf
        Image target;

        float* filter = NULL;
        float* A = NULL; 
        float* B = NULL;

        float* f_i = NULL;
        float* g_i = NULL;

        //scratchpad buffers
        float* frame_conv_buf = NULL;
        float* frame_FT_buf = NULL;
        float* gaussian_buf = NULL;
        float* gaussian_FT_buf = NULL;
        float* A_inst = NULL;
        float* B_inst = NULL;

        //for FFT2D
        float* FFT_buf = NULL;
        float* FFT_trans_buf = NULL;


        void updateTarget(uint8_t* payload, size_t len);
        void generateFG();
        void generateInitialFilter();
        void updateFilter(uint8_t* frame);


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
        void matScalarRealAdd(float* inout, float scalar, int size);
        void matScalarMult(float* inout, float scalar, int size);
        Coord maxG(float* G, int rows, int cols);
};

void NOT_randomAffineTransformation(uint8_t* input, uint8_t* output, int rows, int cols, float theta, float lambda, float x_shift, float y_shift);


