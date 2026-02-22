#pragma once

#include "esp_dsp.h"
#include "randomutils.h"
#include "freertos/idf_additions.h"
#include "benchmark.h"

#define NUM_TRANSFORMATIONS 4

struct Target{
    float* data;
    int true_width;
    int true_height;
    int padded_width;
    int padded_height;
    int x_pos; 
    int y_pos;
};

struct Coord {
    int x;
    int y;
};

class Tracker {
    public:
        Tracker();
        ~Tracker();

        bool isTracking = false; 

        BenchmarkData bd; 

        void test();

        SemaphoreHandle_t target_lock;

        void updateTarget(uint8_t* payload, size_t len, int rows, int cols, int x_pos, int y_pos);
        void updateFilter(uint8_t* frame);

        Coord getTargetPOS();

        void cropFrameToTarget(uint8_t* frame, uint8_t* crop);
        Coord offset = {};
    private:
        const int TILE = 8;
        Target target; //data allocated internally

        void* aligned_blob = NULL;
        void* spiram_blob = NULL;

        //aligned blob
        float* FFT_buf = NULL;
        float* FFT_trans_buf = NULL;

        //allocated internally
        uint8_t* preprocessbuf = NULL;

        //spiram blob
        float* filter = NULL;
        float* A = NULL; 
        float* B = NULL;

        float* f_i = NULL;
        float* g_i = NULL;
        float* F = NULL;
        float* G = NULL;

        float* frame_FT_buf = NULL;
        float* gaussian_buf = NULL;
        float* gaussian_FT_buf = NULL;
        float* A_inst = NULL;
        float* B_inst = NULL;

        float* hann_window_2D = NULL;


        float log_lut[256];
        
        void preprocessFrame(uint8_t* frame);
        void initTracker();

        void generateFG();
        void generateInitialFilter();


        void generate2dHannWindow(float* out, int rows, int cols);
        void FFT2D(float* input, float* output, int M, int N, bool complexInput);
        void IFFT2D(float* input, float* output, int M, int N);
        void generate2dGaussian(float* output, int width, int height, int x_0, int y_0, int xsigma, int ysigma, float amplitude);
        void shiftGaussian(float* input, float* output, int width, int height, int x_shift, int y_shift);
        void generateRandomAffine(float* input, float* output, int rows, int cols, int& x_shift_out, int& y_shift_out);
        void elmWiseComplexDiv(float* in1, float* in2, int size, float* output);
        void elmWiseComplexMult(float* in1, float* in2, int size, float* output, bool conjin1, bool conjin2);
        void complexMatAdd(float* in1, float* in2, float* out, int size);
        void matScalarRealAdd(float* inout, float scalar, int size);
        void matScalarMult(float* inout, float scalar, int size);
        Coord maxG(float* G, int rows, int cols);
        float calcPSR(float* g, int rows, int cols, Coord peak_coord, int excl_thresh);
};



