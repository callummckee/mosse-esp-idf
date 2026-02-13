#include "tracker.h"
#include "config.h"
#include "server.h"
#include "mat.h"

#include "dspm_mult.h"
#include "dsps_fft2r.h"
#include "dsps_wind_hann.h"
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
#include "esp_dsp.h"


const int theta_mean = 0;
const int theta_sigma = 30;
const int lambda_mean = 1;
const float lambda_sigma = 0.5;
const float x_shift_sigma = 0.15;
const float y_shift_sigma = 0.15;
const float reg = 0.01;
const float learning_rate = 0.05;


static const char* TAG = "mosse.cpp";

Tracker::Tracker() {
    size_t img_size = FRAME_WIDTH * FRAME_HEIGHT;

    target.data = (uint8_t*)heap_caps_malloc(img_size, MALLOC_CAP_SPIRAM);
    if (!target.data) {
        ESP_LOGE(TAG, "couldn't allocate target_data mem");
    }
    target_lock = xSemaphoreCreateMutex();
    if (target_lock == NULL) {
        ESP_LOGE(TAG, "failed to create target_lock semaphore");
    }


    filter= (float*)heap_caps_malloc(img_size * 2* sizeof(float), MALLOC_CAP_SPIRAM);
    if(!filter) {
        ESP_LOGE(TAG, "couldn't allocate filter");
    }

    A = (float*)heap_caps_malloc(img_size * 2* sizeof(float), MALLOC_CAP_SPIRAM);
    if(!A) {
        ESP_LOGE(TAG, "couldn't allocate A");
    }

    B = (float*)heap_caps_malloc(img_size * 2* sizeof(float), MALLOC_CAP_SPIRAM);
    if(!B) {
        ESP_LOGE(TAG, "couldn't allocate B");
    }

    A_inst = (float*)heap_caps_malloc(img_size * 2 * sizeof(float), MALLOC_CAP_SPIRAM);
    if(!A_inst) {
        ESP_LOGE(TAG, "couldn't allocate A_inst");
    }

    B_inst = (float*)heap_caps_malloc(img_size * 2 * sizeof(float), MALLOC_CAP_SPIRAM);
    if(!B_inst) {
        ESP_LOGE(TAG, "couldn't allocate B_inst");
    }


    f_i = (float*)heap_caps_malloc(img_size * (NUM_TRANSFORMATIONS + 1) * sizeof(float), MALLOC_CAP_SPIRAM);
    if (!f_i) {
        ESP_LOGE("server", "couldn't allocate f_i_blob");
        return;
    }

    g_i = (float*)heap_caps_malloc(img_size * (NUM_TRANSFORMATIONS + 1), MALLOC_CAP_SPIRAM);
    if (!g_i) {
        ESP_LOGE("server", "couldn't allocate g_i_blob");
        return;
    }


    frame_conv_buf = (float*)heap_caps_malloc(img_size * sizeof(float), MALLOC_CAP_SPIRAM);    
    if(!frame_conv_buf) {
        ESP_LOGE(TAG, "couldn't allocate frame_conv_buf");
    }

    frame_FT_buf = (float*)heap_caps_malloc(img_size * 2 * sizeof(float), MALLOC_CAP_SPIRAM);
    if(!frame_FT_buf) {
        ESP_LOGE(TAG, "couldn't allocate frame_FT_buf");
    }

    gaussian_buf = (float*)heap_caps_malloc(img_size * sizeof(float), MALLOC_CAP_SPIRAM);
    if(!gaussian_buf) {
        ESP_LOGE(TAG, "couldn't allocate gaussian_buf");
    }
    
    gaussian_FT_buf = (float*)heap_caps_malloc(img_size * 2 * sizeof(float), MALLOC_CAP_SPIRAM);
    if(!gaussian_FT_buf) {
        ESP_LOGE(TAG, "couldn't allocate gaussian_FT_buf");
    }

    FFT_buf = (float*)heap_caps_aligned_alloc(16, img_size*2*sizeof(float), MALLOC_CAP_SPIRAM);
    if(!FFT_buf) {
        ESP_LOGE(TAG, "couldn't allocate FFT_buf");
    }

    FFT_trans_buf = (float*)heap_caps_aligned_alloc(16, img_size*2*sizeof(float), MALLOC_CAP_SPIRAM);
    if(!FFT_trans_buf) {
        ESP_LOGE(TAG, "couldn't allocate FFT_trans_buf");
    }

    hann_window_2D = (float*)heap_caps_malloc(img_size * sizeof(float), MALLOC_CAP_SPIRAM);
    if(!hann_window_2D) {
        ESP_LOGE(TAG, "couldn't allocate hann_window_2D");
    }
    
    cropped_frame = (uint8_t*)heap_caps_malloc(img_size * sizeof(uint8_t), MALLOC_CAP_INTERNAL);
    if(!cropped_frame) {
        ESP_LOGE(TAG, "couldn't allocate cropped_frame");
    }

    size_t internal_free = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    size_t psram_free = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
    size_t max_block = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);

    ESP_LOGI(TAG, "Heap Stats - Internal: %d, PSRAM: %d, Max Contiguous: %d\n", internal_free, psram_free, max_block);

}

Tracker::~Tracker() {
    free(target.data);

    free(filter);
    free(A);
    free(B);

    free(f_i);
    free(g_i);

    free(frame_conv_buf);
    free(frame_FT_buf);
    free(gaussian_buf);
    free(gaussian_FT_buf);
    free(A_inst);
    free(B_inst);

    free(FFT_buf);
    free(FFT_trans_buf);
    
    free(hann_window_2D);
}

void Tracker::initTracker() {
    generateFG();
    generateInitialFilter();
}

void Tracker::generateFG() {
    //generates random affine transformations (set F) and associated 2d gaussian (set G)
    int x_shift_temp, y_shift_temp;
    int rows = target.rows;
    int cols = target.cols;
    int target_size = rows * cols;
    int stride = target_size * 2;
    //generate 2d gaussian for untransformed target
    int x_0 = (int)round(cols/2.0f) - 1;
    int y_0 = (int)round(rows/2.0f) - 1;
    generate2dGaussian(g_i, cols, rows, x_0, y_0, 2, 2, 255);
    generate2dHannWindow(hann_window_2D, rows, cols);
    //assign target to first index of f_i
    for (int i = 0; i < target_size; i++) {
        f_i[i] = (float)target.data[i] * hann_window_2D[i];
    } 

    //generate affine transformation and their associated gaussians
    for (int i = 0; i < NUM_TRANSFORMATIONS; i++) {
       generateRandomAffine(target.data, f_i + stride * (i + 1), target.rows, target.cols, x_shift_temp, y_shift_temp);
       shiftGaussian(g_i, g_i + stride * (i + 1), target.rows, target.cols, x_shift_temp, y_shift_temp); 
    }
}

void Tracker::generateInitialFilter() {
   // need to add padding such that target dimensions are always powers of 2 I believe
   int rows = target.rows;
   int cols = target.cols;
   size_t target_size = cols * rows;
   int stride = target_size * 2;
   float* F = (float*)heap_caps_malloc(target_size * 2 * sizeof(float) * (NUM_TRANSFORMATIONS + 1), MALLOC_CAP_SPIRAM);
   float* G = (float*)heap_caps_malloc(target_size * 2 * sizeof(float) * (NUM_TRANSFORMATIONS + 1), MALLOC_CAP_SPIRAM);

   //need to cast target.data and gaussians
   for(int i = 0; i < NUM_TRANSFORMATIONS + 1; i++) {
       FFT2D(f_i + i * stride, &F[target_size * 2 * i], rows, cols, false);
       FFT2D(g_i + i * stride, &G[target_size * 2 * i], rows, cols, false);
   }
    
   float* numer_buf = (float*)heap_caps_malloc(target_size * 2 * sizeof(float), MALLOC_CAP_SPIRAM);
   float* denom_buf = (float*)heap_caps_malloc(target_size * 2 * sizeof(float), MALLOC_CAP_SPIRAM);

   elmWiseComplexMult(&G[0], &F[0], target_size, A, false, true);
   elmWiseComplexMult(&F[0], &F[0], target_size, B, false, true);

   for (int i = 0; i < NUM_TRANSFORMATIONS; i++) {
       elmWiseComplexMult(&G[target_size * 2 * (i + 1)], &F[target_size * 2 * (i + 1)], target_size, numer_buf, false, true);
       elmWiseComplexMult(&F[target_size * 2 * (i + 1)], &F[target_size * 2 * (i + 1)], target_size, denom_buf, false, true);
       complexMatAdd(A, numer_buf, A, target_size);
       complexMatAdd(B, denom_buf, B, target_size);
   } 
   matScalarRealAdd(B, reg, target_size);
   elmWiseComplexDiv(A, B, target_size, filter);

   free(numer_buf);
   free(denom_buf);
   free(F);
   free(G);
}

void Tracker::updateFilter(uint8_t* frame) {
    int rows = target.rows;
    int cols = target.cols;
    int target_size = rows * cols;

    cropFrameToTarget(frame, cropped_frame);

    //convert frame to float and apply hann_window
    for (int i = 0; i < target_size; i++) {
        frame_conv_buf[i] = (float)cropped_frame[i] * hann_window_2D[i];
    }

    //FFT frame
    FFT2D(frame_conv_buf, frame_FT_buf, rows, cols, false);

    //generate tracking gaussian
    elmWiseComplexMult(frame_FT_buf, filter, target_size, gaussian_FT_buf, false, false);
    IFFT2D(gaussian_FT_buf, gaussian_buf, rows, cols, false);
    Coord peak = maxG(gaussian_buf, rows, cols);
    Coord offset = {peak.x - cols/2, peak.y - rows/2};
    target.x_pos += offset.x;
    target.y_pos += offset.y;
    if (target.x_pos < 0) {
        target.x_pos = 0;
    }
    if (target.y_pos < 0) {
        target.y_pos = 0;
    }
    if (target.x_pos > (FRAME_WIDTH)) {
        target.x_pos = FRAME_WIDTH;
    }
    if (target.y_pos > (FRAME_HEIGHT)) {
        target.y_pos = FRAME_HEIGHT;
    }
    ESP_LOGI(TAG, "offset.x: %d, offset.y: %d, x_pos: %d, y_pos: %d", offset.x, offset.y, target.x_pos, target.y_pos);
    cropFrameToTarget(frame, cropped_frame);
    for (int i = 0; i < target_size; i++) {
        frame_conv_buf[i] = (float)cropped_frame[i] * hann_window_2D[i];
    }
    FFT2D(frame_conv_buf, frame_FT_buf, rows, cols, false);
    generate2dGaussian(gaussian_buf, cols, rows, cols/2, rows/2, 2, 2, 255);
    FFT2D(gaussian_buf, gaussian_FT_buf, rows, cols, false);
    elmWiseComplexMult(gaussian_FT_buf, frame_FT_buf, target_size, A_inst, false, true);
    elmWiseComplexMult(frame_FT_buf, frame_FT_buf, target_size, B_inst, false, true);
    matScalarMult(A_inst, learning_rate, target_size);
    matScalarMult(B_inst, learning_rate, target_size);
    matScalarMult(A, (1 - learning_rate), target_size);
    matScalarMult(B, (1 - learning_rate), target_size);
    complexMatAdd(A_inst, A, A, target_size);
    complexMatAdd(B_inst, B, B, target_size);
    matScalarRealAdd(B, reg, target_size);
    elmWiseComplexDiv(A, B, target_size, filter);
}
        
void Tracker::cropFrameToTarget(uint8_t* frame, uint8_t* crop) {
    int source_width = FRAME_WIDTH;
    int width = target.cols;
    int height = target.rows;
    int x_init = target.x_pos;
    int y_init = target.y_pos;

    for (int i = 0; i < width * height; i++) {
        int x_dest = i % width;
        int y_dest = i / width;
        int x_source = x_dest + x_init;
        int y_source = y_dest + y_init;
        int source_index = y_source * source_width + x_source;
        crop[i] = frame[source_index]; 
    }
}

Coord Tracker::maxG(float* G, int rows, int cols) {
    //takes complex input, rows, cols wrt complex elements
    float max = 0;
    Coord max_coord = {0, 0};
    for (int i = 0; i < rows * cols; i++) {
        if (G[i * 2] > max) {
            max = G[i * 2];
            max_coord.x = i % cols;
            max_coord.y = i / cols;
        }
    }
    return max_coord;
}

void Tracker::matScalarRealAdd(float* inout, float scalar, int size) {
    //takes complex input, adds scalar only to real part, size is in complex elements
    for(int i = 0; i < size; i++) {
        inout[i*2] = inout[i*2] + scalar;
    }
}

void Tracker::matScalarMult(float* inout, float scalar, int size) {
    //takes complex input, multiplies both real and complex parts by scalar, size is in complex elements
    for (int i = 0; i < size; i++) {
        inout[i * 2] = inout[i * 2] * scalar;
        inout[i * 2 + 1] = inout[i * 2 + 1] * scalar;
    }
}

void Tracker::complexDiv(float re1, float im1, float re2, float im2, float* out) {
    out[0] = (re1 * re2 + im1 * im2)/(re2 * re2 + im2 * im2);
    out[1] = (im1 * re2 - re1 * im2)/(re2 * re2 + im2 * im2);
}

void Tracker::complexMult(float re1, float im1, float re2, float im2, float* out) {
    out[0] = re1 * re2 - im1 * im2;
    out[1] = re1 * im2 + im1 * re2;
}

void Tracker::complexMatAdd(float* in1, float* in2, float* out, int size) {
    //size in complex elements
    for (int i = 0; i < size; i++) {
        out[i*2] = in1[i*2] + in2[i*2];
        out[i*2 + 1] = in1[i*2 + 1] + in2[i*2 + 1];
    }
}

void Tracker::elmWiseComplexDiv(float* in1, float* in2, int size, float* output) {
    //size in complex elements
    for (int i = 0; i < size; i++) {
        complexDiv(in1[i*2], in1[i*2 + 1], in2[i*2], in2[i*2 + 1], &output[i*2]);
    }

}

void Tracker::elmWiseComplexMult(float* in1, float* in2, int size, float* output, bool conjin1, bool conjin2) {
    // size in complex elements, conjin1 conjin2 set factors to conjugates
    int conj1 = 1;
    int conj2 = 1;
    if (conjin1) {
        conj1 = -1;
    }
    if (conjin2) {
        conj2 = -1;
    }

    for(int i = 0; i < size; i++) {
        complexMult(in1[i*2], conj1*in1[i*2 + 1], in2[i*2], conj2*in2[i*2 + 1], &output[i * 2]);
    }

}

void Tracker::updateTarget(uint8_t* payload, size_t len, int rows, int cols, int x_pos, int y_pos) {
    if(xSemaphoreTake(this->target_lock, portMAX_DELAY)) {
        memcpy(target.data, payload, len);
        target.rows = rows;
        target.cols = cols;
        target.x_pos = x_pos;
        target.y_pos = y_pos;
        xSemaphoreGive(target_lock);
        initTracker();
        ESP_LOGI(TAG, "starting tracking..");
        isTracking = true;
    } 
}


void NOT_generateRandomAffine(uint8_t* input, uint8_t* output, int rows, int cols, float theta_deg, float lambda, float x_shift, float y_shift) {
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


void Tracker::generateRandomAffine(uint8_t* input, float* output, int rows, int cols, int& x_shift_out, int& y_shift_out) {
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
        output[0] = (float)input[index]; 
    }
    else {
        output[0] = 0.0;
    }


    for (int i = 1; i < cols * rows; i++) {
        int x_out = (i % cols);
        int y_out = i / cols;

        x_in = (int)round(t_origin_vec.m_data[0] + (inverse_transform.m_data[0] * x_out) + (inverse_transform.m_data[1] * y_out) + x_shift);
        y_in = (int)round(t_origin_vec.m_data[1] + (inverse_transform.m_data[2] * x_out) + (inverse_transform.m_data[3] * y_out) + y_shift);
        index = y_in * cols + x_in;
        if (x_in >= 0 && x_in < cols && y_in >= 0 && y_in < rows) {
            output[i] = (float)input[index];
        }
        else {
            output[i] = 0.0;
        }

    }
}

void Tracker::generate2dHannWindow(float* out, int rows, int cols) {
    float* window_x = (float*)heap_caps_malloc(cols * sizeof(float), MALLOC_CAP_SPIRAM);
    if(!window_x) {
        ESP_LOGE(TAG, "couldn't allocate window_x");
    }
    float* window_y = (float*)heap_caps_malloc(rows * sizeof(float), MALLOC_CAP_SPIRAM);
    if(!window_y) {
        ESP_LOGE(TAG, "couldn't allocate window_y");
    }
    dsps_wind_hann_f32(window_x, cols);
    dsps_wind_hann_f32(window_y, rows);

    for (int i = 0; i < rows * cols; i++) {
        int x = i % cols;
        int y = i / cols;
        out[i] = window_x[x] * window_y[y];
    }

    free(window_x);
    free(window_y);
}

void Tracker::generate2dGaussian(float* output, int width, int height, int x_0, int y_0, int xsigma, int ysigma, float amplitude) {
    // x_0, y_0 centre of gaussian (must be 0 indexed!), output pre-allocated 
    for (int i = 0; i < width * height; i++) {
        int y = i/width;
        int x = i % width;
        output[i] = (amplitude*exp(-1 * (((x - x_0) * (x - x_0))/(2.0*xsigma*xsigma) + ((y - y_0) * (y - y_0))/(2.0*ysigma*ysigma))));
    }
}

//calculating shiftGaussian like this does lead to some off by one errors in centring the gaussian on the affine transform due to the backwards mapping rounding shit but it really shouldn't matter as long as the tracking window isn't tiny tiny
void Tracker::shiftGaussian(float* input, float* output, int width, int height, int x_shift, int y_shift) {
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

void Tracker::IFFT2D(float* input, float* output, int M, int N, bool complexInput) {
    //intended for use with complex input, M, N refer to complex elements, i.e. size of input in bytes is M*N*2*sizeof(float)
    int size = M*N; 
    //conjugate input
    for (int i = 0; i < size; i++) {
        input[i * 2 + 1] = -1 * input[i * 2 + 1];
    }
    FFT2D(input, output, M, N, complexInput);
    for (int i = 0; i < size; i ++) {
        output[i * 2] = (output[i * 2])/(M*N);
        output[i * 2 + 1] = (-1 * output[i * 2 + 1])/(M*N);
    }


}

void Tracker::FFT2D(float* input, float* output, int M, int N, bool complexInput) { 
    esp_err_t ret;
    ret = dsps_fft2r_init_fc32(NULL, CONFIG_DSP_MAX_FFT_SIZE);
    if (ret  != ESP_OK) {
        ESP_LOGE(TAG, "Not possible to initialize FFT. Error = %i", ret);
        return;
    }

    //transform input to array of form Re[0], Im[0], ... Re[N-1], Im[N-1]
    if (!complexInput) {
        for (int i = 0; i < M*N; i++) {
            FFT_buf[i * 2] = input[i];
            FFT_buf[i * 2 + 1] = 0;
        }
    }
    else {
        for (int i = 0; i < M*N*2; i++) {
            FFT_buf[i] = input[i];
        }
    }

    //apply fft to each row
    for(int i = 0; i < M; i++) {
        float* row_ptr = FFT_buf + i*N*2;
        ret = dsps_fft2r_fc32_aes3(row_ptr, N);
        if (ret  != ESP_OK) {
            ESP_LOGE(TAG, "error performing fft on row %d Error = %x", i, ret);
        }
        dsps_bit_rev_fc32_ansi(row_ptr, N);
    }

    //transpose intermediate matrix
    for (int i = 0; i < M*N; i++) {
        int x = i % M;
        int y = i / M;
        int index = x * N + y;
        FFT_trans_buf[i*2] = FFT_buf[index * 2];
        FFT_trans_buf[i*2 + 1] = FFT_buf[index * 2 + 1];
    }

    //apply fft to each col of og matrix by applying to each row of transpose
    for(int i = 0; i < N; i++) {
        float* col_ptr = FFT_trans_buf + i * M * 2;
        dsps_fft2r_fc32_aes3(col_ptr, M);
        dsps_bit_rev_fc32_ansi(col_ptr, M);
    }

    for (int i = 0; i < M*N; i++) {
        int x = i % N;
        int y = i / N;
        int index = x * M + y;
        FFT_buf[i*2] = FFT_trans_buf[index * 2];
        FFT_buf[i*2 + 1] = FFT_trans_buf[index * 2 + 1];
    }

    memcpy(output, FFT_buf, M*N*2*sizeof(float));
}

Coord Tracker::getTargetPOS() {
    Coord out = {target.x_pos, target.y_pos};
    return out;
}

