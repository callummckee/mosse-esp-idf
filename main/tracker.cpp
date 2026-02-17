#include "tracker.h"
#include "benchmark.h"
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
#include <cstdlib>
#include <float.h>
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
const float learning_rate = 0.125;


static const char* TAG = "mosse.cpp";

Tracker::Tracker() {
    target_lock = xSemaphoreCreateMutex();
    assert(target_lock != NULL && "FATAL: failed to create target_lock semaphore");

    esp_err_t ret = dsps_fft2r_init_fc32(NULL, CONFIG_DSP_MAX_FFT_SIZE);
    assert(ret == ESP_OK && "FATAL: failed to init fft table");

    size_t img_f = FRAME_WIDTH * FRAME_HEIGHT;
    size_t img_c = img_f * 2;

    size_t aligned_size = img_c * 2 * sizeof(float);
    aligned_blob = heap_caps_aligned_alloc(16, aligned_size, MALLOC_CAP_SPIRAM);
    assert(aligned_blob != NULL && "FATAL: failed to allocate aligned blob");
    FFT_buf = (float*)aligned_blob;
    FFT_trans_buf = FFT_buf + img_c;

    preprocessbuf = (uint8_t*)heap_caps_malloc(img_f * sizeof(uint8_t), MALLOC_CAP_INTERNAL);
    assert(preprocessbuf != NULL && "FATAL: failed to allocate preprocess buf");

    target.data = (float*)heap_caps_malloc(img_f * sizeof(float), MALLOC_CAP_INTERNAL);
    assert(target.data != NULL && "FATAL: failed to allocate target.data");

    size_t spiram_size = sizeof(float) * (img_c * 8 + img_f * 4 * (NUM_TRANSFORMATIONS + 1) + img_f * 1); 
    spiram_blob = heap_caps_malloc(spiram_size, MALLOC_CAP_SPIRAM);
    assert(spiram_blob != NULL && "FATAL: failed to allocate spiram_blob");
    float* ptr = (float*)spiram_blob;
    filter = ptr; ptr += img_c;
    A = ptr; ptr += img_c;
    B = ptr; ptr += img_c;
    A_inst = ptr; ptr += img_c;
    B_inst = ptr; ptr += img_c;
    frame_FT_buf = ptr; ptr += img_c;
    gaussian_FT_buf = ptr; ptr += img_c;
    gaussian_buf = ptr; ptr += img_c;
    hann_window_2D = ptr; ptr += img_f;
    f_i = ptr; ptr += img_f * (NUM_TRANSFORMATIONS + 1);
    g_i = ptr; ptr += img_f * (NUM_TRANSFORMATIONS + 1);
    F = ptr; ptr += img_f * (NUM_TRANSFORMATIONS + 1);
    G = ptr; ptr += img_f * (NUM_TRANSFORMATIONS + 1);

    for (int i = 0; i < 256; i++) {
        log_lut[i] = logf((float)i + 1.0f);
    }

    size_t internal_free = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    size_t psram_free = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
    size_t max_block = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
    ESP_LOGI(TAG, "Heap Stats - Internal: %d, PSRAM: %d, Max Contiguous: %d\n", internal_free, psram_free, max_block);
}

Tracker::~Tracker() {
    heap_caps_free(preprocessbuf);
    heap_caps_free(target.data);
    heap_caps_free(aligned_blob);
    heap_caps_free(spiram_blob);
}

void Tracker::initTracker() {
    generateFG();
    generateInitialFilter();
}

void Tracker::generateFG() {
    //generates random affine transformations (set F) and associated 2d gaussian (set G)
    int x_shift_temp, y_shift_temp;
    int rows = target.padded_height;
    int cols = target.padded_width;
    int target_size = rows * cols;
    int stride = target_size * 2;
    //generate 2d gaussian for untransformed target
    int x_0 = (int)round(cols/2.0f);
    int y_0 = (int)round(rows/2.0f);
    generate2dGaussian(g_i, cols, rows, x_0, y_0, 2, 2, 1.0);
    generate2dHannWindow(hann_window_2D, rows, cols);
    //assign target to first index of f_i
    for (int i = 0; i < target_size; i++) {
        f_i[i] = (float)target.data[i];
    } 

    //generate affine transformation and their associated gaussians
    for (int i = 0; i < NUM_TRANSFORMATIONS; i++) {
        generateRandomAffine(target.data, f_i + stride * (i + 1), rows, cols, x_shift_temp, y_shift_temp);
        shiftGaussian(g_i, g_i + stride * (i + 1), rows, cols, x_shift_temp, y_shift_temp); 
    }
}

void Tracker::generateInitialFilter() {
    // need to add padding such that target dimensions are always powers of 2 I believe
    int rows = target.padded_height;
    int cols = target.padded_width;
    size_t target_size = cols * rows;
    int stride = target_size * 2;

    //need to cast target.data and gaussians
    for(int i = 0; i < NUM_TRANSFORMATIONS + 1; i++) {
        FFT2D(f_i + i * stride, &F[target_size * 2 * i], rows, cols, false);
        FFT2D(g_i + i * stride, &G[target_size * 2 * i], rows, cols, false);
    }

    float* numer_buf = (float*)heap_caps_malloc(target_size * 2 * sizeof(float), MALLOC_CAP_SPIRAM);
    float* denom_buf = (float*)heap_caps_malloc(target_size * 2 * sizeof(float), MALLOC_CAP_SPIRAM);

    if (!numer_buf || !denom_buf) {
        ESP_LOGE(TAG, "error allocated numer or denom buf in generateInitialFilter");
    }
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
    matScalarRealAdd(B, -1 * reg, target_size);

    heap_caps_free(numer_buf);
    heap_caps_free(denom_buf);
}

void Tracker::updateFilter(uint8_t* frame) {
    int rows = target.padded_height;
    int cols = target.padded_width;
    int target_size = rows * cols;

    {
        Benchmarker b(&bd.cropprocess_cycles);
        cropFrameToTarget(frame, preprocessbuf);
        preprocessFrame(preprocessbuf);
    }
    {
        Benchmarker b(&bd.fft_cycles);
        FFT2D(target.data, frame_FT_buf, rows, cols, false);
    }
    //generate tracking gaussian
    {
        Benchmarker b(&bd.mult_cycles);
        elmWiseComplexMult(frame_FT_buf, filter, target_size, gaussian_FT_buf, false, false);
    }
    {
        Benchmarker b(&bd.ifft_cycles);
        IFFT2D(gaussian_FT_buf, gaussian_buf, rows, cols);
    }
    Coord peak = maxG(gaussian_buf, rows, cols);
    float psr = calcPSR(gaussian_buf, rows, cols, peak, 5);
    ESP_LOGI(TAG, "psr: %f", psr);
    Coord offset = {peak.x - cols/2, peak.y - rows/2};
    target.x_pos += offset.x;
    target.y_pos += offset.y;
    if (target.x_pos < 0) {
        target.x_pos = 0;
    }
    if (target.y_pos < 0) {
        target.y_pos = 0;
    }
    if (target.x_pos > (FRAME_WIDTH - target.true_width)) {
        target.x_pos = FRAME_WIDTH - target.true_width;
    }
    if (target.y_pos > (FRAME_HEIGHT - target.true_height)) {
        target.y_pos = FRAME_HEIGHT - target.true_height;
    }
    cropFrameToTarget(frame, preprocessbuf);
    preprocessFrame(preprocessbuf);
    FFT2D(target.data, frame_FT_buf, rows, cols, false);
    elmWiseComplexMult(G, frame_FT_buf, target_size, A_inst, false, true); //using ideal centered gaussian
    elmWiseComplexMult(frame_FT_buf, frame_FT_buf, target_size, B_inst, false, true);
    matScalarMult(A_inst, learning_rate, target_size);
    matScalarMult(B_inst, learning_rate, target_size);
    matScalarMult(A, (1 - learning_rate), target_size);
    matScalarMult(B, (1 - learning_rate), target_size);
    complexMatAdd(A_inst, A, A, target_size);
    complexMatAdd(B_inst, B, B, target_size);
    matScalarRealAdd(B, reg, target_size);
    elmWiseComplexDiv(A, B, target_size, filter);
    matScalarRealAdd(B, -1 * reg, target_size);

    bd.frame_count++;
    if (bd.frame_count == 100) {
        bd.avg_cropprocess = bd.cropprocess_cycles/100;
        bd.avg_fft = bd.fft_cycles/100;
        bd.avg_ifft = bd.ifft_cycles/100;
        bd.avg_mult = bd.mult_cycles/100;

        bd.cropprocess_cycles = 0;
        bd.fft_cycles = 0;
        bd.ifft_cycles = 0;
        bd.mult_cycles = 0;
        bd.frame_count = 0;

        esp_rom_printf("cropprocess: %lu\nfft: %lu\nifft: %lu\nmult: %lu\n", bd.avg_cropprocess, bd.avg_fft, bd.avg_ifft, bd.avg_mult);
    }
}

void Tracker::cropFrameToTarget(uint8_t* frame, uint8_t* crop) {
    int source_width = FRAME_WIDTH;
    int width = target.true_width;
    int height = target.true_height;
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
    float max = -FLT_MAX;
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

float Tracker::calcPSR(float* g, int rows, int cols, Coord peak_coord, int excl_thresh) {
    //calculates peak to sidelobe ratio of g (interleaved complex), excl thresh denotes the side length of the area around the peak excluded from the calculation
    int count = 0;
    float mean = 0;
    float M2 = 0;
    float max = -FLT_MAX;
    for (int i = 0; i < rows * cols; i++) {
        int x = i % cols;
        int y = i / cols;
        if (g[i*2] > max) {
            max = g[i*2];
        }
        if (abs(x - peak_coord.x) <= excl_thresh && abs(y - peak_coord.y) <= excl_thresh) {
            continue;
        }
        count += 1;
        float delta = g[i*2] - mean;
        mean += delta/count;
        M2 += delta * (g[i*2] - mean);
    }
    float stddev = sqrtf((float)M2/count);
    return ((max - mean)/stddev);
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

void Tracker::complexMatAdd(float* in1, float* in2, float* out, int size) {
    //size in complex elements
    for (int i = 0; i < size; i++) {
        int idx = i * 2;
        out[idx]     = in1[idx] + in2[idx];
        out[idx + 1] = in1[idx + 1] + in2[idx + 1];
    }
}

void Tracker::elmWiseComplexDiv(float* in1, float* in2, int size, float* output) {
    //size in complex elements
    for (int i = 0; i < size; i++) {
        int idx = i * 2;
        float re1 = in1[idx];
        float im1 = in1[idx + 1];
        float re2 = in2[idx];
        float im2 = in2[idx + 1];

        float denom = (re2 * re2) + (im2 * im2);

        output[idx]     = (re1 * re2 + im1 * im2) / denom;
        output[idx + 1] = (im1 * re2 - re1 * im2) / denom;
    }

}

void Tracker::elmWiseComplexMult(float* in1, float* in2, int size, float* output, bool conjin1, bool conjin2) {
    // size in complex elements, conjin1 conjin2 set factors to conjugates
    float c1 = conjin1 ? -1.0f : 1.0f;
    float c2 = conjin2 ? -1.0f : 1.0f;

    for(int i = 0; i < size; i++) {
        int idx = i * 2;
        float re1 = in1[idx];
        float im1 = in1[idx + 1] * c1;
        float re2 = in2[idx];
        float im2 = in2[idx + 1] * c2;

        output[idx]     = re1 * re2 - im1 * im2;
        output[idx + 1] = re1 * im2 + im1 * re2;
    }
}

void Tracker::preprocessFrame(uint8_t* frame) {
    //takes a frame and then pads it using target.padded_width etc and stores the padded frame in target.data
    int count = 0;
    float mean = 0;
    float M2 = 0;
    int w = target.padded_width;
    int h = target.padded_height;
    int lower_x = (w - target.true_width)/2;
    int upper_x = lower_x + target.true_width;
    int lower_y = (h - target.true_height)/2;
    int upper_y = lower_y + target.true_height;
    for (int i = 0; i < w * h; i++) {
        int x = i % w;
        int y = i / w;
        if (x < lower_x || x >= upper_x || y < lower_y || y >= upper_y) {
            target.data[i] = 0;
        }
        else {
            x = x - lower_x;
            y = y - lower_y;
            int index = y * target.true_width + x;
            target.data[i] = log_lut[frame[index]]; 
            count += 1;
            float delta = target.data[i] - mean;
            mean += delta/count;
            M2 += delta * (target.data[i] - mean);
        }
    }
    float stddev = sqrtf((float)M2/count);
    for (int i = 0; i < w * h; i++) {
        int x = i % w;
        int y = i / w;
        if (x < lower_x || x >= upper_x || y < lower_y || y >= upper_y) {
            continue;
        }
        else {
            target.data[i] = ((target.data[i] - mean)/stddev) * hann_window_2D[i];
        }
    }
}

void Tracker::updateTarget(uint8_t* payload, size_t len, int height, int width, int x_pos, int y_pos) {
    if(xSemaphoreTake(this->target_lock, portMAX_DELAY)) {
        memcpy(preprocessbuf, payload, len);
        target.true_height = height;
        target.true_width = width;
        target.x_pos = x_pos;
        target.y_pos = y_pos;
        xSemaphoreGive(target_lock);
        int height_exp = ceil(log2(height));
        target.padded_height = 1 << height_exp;
        ESP_LOGI(TAG, "target.padded_height: %d", target.padded_height);
        int width_exp = ceil(log2(width));
        target.padded_width = 1 << width_exp;
        ESP_LOGI(TAG, "target.padded_width: %d", target.padded_width);
        preprocessFrame(preprocessbuf);
        initTracker();
        ESP_LOGI(TAG, "starting tracking..");
        isTracking = true;
    } 
}

void Tracker::generateRandomAffine(float* input, float* output, int rows, int cols, int& x_shift_out, int& y_shift_out) {
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

void Tracker::IFFT2D(float* input, float* output, int M, int N) {
    //intended for use with complex input, M, N refer to complex elements, i.e. size of input in bytes is M*N*2*sizeof(float)
    int size = M*N; 
    //conjugate input
    for (int i = 0; i < size; i++) {
        input[i * 2 + 1] = -1 * input[i * 2 + 1];
    }
    FFT2D(input, output, M, N, true);
    for (int i = 0; i < size; i ++) {
        output[i * 2] = (output[i * 2])/(M*N);
        output[i * 2 + 1] = (-1 * output[i * 2 + 1])/(M*N);
    }


}

void Tracker::FFT2D(float* input, float* output, int M, int N, bool complexInput) { 
    esp_err_t ret;

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
    // outer loops navigate frame
    for (int i = 0; i < M; i += TILE) {
        for (int j = 0; j < N; j += TILE) {
            // inner loops navigate 
            for (int ti = 0; ti < TILE && (i + ti) < M; ++ti) {
                for (int tj = 0; tj < TILE && (j + tj) < N; ++tj) {
                    int src_idx = (i + ti) * N + (j + tj);
                    int dst_idx = (j + tj) * M + (i + ti);
                    FFT_trans_buf[dst_idx * 2]     = FFT_buf[src_idx * 2];
                    FFT_trans_buf[dst_idx * 2 + 1] = FFT_buf[src_idx * 2 + 1];
                }
            }
        }
    }

    //apply fft to each col of og matrix by applying to each row of transpose
    for(int i = 0; i < N; i++) {
        float* col_ptr = FFT_trans_buf + i * M * 2;
        dsps_fft2r_fc32_aes3(col_ptr, M);
        dsps_bit_rev_fc32_ansi(col_ptr, M);
    }

    // transpose back
    // outer loops navigate frame
    for (int i = 0; i < M; i += TILE) {
        for (int j = 0; j < N; j += TILE) {
            // inner loops navigate window
            for (int ti = 0; ti < TILE && (i + ti) < M; ++ti) {
                for (int tj = 0; tj < TILE && (j + tj) < N; ++tj) {
                    int src_idx = (i + ti) * N + (j + tj);
                    int dst_idx = (j + tj) * M + (i + ti);
                    FFT_buf[dst_idx * 2]     = FFT_trans_buf[src_idx * 2];
                    FFT_buf[dst_idx * 2 + 1] = FFT_trans_buf[src_idx * 2 + 1];
                }
            }
        }
    }

    memcpy(output, FFT_buf, M*N*2*sizeof(float));
}

Coord Tracker::getTargetPOS() {
    Coord out = {target.x_pos, target.y_pos};
    return out;
}

