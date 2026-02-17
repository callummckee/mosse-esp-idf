#pragma once

#include <stdint.h>
#include "esp_cpu.h"


struct BenchmarkData {
    uint64_t cropprocess_cycles = 0;
    uint64_t fft_cycles = 0;
    uint64_t ifft_cycles = 0;
    uint64_t mult_cycles = 0;
    
    uint32_t avg_cropprocess = 0;
    uint32_t avg_fft = 0;
    uint32_t avg_ifft = 0;
    uint32_t avg_mult = 0;
    uint32_t frame_count = 0;
};

struct Benchmarker{
    uint64_t* data;
    uint32_t start_cycles;

    Benchmarker(uint64_t* acc) : data(acc) {
        start_cycles = esp_cpu_get_cycle_count();
    }

    ~Benchmarker() {
        uint32_t diff = esp_cpu_get_cycle_count() - start_cycles;
        *data += diff;
    }
};

