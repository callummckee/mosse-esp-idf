#pragma once

#include "esp_random.h"
#include <cstdint>
#include <cmath>
#include "esp_log.h"


template <typename T>
T generateRandom(T mean, T sigma) {
    /* generates random numbers of type T normally distributed with mean mean and standard dev sigma using the box-muller transform 
     * can add Z1 using the same U1 and U2 and return two random variables to improve speed
     *
     *
     * */
    uint32_t U1seed = esp_random();
    uint32_t U2seed = esp_random();
    double U1 = (double)U1seed/(double)UINT32_MAX;
    double U2 = (double)U2seed/(double)UINT32_MAX;

    double Z0 = (sqrt(-2 * log(U1))) * cos(2 * M_PI * U2) * sigma + mean;
    return Z0;  
}


