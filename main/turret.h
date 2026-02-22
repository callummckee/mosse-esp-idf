#pragma once

#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "hal/ledc_types.h"

#include "config.h"

class PID {
    public:
        float kp = 0.0;
        float ki = 0.0;
        float kd = 0.0;
        float iState = 0.0; 
        float prevErr = 0.0;

        PID() {}
        PID(float kp, float ki, float kd) : kp {kp}, ki{ki}, kd {kd} {}
        float loop(float error, float dt) {
            float p = error * kp;
            iState += dt * error;
            if (iState > intergratMax) iState = intergratMax;
            if (iState < intergratMin) iState = intergratMin;
            float i = iState * ki;
            float d = kd * (error - prevErr)/dt;
            float output = p + i + d;
            prevErr = error;
            return output;
        }

    private:

        const float intergratMax = 100.0;
        const float intergratMin = -100.0;
};

class Turret {
    public:
        Turret();
        ~Turret();

        float current_tilt_angle = 0;
        float current_pan_angle = 0;

        PID tiltpid = {};
        PID panpid = {};
        void applyPIDConfig(const PIDConfig& pidcfg);

        void set_servo_angle(ledc_channel_t channel, float angle);
        void move(float error, float time, ledc_channel_t channel);

        const ledc_channel_t pan_channel = LEDC_CHANNEL_0;
        const ledc_channel_t tilt_channel = LEDC_CHANNEL_1;
    private: 
        const float pan_min_angle = 0.0;
        const float pan_max_angle = 0.0;
        const float tilt_min_angle = 0.0;
        const float tilt_max_angle = 0.0;

        SemaphoreHandle_t configMutex = NULL;

        static void init_channel(int gpio_num, ledc_channel_t channel);
        static ledc_timer_config_t get_servo_config();

};

