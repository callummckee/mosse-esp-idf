#include "turret.h"
#include "driver/ledc.h"
#include "hal/ledc_types.h"
#include "pins.h"

#define SERVO_MODE LEDC_LOW_SPEED_MODE
#define SERVO_RES LEDC_TIMER_13_BIT
#define MAX_DUTY 8191 //2^13 - 1, thus angles can be set in 180/8191 \approx 0.02 degree increments
#define MIN_PULSE_US 500
#define MAX_PULSE_US 2500
#define PERIOD_US 20000

#define INITIAL_PAN_ANGLE 90
#define INITIAL_TILT_ANGLE 45

const PIDConfig defaultpidcdg = {0.05, 0.05, 0.2, 0.05, 0.05, 0.2};

Turret::Turret() {
    ledc_timer_config_t config_ledc = get_servo_config();
    ESP_ERROR_CHECK(ledc_timer_config(&config_ledc));
    init_channel(PAN_SERVO_PIN, pan_channel);
    init_channel(TILT_SERVO_PIN, tilt_channel);
    applyPIDConfig(defaultpidcdg);
    current_tilt_angle = INITIAL_TILT_ANGLE;
    current_pan_angle = INITIAL_PAN_ANGLE;
    set_servo_angle(tilt_channel, current_tilt_angle);
    set_servo_angle(pan_channel, current_pan_angle);

}

Turret::~Turret() {

}

void Turret::move(float error, float time, ledc_channel_t channel) {
    float max_angle = 0;
    float min_angle = 0;
    float* current_angle = 0;
    PID* pid;
    if (channel == pan_channel) {
        max_angle = pan_max_angle;
        min_angle = pan_min_angle;
        current_angle = &current_pan_angle;
        pid = &panpid;
    }
    else if (channel == tilt_channel) {
        max_angle = tilt_max_angle;
        min_angle = tilt_min_angle;
        current_angle = &current_tilt_angle;
        pid = &tiltpid;
    }
    else {
        return;
    }

    float shift = pid->loop(error, time);
    *current_angle += shift;
    if (*current_angle > max_angle) {
        *current_angle = max_angle;
    }
    if (*current_angle < min_angle) {
        *current_angle = min_angle;
    }
    set_servo_angle(channel, *current_angle);
}

void Turret::applyPIDConfig(const PIDConfig& pidcfg) {
    tiltpid.kd = pidcfg.tilt_kd;
    tiltpid.kp = pidcfg.tilt_kp;
    tiltpid.ki = pidcfg.tilt_ki;
    panpid.kd = pidcfg.pan_kd;
    panpid.kp = pidcfg.pan_kp;
    panpid.ki = pidcfg.pan_ki;
}

void Turret::init_channel(int gpio_num, ledc_channel_t channel) {
    ledc_channel_config_t channel_config = {};

    channel_config.gpio_num = gpio_num;
    channel_config.speed_mode = LEDC_LOW_SPEED_MODE;
    channel_config.channel = channel;
    channel_config.timer_sel = LEDC_TIMER_0;
    channel_config.duty = 0;
    channel_config.hpoint = 0;

    ledc_channel_config(&channel_config);

    return;
}

ledc_timer_config_t Turret::get_servo_config() {
    ledc_timer_config_t config = {};

    config.speed_mode = LEDC_LOW_SPEED_MODE;
    config.timer_num = LEDC_TIMER_0;
    config.duty_resolution = LEDC_TIMER_13_BIT;
    config.freq_hz = 50;
    config.clk_cfg = LEDC_USE_APB_CLK;

    return config; 
}

void Turret::set_servo_angle(ledc_channel_t channel, float angle) {
    if (angle < 0.0) {
        angle = 0.0;
    }
    else if (angle > 180.0) {
        angle = 180.0;
    }

    if (channel == LEDC_CHANNEL_1) {
        if (angle > 90.0) {
            angle = 90.0;
        }
    }
    // ESP_LOGI(TAG, "Setting servo on channel %d to angle %f", channel, angle);

    uint32_t pulse_us = MIN_PULSE_US + ((MAX_PULSE_US - MIN_PULSE_US) * angle/180); //convert angle to seconds
    uint32_t duty = (pulse_us * MAX_DUTY) / PERIOD_US; //convert seconds to ticks
    ledc_set_duty(SERVO_MODE, channel, duty);
    ledc_update_duty(SERVO_MODE, channel);

    return;
}

