#pragma once

#if defined(BOARD_ESP32_CAM)

#define PWDN_GPIO_NUM  32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM  0
#define PCLK_GPIO_NUM 22
#define SIOD_GPIO_NUM  26
#define SIOC_GPIO_NUM  27
#define Y9_GPIO_NUM    35
#define Y8_GPIO_NUM    34
#define Y7_GPIO_NUM    39
#define Y6_GPIO_NUM    36
#define Y5_GPIO_NUM    21
#define Y4_GPIO_NUM    19
#define Y3_GPIO_NUM    18
#define Y2_GPIO_NUM    5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM  23

#define PAN_SERVO_PIN 14
#define TILT_SERVO_PIN 15


#elif defined(BOARD_ESP32_S3)

#define PWDN_GPIO_NUM  -1 //hardwired on board
#define RESET_GPIO_NUM -1 //hardwired on board
#define XCLK_GPIO_NUM 15 
#define PCLK_GPIO_NUM 13 
#define SIOD_GPIO_NUM 4 
#define SIOC_GPIO_NUM 5 
#define Y9_GPIO_NUM   16 
#define Y8_GPIO_NUM    17
#define Y7_GPIO_NUM    18
#define Y6_GPIO_NUM    12
#define Y5_GPIO_NUM    10
#define Y4_GPIO_NUM    8
#define Y3_GPIO_NUM    9
#define Y2_GPIO_NUM    11
#define VSYNC_GPIO_NUM 6
#define HREF_GPIO_NUM  7

#define PAN_SERVO_PIN 38
#define TILT_SERVO_PIN 21


#else
    #error "no board in cmake"

#endif
