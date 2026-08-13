#pragma once

// ESP32-C3-LCDkit GPIO map (ESP32-C3-MINI-1).
// Source: https://docs.espressif.com/projects/esp-dev-kits/zh_CN/latest/esp32c3/esp32-c3-lcdkit/user_guide.html

#include "driver/gpio.h"

#ifndef BOARD_LCD_SDA
#define BOARD_LCD_SDA          GPIO_NUM_0
#define BOARD_LCD_SCL          GPIO_NUM_1
#define BOARD_LCD_DC           GPIO_NUM_2
#define BOARD_AUDIO_PA         GPIO_NUM_3
#define BOARD_IR               GPIO_NUM_4
#define BOARD_LCD_BL           GPIO_NUM_5
#define BOARD_ENCODER_B        GPIO_NUM_6
#define BOARD_LCD_CS           GPIO_NUM_7
#define BOARD_RGB_LED          GPIO_NUM_8
#define BOARD_ENCODER_SW       GPIO_NUM_9
#define BOARD_ENCODER_A        GPIO_NUM_10
#endif

#define HAP_DEVICE_NAME        "LCDkit Light"
#define HAP_SETUP_ID           "LCKT"

// Color temperature presets matching knob_panel warm / cool.
#define LIGHT_COOL_MIREDS      154u   // ~6500 K
#define LIGHT_WARM_MIREDS      370u   // ~2700 K
#define LIGHT_PWM_STEP         25
