// led_config.h
#pragma once
#include "driver/ledc.h"

#define LED_GPIO        2   // এখানে তোমার LED pin নাম্বার দাও
#define LEDC_CHANNEL    LEDC_CHANNEL_0
#define LEDC_TIMER      LEDC_TIMER_0
#define LEDC_MODE       LEDC_HIGH_SPEED_MODE
#define LEDC_DUTY_RES   LEDC_TIMER_8_BIT // 0-255 duty
#define LEDC_FREQUENCY  5000             // 5 KHz
void led_init(void);