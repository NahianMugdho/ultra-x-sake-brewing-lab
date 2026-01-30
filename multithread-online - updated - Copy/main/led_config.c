
#include "led_config.h"

void led_init(void) {
    // Timer config
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    // Channel config
    ledc_channel_config_t ledc_channel = {
        .gpio_num       = LED_GPIO,
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = LEDC_TIMER,
        .duty           = 0, // শুরুতে OFF
        .hpoint         = 0
    };
    ledc_channel_config(&ledc_channel);
}