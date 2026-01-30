#include "mq135_handler.h"
#include "config.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_log.h"
#include <math.h>
#include <stdbool.h>  // ← ADD THIS at top
static const char *TAG = "MQ135";

// MQ135 calibration parameters
#define RL_VALUE 1.0f                    // Load resistance in kΩ (typical for Flying Fish module)
#define RO_CLEAN_AIR_FACTOR 3.6f         // RS/R0 in clean air
#define CALIBRATION_SAMPLE_COUNT 10      // Number of samples for calibration
#define CALIBRATION_SAMPLE_INTERVAL 100  // Delay between samples in ms

// Alcohol gas curve parameters (from MQUnifiedsensor library)
// Regression: ppm = A * (RS/R0)^B
#define ALCOHOL_A 77.255f
#define ALCOHOL_B -3.18f

static float R0 = 10.0f;  // Will be calibrated
static adc_oneshot_unit_handle_t adc1_handle;
static adc_cali_handle_t adc1_cali_handle = NULL;
static bool do_calibration = false;

// ADC Calibration Init
static bool adc_calibration_init(adc_unit_t unit, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "Calibration scheme version is Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "Calibration scheme version is Line Fitting");
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

    *out_handle = handle;
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "ADC Calibration Success");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    } else {
        ESP_LOGE(TAG, "Invalid arg or no memory");
    }

    return calibrated;
}

esp_err_t mq135_init(void) {
    // Configure ADC OneShot mode
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc1_handle));

    // Configure ADC channel
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_5, &config));

    // ADC Calibration
    do_calibration = adc_calibration_init(ADC_UNIT_1, ADC_ATTEN_DB_12, &adc1_cali_handle);
    
    ESP_LOGI(TAG, "Calibrating MQ135 sensor...");
    ESP_LOGI(TAG, "Please ensure sensor is in clean air");
    
    float sum_R0 = 0;
    
    for (int i = 0; i < CALIBRATION_SAMPLE_COUNT; i++) {
        // Read ADC value (average of 10 samples)
        int adc_raw = 0;
        int voltage_mv = 0;
        
        for (int j = 0; j < 10; j++) {
            int raw;
            ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_5, &raw));
            adc_raw += raw;
        }
        adc_raw /= 10;
        
        // Convert to voltage
        if (do_calibration) {
            ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_handle, adc_raw, &voltage_mv));
        } else {
            voltage_mv = (adc_raw * 3300) / 4095; // Simple linear conversion
        }
        
        float voltage = voltage_mv / 1000.0f;
        
        // Calculate RS (sensor resistance)
        if (voltage <= 0.0f || voltage >= 3.3f) {
            ESP_LOGW(TAG, "Invalid voltage reading during calibration: %.3fV", voltage);
            continue;
        }
        
        float RS = ((3.3f - voltage) / voltage) * RL_VALUE;
        sum_R0 += RS / RO_CLEAN_AIR_FACTOR;
        
        ESP_LOGI(TAG, "Calibration sample %d: ADC=%d, V=%.3f, RS=%.2f", 
                 i+1, adc_raw, voltage, RS);
        
        vTaskDelay(pdMS_TO_TICKS(CALIBRATION_SAMPLE_INTERVAL));
    }
    
    R0 = sum_R0 / CALIBRATION_SAMPLE_COUNT;
    
    if (R0 <= 0 || isinf(R0) || isnan(R0)) {
        ESP_LOGE(TAG, "❌ Calibration failed! Check wiring & power.");
        ESP_LOGE(TAG, "   R0 = %.2f (should be positive finite number)", R0);
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "✅ MQ135 calibrated successfully! R0 = %.2f kΩ", R0);
    return ESP_OK;
}

esp_err_t mq135_read_alcohol(float *alcohol) {
    // Read ADC value (average of 10 samples for stability)
    int adc_raw = 0;
    int voltage_mv = 0;
    
    for (int i = 0; i < 10; i++) {
        int raw;
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_5, &raw));
        adc_raw += raw;
    }
    adc_raw /= 10;
    
    // Convert to voltage
    if (do_calibration) {
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_handle, adc_raw, &voltage_mv));
    } else {
        voltage_mv = (adc_raw * 3300) / 4095;
    }
    
    float voltage = voltage_mv / 1000.0f;
    
    // Calculate RS (sensor resistance)
    if (voltage <= 0.0f || voltage >= 3.3f) {
        ESP_LOGW(TAG, "Invalid voltage reading: %.3fV", voltage);
        return ESP_ERR_INVALID_RESPONSE;
    }
    
    float RS = ((3.3f - voltage) / voltage) * RL_VALUE;
    
    // Calculate RS/R0 ratio
    float ratio = RS / R0;
    
    // Apply alcohol regression curve: ppm = A * ratio^B
    *alcohol = ALCOHOL_A * pow(ratio, ALCOHOL_B);
    
    // Validate reading (alcohol should be positive)
    if (*alcohol < 0 || isinf(*alcohol) || isnan(*alcohol)) {
        ESP_LOGW(TAG, "Invalid alcohol reading: %.2f ppm", *alcohol);
        return ESP_ERR_INVALID_RESPONSE;
    }
    
    return ESP_OK;
}