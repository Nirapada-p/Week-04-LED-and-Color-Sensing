#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"

static const char *TAG = "LAB2_ADC_SETTLING";

// =========================================================================
// ปรับแก้ Pinout ตามโจทย์ (สำหรับ ESP32 38-Pin & Common Cathode)
// =========================================================================
#define TX_LED_R_GPIO        GPIO_NUM_4
#define TX_LED_G_GPIO        GPIO_NUM_5
#define TX_LED_B_GPIO        GPIO_NUM_18    // เปลี่ยนจาก GPIO6 เป็น GPIO18


// ขาภาครับอนาล็อก (เลือกใช้ GPIO39 ซึ่งตรงกับ ADC1_CHANNEL_3 บน ESP32)
#define RX_ADC_UNIT          ADC_UNIT_1
#define RX_ADC_CHANNEL       ADC_CHANNEL_3

#define NUM_SAMPLES          20
#define SAMPLING_DELAY_MS    300

void init_hardware(adc_oneshot_unit_handle_t *adc_handle)
{
    // 1. ตั้งค่าขาเอาต์พุตดิจิทัลสำหรับควบคุม LED RGB (Common Cathode)
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << TX_LED_R_GPIO) | (1ULL << TX_LED_G_GPIO) | (1ULL << TX_LED_B_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    gpio_set_level(TX_LED_R_GPIO, 1);
    gpio_set_level(TX_LED_G_GPIO, 1);
    gpio_set_level(TX_LED_B_GPIO, 1);

    // 2. ตั้งค่าหน่วย ADC Unit 1 (ไม่calibrate เพื่อดูค่า Raw 0-4095)
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = RX_ADC_UNIT,
        .clk_src = ADC_DIGI_CLK_SRC_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, adc_handle));

    // 3. ตั้งค่าขาสัญญาณอนาล็อก GPIO34 (12 บิต: 0 - 4095)
    adc_oneshot_chan_cfg_t chan_config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12, // รองรับช่วงแรงดัน 0 - 3.3V
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(*adc_handle, RX_ADC_CHANNEL, &chan_config));
}

void sample_and_print(adc_oneshot_unit_handle_t adc_handle, const char* phase_name)
{
    printf("Color %s:\n", phase_name);
    printf("No, ADC Raw\n");

    for (int i = 1; i <= NUM_SAMPLES; i++) {
        int raw_value = 0;
        ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, RX_ADC_CHANNEL, &raw_value));

        // พิมพ์ผลลัพธ์เป็นฟอร์แมต CSV (No, ADC Raw)
        printf("%d, %d\n", i, raw_value);

        vTaskDelay(pdMS_TO_TICKS(SAMPLING_DELAY_MS));
    }
}

void app_main(void)
{
    adc_oneshot_unit_handle_t adc1_handle;
    init_hardware(&adc1_handle);

    ESP_LOGI(TAG, "Transient Observation System Online (ESP32).");
    printf("==============================================================\n");

    while (1) {
        // --- รอบไฟสีแดง ---
        gpio_set_level(TX_LED_R_GPIO, 0);  // Common Cathode: HIGH = ติด
        vTaskDelay(pdMS_TO_TICKS(2500));
        gpio_set_level(TX_LED_R_GPIO, 1);  // HIGH = ดับเข้าสู่ช่วง Rest Phase
        sample_and_print(adc1_handle, "R");
        printf("--------------------------------------------------------------\n");

        // --- รอบไฟสีเขียว ---
        gpio_set_level(TX_LED_G_GPIO, 0);  // HIGH = ติด
        vTaskDelay(pdMS_TO_TICKS(2500));
        gpio_set_level(TX_LED_G_GPIO, 1);  // LOW = ดับเข้าสู่ช่วง Rest Phase
        sample_and_print(adc1_handle, "G");
        printf("--------------------------------------------------------------\n");

        // --- รอบไฟสีน้ำเงิน ---
        gpio_set_level(TX_LED_B_GPIO, 0);  // HIGH = ติด
        vTaskDelay(pdMS_TO_TICKS(2500));
        gpio_set_level(TX_LED_B_GPIO, 1);  // LOW = ดับเข้าสู่ช่วง Rest Phase
        sample_and_print(adc1_handle, "B");
        printf("==============================================================\n");
    }
}