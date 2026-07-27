#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

static const char *TAG = "LAB3_STAT_FILTER";

#define TX_LED_R_GPIO        GPIO_NUM_4
#define TX_LED_G_GPIO        GPIO_NUM_5
#define TX_LED_B_GPIO        GPIO_NUM_18

#define RX_ADC_UNIT          ADC_UNIT_1
#define RX_ADC_CHANNEL       ADC_CHANNEL_3
#define V_REF                3300

#define NUM_SAMPLES          50    // สุ่มเก็บ 50 แซมเปิ้ลที่ไม่เป็น 0

static adc_cali_handle_t adc_cali_handle = NULL;
static bool do_cali = false;

// กำหนดสถานะสำหรับ Common Anode (ON = 0, OFF = 1)
#define LED_ON  0
#define LED_OFF 1

int compare_ints(const void *a, const void *b) {
    return (*(int*)a - *(int*)b);
}

// ฟังก์ชันปิด LED ภาคส่งทุกดวง
void turn_off_all_leds(void) {
    gpio_set_level(TX_LED_R_GPIO, LED_OFF);
    gpio_set_level(TX_LED_G_GPIO, LED_OFF);
    gpio_set_level(TX_LED_B_GPIO, LED_OFF);
}

void init_hardware(adc_oneshot_unit_handle_t *adc_handle)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << TX_LED_R_GPIO) | (1ULL << TX_LED_G_GPIO) | (1ULL << TX_LED_B_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    turn_off_all_leds();

    adc_oneshot_unit_init_cfg_t init_config = { .unit_id = RX_ADC_UNIT, .clk_src = ADC_DIGI_CLK_SRC_DEFAULT };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, adc_handle));
    adc_oneshot_chan_cfg_t chan_config = { .bitwidth = ADC_BITWIDTH_DEFAULT, .atten = ADC_ATTEN_DB_12 };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(*adc_handle, RX_ADC_CHANNEL, &chan_config));

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = RX_ADC_UNIT,
        .chan = RX_ADC_CHANNEL,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    if (adc_cali_create_scheme_curve_fitting(&cali_config, &adc_cali_handle) == ESP_OK) {
        do_cali = true;
    }
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    adc_cali_line_fitting_config_t cali_config = {
        .unit_id = RX_ADC_UNIT,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    if (adc_cali_create_scheme_line_fitting(&cali_config, &adc_cali_handle) == ESP_OK) {
        do_cali = true;
    }
#endif
}

void process_color_sensing(adc_oneshot_unit_handle_t adc_handle, gpio_num_t led_gpio, const char *color_name)
{
    int raw_samples[NUM_SAMPLES];

    // 1. สั่งเปิด LED สีที่ต้องการวัด (Common Anode จ่าย 0)
    turn_off_all_leds();
    gpio_set_level(led_gpio, LED_ON);

    // 2. หน่วงเวลา 1500ms ให้ระบบข้ามผ่านช่วง Transient State เข้าสู่ Steady State
    vTaskDelay(pdMS_TO_TICKS(1500));

    // 3. สุ่มเก็บข้อมูล 50 แซมเปิ้ล
    for (int i = 0; i < NUM_SAMPLES; i++) {
        int raw_value = 0;

        // วนอ่านซ้ำไปเรื่อยๆ จนกว่าจะได้ค่าที่ไม่ใช่ 0
        while (1) {
            esp_err_t ret = adc_oneshot_read(adc_handle, RX_ADC_CHANNEL, &raw_value);
            if (ret == ESP_OK && raw_value > 0) {
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(10)); // พัก 10ms แล้วลองอ่านใหม่
        }

        raw_samples[i] = raw_value;
        vTaskDelay(pdMS_TO_TICKS(130)); // ใช้ 130ms หลีกเลี่ยง Phase-locking 50Hz
    }

    // 4. อ่านค่าครบ 50  จึงสั่งปิด LED ทันที
    gpio_set_level(led_gpio, LED_OFF);

    // 5. จัดเรียงข้อมูลเพื่อทำ Trimmed Mean
    qsort(raw_samples, NUM_SAMPLES, sizeof(int), compare_ints);

    // 6. ตัดข้อมูลหัว-ท้ายออกฝั่งละ 20% (ฝั่งละ 10 ตัวอย่าง)
    int trim_count = NUM_SAMPLES * 0.20;
    int valid_count = NUM_SAMPLES - (2 * trim_count);

    double raw_sum = 0.0;
    for (int i = trim_count; i < NUM_SAMPLES - trim_count; i++) {
        raw_sum += raw_samples[i];
    }

    // 7. คำนวณค่าเฉลี่ยทางสถิติระดับบิต raw
    double mean_raw = raw_sum / valid_count;

    // 8. คำนวณค่าส่วนเบี่ยงเบนมาตรฐาน (SD) ระดับบิต raw
    double variance_sum = 0.0;
    for (int i = trim_count; i < NUM_SAMPLES - trim_count; i++) {
        variance_sum += pow((raw_samples[i] - mean_raw), 2);
    }
    double sd_raw = sqrt(variance_sum / (valid_count - 1));

    // 9. แปลงหน่วยเป็นมิลลิโวลต์ (mV Metadata)
    int final_voltage_mv = 0;
    double sd_voltage_mv = 0.0;

    if (do_cali) {
        adc_cali_raw_to_voltage(adc_cali_handle, (int)mean_raw, &final_voltage_mv);

        // คำนวณอัตราส่วน mV ต่อ Raw Bit เพื่อแปลงค่า SD ให้มีความเที่ยงตรงสูง
        int v_upper = 0, v_lower = 0;
        int raw_ref = (int)mean_raw;
        adc_cali_raw_to_voltage(adc_cali_handle, raw_ref + 10, &v_upper);
        adc_cali_raw_to_voltage(adc_cali_handle, raw_ref - 10 > 0 ? raw_ref - 10 : 0, &v_lower);

        double mv_per_bit = (double)(v_upper - v_lower) / 20.0;
        sd_voltage_mv = sd_raw * mv_per_bit;
    } else {
        final_voltage_mv = ((int)mean_raw * V_REF) / 4095;
        sd_voltage_mv = (sd_raw * V_REF) / 4095.0;
    }

    if (mean_raw <= 2.0) {
        final_voltage_mv = 0;
        sd_voltage_mv = 0.0;
    }

    // 10. พิมพ์ผลลัพธ์ข้อมูลเชิงสถิติออกทาง Serial Port
    printf("Color %s, n = %d (filtered), mean = %d, sd = %.2f\n",
           color_name, valid_count, final_voltage_mv, sd_voltage_mv);
}

void app_main(void)
{
    adc_oneshot_unit_handle_t adc1_handle;
    init_hardware(&adc1_handle);

    ESP_LOGI(TAG, "Statistical Signal Processing System Online.");
    printf("==============================================================\n");

    while (1) {
        process_color_sensing(adc1_handle, TX_LED_R_GPIO, "R");
        vTaskDelay(pdMS_TO_TICKS(1000));

        process_color_sensing(adc1_handle, TX_LED_G_GPIO, "G");
        vTaskDelay(pdMS_TO_TICKS(1000));

        process_color_sensing(adc1_handle, TX_LED_B_GPIO, "B");
        vTaskDelay(pdMS_TO_TICKS(1000));

        // หน่วงเวลาพักรอบระบบ 3 วินาที เพื่อรีเซ็ตและคายประจุแฝงบน PN Junction จนหมด
        printf("--------------------------------------------------------------\n");
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}