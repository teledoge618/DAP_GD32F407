#include "app_adc.h"

#include "app_delay.h"
#include "gd32f4xx_libopt.h"

void APP_ADC_Init(void) {
    /* PC5 */
    gpio_mode_set(GPIOC, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO_PIN_15);

    rcu_periph_clock_enable(RCU_ADC0);
    adc_clock_config(ADC_ADCCK_PCLK2_DIV8);

    adc_data_alignment_config(ADC0, ADC_DATAALIGN_RIGHT);
    adc_channel_length_config(ADC0, ADC_ROUTINE_CHANNEL, 1);
    adc_sync_mode_config(ADC_SYNC_MODE_INDEPENDENT);
    adc_sync_delay_config(ADC_SYNC_DELAY_5CYCLE);
    adc_dma_mode_disable(ADC0);
    adc_special_function_config(ADC0, ADC_SCAN_MODE, DISABLE);                         // 非扫描模式
    adc_special_function_config(ADC0, ADC_CONTINUOUS_MODE, DISABLE);                   // 关闭连续转换
    adc_resolution_config(ADC0, ADC_RESOLUTION_12B);                                   // 12位分辨率
    adc_external_trigger_config(ADC0, ADC_ROUTINE_CHANNEL, EXTERNAL_TRIGGER_DISABLE);  // 外部触发禁止
    adc_routine_channel_config(ADC0, 0, ADC_CHANNEL_15, ADC_SAMPLETIME_56);

    adc_enable(ADC0);
    APP_Delay(10 * 1000);
    adc_calibration_enable(ADC0);
}

uint16_t APP_ADC_Read(uint8_t ch_index) {
    adc_software_trigger_enable(ADC0, ADC_ROUTINE_CHANNEL);

    while (adc_flag_get(ADC0, ADC_FLAG_EOC) == 0) {
    }
    adc_flag_clear(ADC0, ADC_FLAG_EOC);

    return adc_routine_data_read(ADC0) & 0x0FFF;
}
