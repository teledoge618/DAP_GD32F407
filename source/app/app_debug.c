#include "app_debug.h"

#include "gd32f4xx_libopt.h"
#include "pin_config.h"
#include "ulog.h"

/**
 * @brief 初始化调试串口
 *
 * @param bps 波特率
 */
void APP_Debug_UartInit(uint32_t bps) {
    rcu_periph_clock_enable(RCU_USART0);

    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_9);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_9);
    gpio_af_set(GPIOA, GPIO_AF_7, GPIO_PIN_9);

    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_10);
    gpio_af_set(GPIOA, GPIO_AF_7, GPIO_PIN_10);

    usart_deinit(USART0);
    usart_baudrate_set(USART0, bps);
    usart_word_length_set(USART0, USART_WL_8BIT);
    usart_parity_config(USART0, USART_PM_NONE);
    usart_stop_bit_set(USART0, USART_STB_1BIT);
    usart_hardware_flow_cts_config(USART0, USART_CTS_DISABLE);
    usart_hardware_flow_rts_config(USART0, USART_RTS_DISABLE);
    usart_enable(USART0);

    usart_transmit_config(USART0, USART_TRANSMIT_ENABLE);
}

/**
 * @brief 打印主频
 *
 */
void APP_Debug_PrintSclk(void) {
    ULOG_INFO("\r\n");                                        //
    ULOG_INFO("GD32F407VE sclk: %dHz\r\n", SystemCoreClock);  //
}

/**
 * @brief printf库单字节输出接口
 *
 * @param character
 */
void _putchar(char character) {
    while (RESET == usart_flag_get(USART0, USART_FLAG_TBE)) {
    }
    usart_data_transmit(USART0, character);  //
}
