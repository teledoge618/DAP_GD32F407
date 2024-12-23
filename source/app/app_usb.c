#include "app_usb.h"

#include "gd32f4xx_libopt.h"
#include "pin_config.h"
#include "ulog.h"
#include "usbd_core.h"

/**
 * @brief 初始化USB外设
 *
 */
void APP_USB_Init(void) {
    /* USBPHY复位脚 */
    gpio_mode_set(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_7);
    gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_7);
    gpio_bit_reset(GPIOC, GPIO_PIN_7);

    /* MCO 24MHz作为PHY的晶振  */
    rcu_ckout1_config(RCU_CKOUT1SRC_HXTAL, 1); /* 直接输出24M晶振 */
    gpio_mode_set(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_9);
    gpio_af_set(GPIOC, GPIO_AF_0, GPIO_PIN_9);
    gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_MAX, GPIO_PIN_9);

    /* ULPI IOs */
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_5);
    gpio_mode_set(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_3);
    gpio_mode_set(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_2);
    gpio_mode_set(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_0);
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_5);
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_13);
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_12);
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_11);
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_10);
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_1);
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_0);
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_3);

    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_MAX, GPIO_PIN_5);
    gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_MAX, GPIO_PIN_3);
    gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_MAX, GPIO_PIN_2);
    gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_MAX, GPIO_PIN_0);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_MAX, GPIO_PIN_5);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_MAX, GPIO_PIN_13);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_MAX, GPIO_PIN_12);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_MAX, GPIO_PIN_11);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_MAX, GPIO_PIN_10);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_MAX, GPIO_PIN_1);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_MAX, GPIO_PIN_0);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_MAX, GPIO_PIN_3);

    gpio_af_set(GPIOA, GPIO_AF_10, GPIO_PIN_5);
    gpio_af_set(GPIOC, GPIO_AF_10, GPIO_PIN_3);
    gpio_af_set(GPIOC, GPIO_AF_10, GPIO_PIN_2);
    gpio_af_set(GPIOC, GPIO_AF_10, GPIO_PIN_0);
    gpio_af_set(GPIOB, GPIO_AF_10, GPIO_PIN_5);
    gpio_af_set(GPIOB, GPIO_AF_10, GPIO_PIN_13);
    gpio_af_set(GPIOB, GPIO_AF_10, GPIO_PIN_12);
    gpio_af_set(GPIOB, GPIO_AF_10, GPIO_PIN_11);
    gpio_af_set(GPIOB, GPIO_AF_10, GPIO_PIN_10);
    gpio_af_set(GPIOB, GPIO_AF_10, GPIO_PIN_1);
    gpio_af_set(GPIOB, GPIO_AF_10, GPIO_PIN_0);
    gpio_af_set(GPIOA, GPIO_AF_10, GPIO_PIN_3);

    /* 60MHz from PHY to USBHS */
    rcu_periph_clock_enable(RCU_USBHSULPI);
    rcu_periph_clock_enable(RCU_USBHS);
}

/******** for CherryUSB *******************************************************/

/**
 * @brief 开USB中断
 *
 * @param busid 0
 */
void usb_dc_low_level_init(uint8_t busid) {
    if (busid == 0) {
        nvic_irq_enable(USBHS_IRQn, 1, 0);
    }
}

/**
 * @brief 关USB中断
 *
 * @param busid 0
 */
void usb_dc_low_level_deinit(uint8_t busid) {
    if (busid == 0) {
        nvic_irq_disable(USBHS_IRQn);
    }
}

/* The USB IRQ handle of CherryUSB protocol stack */
extern void USBD_IRQHandler(uint8_t busid);

/**
 * @brief USB中断函数，套接到CherryUSB的处理函数
 *
 */
void USBHS_IRQHandler(void) {
    USBD_IRQHandler(0);  //
}
