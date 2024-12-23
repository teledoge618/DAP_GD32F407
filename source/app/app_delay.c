#include "app_delay.h"

#include "gd32f4xx_libopt.h"

/* 位带地址计算公式 */
#define BIT_BAND_PERIPH(reg_addr, nbit) (*((volatile uint8_t *)(uint32_t)(0x42000000 + ((reg_addr) - 0x40000000) * 32U + nbit * 4U)))
#define BIT_BAND_RAM(ram_addr, nbit) (*((volatile uint8_t *)(uint32_t)(0x22000000 + ((ram_addr) - 0x20000000) * 32U + nbit * 4U)))

/* T5 */
#define T5_ENABLE() (BIT_BAND_PERIPH((TIMER5 + 0x00), 0) = 0x01)
#define T5_IS_ENABLE() BIT_BAND_PERIPH((TIMER5 + 0x00), 0)
#define T5_DISABLE() (BIT_BAND_PERIPH((TIMER5 + 0x00), 0) = 0x00)
#define T5_SOFT_UPDATE() (BIT_BAND_PERIPH((TIMER5 + 0x14U), 0) = 0x01)
#define T5_UP_FLAG_CLEAR() (BIT_BAND_PERIPH((TIMER5 + 0x10U), 0) = 0x00)

void APP_Delay_Init(void) {
    rcu_periph_clock_enable(RCU_TIMER5);

    timer_deinit(TIMER5);
    timer_parameter_struct timer5_initpara;
    timer_struct_para_init(&timer5_initpara);
    timer5_initpara.prescaler = SystemCoreClock / 1000000 - 1;
    timer5_initpara.alignedmode = TIMER_COUNTER_UP;
    timer5_initpara.counterdirection = TIMER_COUNTER_UP;
    timer5_initpara.period = 0xFFFF;
    timer5_initpara.clockdivision = TIMER_CKDIV_DIV1;
    timer5_initpara.repetitioncounter = 0U;
    timer_init(TIMER5, &timer5_initpara);
    timer_enable(TIMER5);
    timer_single_pulse_mode_config(TIMER5, TIMER_SP_MODE_SINGLE); /* 单脉冲 */
}

void APP_Delay(uint32_t delay_us) {
    while (delay_us > 0xFFFF) {
        timer_counter_value_config(TIMER5, 0xFFFF);
        T5_ENABLE();
        while (T5_IS_ENABLE()) {
        }
        delay_us -= 0xFFFF;
    }

    timer_counter_value_config(TIMER5, 0xFFFF - delay_us);
    T5_ENABLE();
    while (T5_IS_ENABLE()) {
    }
}
