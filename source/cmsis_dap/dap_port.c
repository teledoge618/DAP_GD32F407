#include "dap_port.h"

#include "gd32f4xx.h"
#include "gd32f4xx_libopt.h"
#include "ulog.h"

/* SWD和JTAG的相关配置信息，定义在dap.c */
extern dap_data_t dap_data;

static uint32_t swj_timer0_cnt = 0xFFFFFFFF;
static volatile uint8_t spi0_data_buf[4] = {0};
static volatile uint8_t spi2_data_buf[4] = {0};

// clang-format off

/* 奇偶校验快速计算表 */
static const uint8_t parity_mapping_table[256] = {
    0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 
    1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 
    1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 
    0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 
    1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 
    0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 
    0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 
    1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 
    1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 
    0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 
    0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 
    1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 
    0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 
    1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 
    1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 
    0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0};

// clang-format on

/* 位带地址计算公式 */
#define BIT_BAND_PERIPH(reg_addr, nbit) (*((volatile uint8_t *)(uint32_t)(0x42000000 + ((reg_addr) - 0x40000000) * 32U + nbit * 4U)))
#define BIT_BAND_RAM(ram_addr, nbit) (*((volatile uint8_t *)(uint32_t)(0x22000000 + ((ram_addr) - 0x20000000) * 32U + nbit * 4U)))

/* T0 */
#define T0_ENABLE() (BIT_BAND_PERIPH((TIMER0 + 0x00), 0) = 0x01)
#define T0_DISABLE() (BIT_BAND_PERIPH((TIMER0 + 0x00), 0) = 0x00)
#define T0_IS_ENABLE() BIT_BAND_PERIPH((TIMER0 + 0x00), 0)
#define T0_SOFT_UPDATE() (BIT_BAND_PERIPH((TIMER0 + 0x14U), 0) = 0x01)
#define T0_UP_FLAG_CLEAR() (BIT_BAND_PERIPH((TIMER0 + 0x10U), 0) = 0x00)

/* T3 */
#define T3_ENABLE() (BIT_BAND_PERIPH((TIMER3 + 0x00), 0) = 0x01)
#define T3_DISABLE() (BIT_BAND_PERIPH((TIMER3 + 0x00), 0) = 0x00)

/* SPI0 */
#define SPI0_ENABLE() (BIT_BAND_PERIPH((SPI0 + 0x00), 6) = 0x01)
#define SPI0_DISABLE() (BIT_BAND_PERIPH((SPI0 + 0x00), 6) = 0x00)
#define SPI0_INT_RBNE_ENABLE() (BIT_BAND_PERIPH((SPI0 + 0x04), 6) = 0x01)
#define SPI0_INT_RBNE_DISABLE() (BIT_BAND_PERIPH((SPI0 + 0x04), 6) = 0x00)
#define SPI0_RBNE() BIT_BAND_PERIPH((SPI0 + 0x08), 0)

/* SPI0 DMA */
#define SPI0_DMA_TX_ENABLE() (BIT_BAND_PERIPH((SPI0 + 0x04), 1) = 0x01)
#define SPI0_DMA_TX_DISABLE() (BIT_BAND_PERIPH((SPI0 + 0x04), 1) = 0x00)
#define SPI0_DMA_RX_ENABLE() (BIT_BAND_PERIPH((SPI0 + 0x04), 0) = 0x01)
#define SPI0_DMA_RX_DISABLE() (BIT_BAND_PERIPH((SPI0 + 0x04), 0) = 0x00)

/* SPI2 */
#define SPI2_ENABLE() (BIT_BAND_PERIPH((SPI2 + 0x00), 6) = 0x01)
#define SPI2_DISABLE() (BIT_BAND_PERIPH((SPI2 + 0x00), 6) = 0x00)
#define SPI2_INT_RBNE_ENABLE() (BIT_BAND_PERIPH((SPI2 + 0x04), 6) = 0x01)
#define SPI2_INT_RBNE_DISABLE() (BIT_BAND_PERIPH((SPI2 + 0x04), 6) = 0x00)
#define SPI2_RBNE() BIT_BAND_PERIPH((SPI2 + 0x08), 0)

/* SPI2 DMA */
#define SPI2_DMA_TX_ENABLE() (BIT_BAND_PERIPH((SPI2 + 0x04), 1) = 0x01)
#define SPI2_DMA_TX_DISABLE() (BIT_BAND_PERIPH((SPI2 + 0x04), 1) = 0x00)
#define SPI2_DMA_RX_ENABLE() (BIT_BAND_PERIPH((SPI2 + 0x04), 0) = 0x01)
#define SPI2_DMA_RX_DISABLE() (BIT_BAND_PERIPH((SPI2 + 0x04), 0) = 0x00)

/* DMA */
#define DMA1_CH5_ENABLE() (BIT_BAND_PERIPH((((DMA1) + 0x10U) + 0x18U * (DMA_CH5)), 0) = 0x01)
#define DMA1_CH5_DISABLE() (BIT_BAND_PERIPH((((DMA1) + 0x10U) + 0x18U * (DMA_CH5)), 0) = 0x00)
#define DMA1_CH5_IS_ENABLE() BIT_BAND_PERIPH((((DMA1) + 0x10U) + 0x18U * (DMA_CH5)), 0)
#define DMA1_CH0_ENABLE() (BIT_BAND_PERIPH((((DMA1) + 0x10U) + 0x18U * (DMA_CH0)), 0) = 0x01)
#define DMA1_CH0_DISABLE() (BIT_BAND_PERIPH((((DMA1) + 0x10U) + 0x18U * (DMA_CH0)), 0) = 0x00)
#define DMA1_CH0_IS_ENABLE() BIT_BAND_PERIPH((((DMA1) + 0x10U) + 0x18U * (DMA_CH0)), 0)

/* GPIO */
#define TMS_OEN_SET() (BIT_BAND_PERIPH(((GPIOD) + 0x18U), 12) = 0x01)
#define TMS_OEN_RESET() (BIT_BAND_PERIPH(((GPIOD) + 0x18U), 12 + 16) = 0x01) /* 高位清零 */
#define TMS_OEN_DO() (BIT_BAND_PERIPH(((GPIOD) + 0x14U), 12))
#define TMS_CTL_SET() (BIT_BAND_PERIPH(((GPIOA) + 0x18U), 6) = 0x01)
#define TMS_CTL_RESET() (BIT_BAND_PERIPH(((GPIOA) + 0x18U), 6 + 16) = 0x01) /* 高位清零 */
#define TMS_CTL_DO() (BIT_BAND_PERIPH(((GPIOA) + 0x14U), 6))

/* T5 */
#define T5_ENABLE() (BIT_BAND_PERIPH((TIMER5 + 0x00), 0) = 0x01)
#define T5_IS_ENABLE() BIT_BAND_PERIPH((TIMER5 + 0x00), 0)
#define T5_DISABLE() (BIT_BAND_PERIPH((TIMER5 + 0x00), 0) = 0x00)
#define T5_SOFT_UPDATE() (BIT_BAND_PERIPH((TIMER5 + 0x14U), 0) = 0x01)
#define T5_UP_FLAG_CLEAR() (BIT_BAND_PERIPH((TIMER5 + 0x10U), 0) = 0x00)

/**
 * @brief 小于1字节的TCK周期
 *
 * @param n_cycle 小于8
 */
__STATIC_INLINE void tck_cycle_n(uint16_t n_cycle) {
    if (n_cycle >= 8) {
        timer_channel_output_pulse_value_config(TIMER3, TIMER_CH_1, 0xFFFF);
    } else {
        uint16_t cnt = swj_timer0_cnt * n_cycle + swj_timer0_cnt / 4;
        timer_channel_output_pulse_value_config(TIMER3, TIMER_CH_1, cnt);
    }
    timer_repetition_value_config(TIMER0, 8 - 1);
    T0_SOFT_UPDATE();
    // T0_UP_FLAG_CLEAR();

    __set_BASEPRI(0x04); /* 屏蔽其他中断 */
    T3_ENABLE();
    T0_ENABLE();
    while (T0_IS_ENABLE()) {
    }

    T3_DISABLE();
    __set_BASEPRI(0x00);
    // T0_UP_FLAG_CLEAR();
}

/**
 * @brief 8倍数TCK周期
 *
 * @param n_byte
 */
__STATIC_INLINE void tck_cycle_byte(uint16_t n_byte) {
    timer_channel_output_pulse_value_config(TIMER3, TIMER_CH_1, 0xFFFF);
    timer_repetition_value_config(TIMER0, (8 * n_byte) - 1);
    T0_SOFT_UPDATE();
    // T0_UP_FLAG_CLEAR();

    __set_BASEPRI(0x04); /* 屏蔽其他中断 */
    T3_ENABLE();
    T0_ENABLE();
    while (T0_IS_ENABLE()) {
    }

    T3_DISABLE();
    __set_BASEPRI(0x00);
    // T0_UP_FLAG_CLEAR();
}

/* TMS传输方向控制 */
static void TmsOen(bool bit_enable) {
#if 1
    TMS_OEN_DO() = bit_enable;
#else
    if (bit_enable) {
        gpio_bit_set(GPIOD, GPIO_PIN_12);
        /* TMS_CTL */
        gpio_af_set(GPIOA, GPIO_AF_5, GPIO_PIN_6);
        gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_6);
    } else {
        gpio_bit_reset(GPIOD, GPIO_PIN_12);

        gpio_mode_set(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO_PIN_6);
    }
#endif
}

/* TMS_OEN复用模式控制  1自动 0手动 */
static void TmsOenAfMode(bool bit_enable) {
    if (bit_enable) {
        gpio_mode_set(GPIOD, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_12);
        gpio_af_set(GPIOD, GPIO_AF_2, GPIO_PIN_12); /* T3 */
    } else {
        gpio_mode_set(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_12);
    }
}

/* TMS复用模式控制 1自动 0手动 */
static void TmsAfMode(bool bit_enable) {
    if (bit_enable) {
        /* TMS_CTL */
        gpio_af_set(GPIOA, GPIO_AF_5, GPIO_PIN_6);
        gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_6);
        /* TMS_RET */
        gpio_af_set(GPIOA, GPIO_AF_5, GPIO_PIN_7);
        gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_7);
    } else {
        gpio_mode_set(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_6);
        gpio_mode_set(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO_PIN_7);
    }
}

/* TMS写 */
static void TmsWrite(bool bit_out) {
#if 1
    TMS_CTL_DO() = bit_out;
#else
    if (bit_out) {
        gpio_bit_set(GPIOA, GPIO_PIN_6);
    } else {
        gpio_bit_reset(GPIOA, GPIO_PIN_6);
    }
#endif
}

/* TMS读 */
static bool TmsRead() {
    return gpio_input_bit_get(GPIOA, GPIO_PIN_6);  //
}

/* TCK默认电平 TCK_DCTL */
static void TckPull(bool bit_out) {
    if (bit_out) {
        gpio_mode_set(GPIOD, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO_PIN_15);
    } else {
        gpio_mode_set(GPIOD, GPIO_MODE_INPUT, GPIO_PUPD_PULLDOWN, GPIO_PIN_15);
    }
}

/* TCK传输方向控制 */
static void TckOen(bool bit_enable) {
    if (bit_enable) {
        gpio_bit_set(GPIOC, GPIO_PIN_6);
    } else {
        gpio_bit_reset(GPIOC, GPIO_PIN_6);
    }
}

/**
 * @brief
 *
 */
void DAP_Port_InitHardware(void) {
    rcu_periph_clock_enable(RCU_SPI0);
    rcu_periph_clock_enable(RCU_SPI2);
    rcu_periph_clock_enable(RCU_TIMER0);
    rcu_periph_clock_enable(RCU_TIMER3);
    rcu_periph_clock_enable(RCU_TIMER4);
    rcu_periph_clock_enable(RCU_TIMER5);

    /* LED RUNNING = PE0 */
    gpio_bit_set(GPIOE, GPIO_PIN_0);
    gpio_mode_set(GPIOE, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_0);
    gpio_output_options_set(GPIOE, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_0);

    /* LED CONNECT = PE1 */
    gpio_bit_set(GPIOE, GPIO_PIN_1);
    gpio_mode_set(GPIOE, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_1);
    gpio_output_options_set(GPIOE, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_1);

    /* LED VOUT_EN = PE3 */
    gpio_bit_reset(GPIOE, GPIO_PIN_3);
    gpio_mode_set(GPIOE, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_3);
    gpio_output_options_set(GPIOE, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_3);

    /* 定时器计数频率等于AHB */
    rcu_timer_clock_prescaler_config(RCU_TIMER_PSC_MUL4);

    /* SPI0 APB2 AHB/2 160/2 = 80M */
    spi_parameter_struct spi0_initpara;
    spi_i2s_deinit(SPI0);
    spi_struct_para_init(&spi0_initpara);
    spi0_initpara.clock_polarity_phase = SPI_CK_PL_HIGH_PH_2EDGE;
    spi0_initpara.device_mode = SPI_SLAVE; /* 从机 */
    spi0_initpara.endian = SPI_ENDIAN_LSB;
    spi0_initpara.frame_size = SPI_FRAMESIZE_8BIT;
    spi0_initpara.trans_mode = SPI_TRANSMODE_FULLDUPLEX; /* 全双工 */
    spi0_initpara.nss = SPI_NSS_SOFT;
    spi0_initpara.prescale = SPI_PSC_2; /* 从机无意义，设为最高速度 */
    spi_init(SPI0, &spi0_initpara);
    SPI0_DISABLE();
    SPI0_INT_RBNE_ENABLE();
    spi_i2s_interrupt_enable(SPI0, SPI_I2S_INT_RBNE);
    nvic_irq_enable(SPI0_IRQn, 0, 0);

    /* SPI2 APB1 AHB/4 160/4 = 40M */
    spi_parameter_struct spi2_initpara;
    spi_i2s_deinit(SPI2);
    spi_struct_para_init(&spi2_initpara);
    spi2_initpara.clock_polarity_phase = SPI_CK_PL_HIGH_PH_2EDGE;
    spi2_initpara.device_mode = SPI_SLAVE; /* 从机 */
    spi2_initpara.endian = SPI_ENDIAN_LSB;
    spi2_initpara.frame_size = SPI_FRAMESIZE_8BIT;
    spi2_initpara.trans_mode = SPI_TRANSMODE_FULLDUPLEX; /* 全双工 */
    spi2_initpara.nss = SPI_NSS_SOFT;
    spi2_initpara.prescale = SPI_PSC_2; /* 从机无意义，设为最高速度 */
    spi_init(SPI2, &spi2_initpara);
    SPI2_DISABLE();
    SPI2_INT_RBNE_ENABLE();
    spi_i2s_interrupt_enable(SPI2, SPI_I2S_INT_RBNE);
    nvic_irq_enable(SPI2_IRQn, 0, 0);

    /* TIMER0 */
    timer_deinit(TIMER0);
    timer_parameter_struct timer0_initpara;
    timer_struct_para_init(&timer0_initpara);
    timer0_initpara.prescaler = 0U;
    timer0_initpara.alignedmode = TIMER_COUNTER_UP;
    timer0_initpara.counterdirection = TIMER_COUNTER_UP;
    timer0_initpara.period = (SystemCoreClock / dap_data.frequency) - 1U;
    timer0_initpara.clockdivision = TIMER_CKDIV_DIV1;
    timer0_initpara.repetitioncounter = 8U - 1U;
    timer_init(TIMER0, &timer0_initpara);
    timer_single_pulse_mode_config(TIMER0, TIMER_SP_MODE_SINGLE);

    timer_oc_parameter_struct timer0_ocintpara;
    timer0_ocintpara.outputstate = TIMER_CCX_ENABLE;
    timer0_ocintpara.outputnstate = TIMER_CCXN_DISABLE;
    timer0_ocintpara.ocpolarity = TIMER_OC_POLARITY_HIGH;
    timer0_ocintpara.ocnpolarity = TIMER_OCN_POLARITY_HIGH;
    timer0_ocintpara.ocidlestate = TIMER_OC_IDLE_STATE_HIGH;
    timer0_ocintpara.ocnidlestate = TIMER_OCN_IDLE_STATE_HIGH;
    timer_channel_output_config(TIMER0, TIMER_CH_0, &timer0_ocintpara);

    timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_0, (SystemCoreClock / dap_data.frequency) / 2);
    timer_channel_output_mode_config(TIMER0, TIMER_CH_0, TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER0, TIMER_CH_0, TIMER_OC_SHADOW_ENABLE);
    timer_primary_output_config(TIMER0, ENABLE);
    timer_auto_reload_shadow_disable(TIMER0);

    timer_master_output_trigger_source_select(TIMER0, TIMER_TRI_OUT_SRC_ENABLE); /* 输出使能信号 */
    timer_master_slave_mode_config(TIMER0, TIMER_MASTER_SLAVE_MODE_ENABLE);
    // timer_interrupt_enable(TIMER0, TIMER_INT_UP);
    // nvic_irq_enable(TIMER0_UP_TIMER9_IRQn, 0, 0); /* 中断太慢 */

    /* TIMER3 */
    timer_deinit(TIMER3);
    timer_parameter_struct timer3_initpara;
    timer_struct_para_init(&timer3_initpara);
    timer3_initpara.prescaler = 0U;
    timer3_initpara.alignedmode = TIMER_COUNTER_UP;
    timer3_initpara.counterdirection = TIMER_COUNTER_UP;
    timer3_initpara.period = (SystemCoreClock / dap_data.frequency) * 8 - 1U;
    timer3_initpara.clockdivision = TIMER_CKDIV_DIV1;
    timer3_initpara.repetitioncounter = 0U;
    timer_init(TIMER3, &timer3_initpara);
    // timer_single_pulse_mode_config(TIMER3, TIMER_SP_MODE_SINGLE);
    timer_dma_transfer_config(TIMER3, TIMER_DMACFG_DMATA_CH0CV, TIMER_DMACFG_DMATC_2TRANSFER);
    timer_channel_dma_request_source_select(TIMER3, TIMER_DMAREQUEST_UPDATEEVENT);

    timer_oc_parameter_struct timer3_ocintpara;
    timer3_ocintpara.outputstate = TIMER_CCX_ENABLE;
    timer3_ocintpara.outputnstate = TIMER_CCXN_DISABLE;
    timer3_ocintpara.ocpolarity = TIMER_OC_POLARITY_HIGH;
    timer3_ocintpara.ocnpolarity = TIMER_OCN_POLARITY_HIGH;
    timer3_ocintpara.ocidlestate = TIMER_OC_IDLE_STATE_HIGH;
    timer3_ocintpara.ocnidlestate = TIMER_OCN_IDLE_STATE_HIGH;
    timer_channel_output_config(TIMER3, TIMER_CH_0, &timer3_ocintpara);
    timer_channel_output_config(TIMER3, TIMER_CH_1, &timer3_ocintpara);

    timer_channel_output_pulse_value_config(TIMER3, TIMER_CH_0, 0xFFFF);
    timer_channel_output_pulse_value_config(TIMER3, TIMER_CH_1, 0xFFFF);
    timer_channel_output_mode_config(TIMER3, TIMER_CH_0, TIMER_OC_MODE_PWM0);
    timer_channel_output_mode_config(TIMER3, TIMER_CH_1, TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER3, TIMER_CH_0, TIMER_OC_SHADOW_ENABLE);
    timer_channel_output_shadow_config(TIMER3, TIMER_CH_1, TIMER_OC_SHADOW_ENABLE);
    timer_auto_reload_shadow_disable(TIMER3);

    timer_slave_mode_select(TIMER3, TIMER_SLAVE_MODE_RESTART);          /* 重置模式 */
    timer_input_trigger_source_select(TIMER3, TIMER_SMCFG_TRGSEL_ITI0); /* TIMER0 */
    // timer_interrupt_enable(TIMER3, TIMER_INT_UP);

    /* DMA SPI0 TX */
    dma_deinit(DMA1, DMA_CH5);
    dma_single_data_parameter_struct dma_spi_initpara;
    dma_spi_initpara.periph_addr = (uint32_t)&SPI_DATA(SPI0);
    dma_spi_initpara.periph_inc = DMA_PERIPH_INCREASE_DISABLE; /* 外设固定 */
    dma_spi_initpara.memory0_addr = (uint32_t)0;
    dma_spi_initpara.memory_inc = DMA_MEMORY_INCREASE_ENABLE;     /* 内存递增 */
    dma_spi_initpara.periph_memory_width = DMA_PERIPH_WIDTH_8BIT; /* 8bit */
    dma_spi_initpara.circular_mode = DMA_CIRCULAR_MODE_DISABLE;   /* 不循环 */
    dma_spi_initpara.direction = DMA_MEMORY_TO_PERIPH;
    dma_spi_initpara.number = 0xFFFF;
    dma_spi_initpara.priority = DMA_PRIORITY_MEDIUM;
    dma_single_data_mode_init(DMA1, DMA_CH5, &dma_spi_initpara);
    dma_channel_subperipheral_select(DMA1, DMA_CH5, DMA_SUBPERI3); /* SPI0 TX */
    dma_interrupt_enable(DMA1, DMA_CH5, DMA_INT_FTF);
    nvic_irq_enable(DMA1_Channel5_IRQn, 0, 2);

    /* DMA SPI0 RX */
    dma_deinit(DMA1, DMA_CH0);
    dma_spi_initpara.periph_addr = (uint32_t)&SPI_DATA(SPI0);
    dma_spi_initpara.periph_inc = DMA_PERIPH_INCREASE_DISABLE; /* 外设固定 */
    dma_spi_initpara.memory0_addr = (uint32_t)0;
    dma_spi_initpara.memory_inc = DMA_MEMORY_INCREASE_ENABLE;     /* 内存递增 */
    dma_spi_initpara.periph_memory_width = DMA_PERIPH_WIDTH_8BIT; /* 8bit */
    dma_spi_initpara.circular_mode = DMA_CIRCULAR_MODE_DISABLE;   /* 不循环 */
    dma_spi_initpara.direction = DMA_PERIPH_TO_MEMORY;
    dma_spi_initpara.number = 0xFFFF;
    dma_spi_initpara.priority = DMA_PRIORITY_MEDIUM;
    dma_single_data_mode_init(DMA1, DMA_CH0, &dma_spi_initpara);
    dma_channel_subperipheral_select(DMA1, DMA_CH0, DMA_SUBPERI3); /* SPI0 RX */
    dma_interrupt_enable(DMA1, DMA_CH0, DMA_INT_FTF);
    nvic_irq_enable(DMA1_Channel0_IRQn, 0, 2);

    /* DMA TIMER3 比较器 */
    dma_deinit(DMA0, DMA_CH6);
    dma_spi_initpara.periph_addr = (uint32_t)&TIMER_DMATB(TIMER3);
    dma_spi_initpara.periph_inc = DMA_PERIPH_INCREASE_DISABLE; /* 外设固定 */
    dma_spi_initpara.memory0_addr = (uint32_t)0;
    dma_spi_initpara.memory_inc = DMA_MEMORY_INCREASE_ENABLE;      /* 内存递增 */
    dma_spi_initpara.periph_memory_width = DMA_PERIPH_WIDTH_16BIT; /* 16bit */
    dma_spi_initpara.circular_mode = DMA_CIRCULAR_MODE_DISABLE;    /* 不循环 */
    dma_spi_initpara.direction = DMA_MEMORY_TO_PERIPH;
    dma_spi_initpara.number = 0xFFFF;
    dma_spi_initpara.priority = DMA_PRIORITY_HIGH; /* 优先级高 */
    dma_single_data_mode_init(DMA0, DMA_CH6, &dma_spi_initpara);
    dma_channel_subperipheral_select(DMA0, DMA_CH6, DMA_SUBPERI2); /* TIMER3 UP */
    dma_interrupt_enable(DMA0, DMA_CH6, DMA_INT_FTF);
    nvic_irq_enable(DMA0_Channel6_IRQn, 0, 3);

    /* TIMER4 时间戳 */
    timer_deinit(TIMER4);
    timer_parameter_struct timer4_initpara;
    timer_struct_para_init(&timer4_initpara);
    timer4_initpara.prescaler = SystemCoreClock / 1000000 - 1;
    timer4_initpara.alignedmode = TIMER_COUNTER_UP;
    timer4_initpara.counterdirection = TIMER_COUNTER_UP;
    timer4_initpara.period = 0xFFFFFFFF; /* 32bit */
    timer4_initpara.clockdivision = TIMER_CKDIV_DIV1;
    timer4_initpara.repetitioncounter = 0U;
    timer_init(TIMER4, &timer4_initpara);
    timer_enable(TIMER4);

    /* TIMER5 */
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

    DAP_Port_SWJ_Disconnect();
}

/**
 * @brief 设置Running灯的状态
 *
 * @param bit
 */
void DAP_Port_SetRunningStatus(uint8_t bit) {
    /* LED RUNNING = PE0 */
    if (bit) {
        gpio_bit_reset(GPIOE, GPIO_PIN_0);
    } else {
        gpio_bit_set(GPIOE, GPIO_PIN_0);
    }
}

/**
 * @brief 设置Connected灯的状态
 *
 * @param bit
 */
void DAP_Port_SetConnectedStatus(uint8_t bit) {
    /* LED CONNECT = PE1 */
    if (bit) {
        gpio_bit_reset(GPIOE, GPIO_PIN_1);
    } else {
        gpio_bit_set(GPIOE, GPIO_PIN_1);
    }
}

/**
 * @brief 初始化SWD端口
 *
 */
void DAP_Port_SWD_Connect(void) {
    /*
    SWD模式
    TCK_CLT:
        -PA8:  T0_CH0,    AF1  OUT 主时钟输出
        -PC10: SPI2_SCK,  AF6  IN  SPI从机时钟
        -PB3:  SPI0_SCK,  AF5  IN  SPI从机时钟
    SCK_OEN:
        -PD13: T3_CH1,    AF2  OUT 时钟输出使能
    TCK_OEN:
        -PC6:                  OUT TCK引脚输出使能
    TCK_DCTL:
        -PD15:                 OUT 高阻时默认电平
    TMS_CTL:
        -PA6:  SPI0_MISO, AF5  OUT SPI从机输出
    TMS_ERT:
        -PA7:  SPI0_MOSI, AF5  IN  SPI从机输入
    TMS_OEN:
        -PD12: T3_CH0,    AF2  OUT
    */

    /* TCK_CTL T0时钟输出，SPI0/2输入 */
    gpio_bit_set(GPIOA, GPIO_PIN_8);
    gpio_af_set(GPIOA, GPIO_AF_1, GPIO_PIN_8);                      /* T0_CH0 */
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_8); /* T0 */
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_8);

    gpio_af_set(GPIOC, GPIO_AF_6, GPIO_PIN_10); /* SPI0 */
    gpio_mode_set(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_10);

    gpio_af_set(GPIOB, GPIO_AF_5, GPIO_PIN_3); /* SPI2 */
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_3);

    /* TCK_OEN */
    TckOen(1);

    /* TCK_DCTL 默认电平高 */
    TckPull(1);

    /* TMS_CTL TMS_ERT 切到SPI0 */
    TmsAfMode(1);

    /* TMS_OEN */
    TMS_OEN_DO() = 1;
    TmsOenAfMode(0);

    SPI0_DISABLE();
    SPI2_DISABLE();
    SPI0_INT_RBNE_ENABLE();
    SPI2_INT_RBNE_ENABLE();
}

/**
 * @brief 初始化JTAG端口
 *
 */
void DAP_Port_JTAG_Connect(void) {
    /*
    JTAG模式
    TCK_CLT:
        -PA8:  T0_CH0,    AF1  OUT 主时钟输出
        -PC10: SPI2_SCK,  AF6  IN  SPI从机时钟
        -PB3:  SPI0_SCK,  AF5  IN  SPI从机时钟
    SCK_OEN:
        -PD13: T3_CH1,    AF2  OUT 时钟输出使能
    TCK_OEN:
        -PC6:                  OUT TCK引脚输出使能
    TCK_DCTL:
        -PD15:                 OUT 高阻时默认电平
    TMS_CTL:
        -PA6:  SPI0_MISO, AF5  OUT SPI从机输出
    TMS_ERT:
        -PA7:  SPI0_MOSI, AF5  IN  SPI从机输入
    TMS_OEN:
        -PD12: T3_CH0,    AF2  OUT
    TDI_CTL:
        -PC11: SPI2_MISO  AF6  OUT SPI输出
    TDI_OEN:
        -PC8:  GPIO            OUT
    TDO_CTL:
        -PD6:  SPI2_MOSI  AF5  IN  SPI输入
    TDO_OEN:
        -PA2:  GPIO            OUT
    TRST_CTL:
        -PD14: GPIO            OUT
    TRST_OEN:
        -PB14: GPIO            OUT

    */

    /* TCK_CTL */
    gpio_bit_set(GPIOA, GPIO_PIN_8);
    gpio_af_set(GPIOA, GPIO_AF_1, GPIO_PIN_8); /* T0_CH0 */
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_8);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_8);

    gpio_af_set(GPIOC, GPIO_AF_6, GPIO_PIN_10); /* SPI0 SCK */
    gpio_mode_set(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_10);

    gpio_af_set(GPIOB, GPIO_AF_5, GPIO_PIN_3); /* SPI2 SCK */
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_3);

    /* TCK_OEN */
    TckOen(1);

    /* TCK_DCTL */
    TckPull(1);

    /* TMS_CTL TMS_ERT */
    TmsAfMode(1);

    /* TMS_OEN */
    TMS_OEN_DO() = 1;
    TmsOenAfMode(0);

    /* TDI_CTL 切到SPI2输出 */
    gpio_af_set(GPIOC, GPIO_AF_6, GPIO_PIN_11); /* SPI2 MISO */
    gpio_mode_set(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_11);

    /* TDI_OEN 拉高 */
    gpio_bit_set(GPIOC, GPIO_PIN_8);

    /* TDO_CTL 切到SPI2输入 */
    gpio_af_set(GPIOD, GPIO_AF_5, GPIO_PIN_6); /* SPI2 MOSI */
    gpio_mode_set(GPIOD, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_6);

    /* TDO_OEN 拉低 */
    gpio_bit_reset(GPIOA, GPIO_PIN_2);

    /* TRST_CTL */
    gpio_bit_set(GPIOD, GPIO_PIN_14);

    /* TRST_OEN */
    gpio_bit_set(GPIOB, GPIO_PIN_14);

    SPI0_DISABLE();
    SPI2_DISABLE();
    SPI0_INT_RBNE_ENABLE();
    SPI2_INT_RBNE_ENABLE();
}

/**
 * @brief 所有引脚设为高阻输入状态
 *
 */
void DAP_Port_SWJ_Disconnect(void) {
    /* TCK_CTL PA8 PC10 PB3 */
    gpio_af_set(GPIOA, GPIO_AF_1, GPIO_PIN_8); /* T0_CH0 */
    gpio_mode_set(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO_PIN_8);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_8);

    gpio_af_set(GPIOC, GPIO_AF_6, GPIO_PIN_10); /* SPI0 SCK */
    gpio_mode_set(GPIOC, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO_PIN_10);

    gpio_af_set(GPIOB, GPIO_AF_5, GPIO_PIN_3); /* SPI2 SCK */
    gpio_mode_set(GPIOB, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO_PIN_3);

    /* SCK_OEN PD13 控制TCK周期个数 */
    gpio_af_set(GPIOD, GPIO_AF_2, GPIO_PIN_13); /* T3_CH1 */
    gpio_mode_set(GPIOD, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_13);
    gpio_output_options_set(GPIOD, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_13);

    /* TCK_OEN */
    gpio_bit_reset(GPIOC, GPIO_PIN_6); /* IN */
    gpio_mode_set(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_6);
    gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_6);

    /* TMS_CTL */
    gpio_mode_set(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO_PIN_6);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_6);

    /* TMS_ERT */
    gpio_mode_set(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO_PIN_7);

    /* TMS_OEN */
    gpio_bit_reset(GPIOD, GPIO_PIN_12); /* IN */
    gpio_mode_set(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_12);
    gpio_output_options_set(GPIOD, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_12);

    /* TDI_CTL */
    gpio_mode_set(GPIOC, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO_PIN_11);
    gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_11);
    gpio_mode_set(GPIOC, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO_PIN_12);
    gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_12);

    /* TDI_OEN */
    gpio_bit_reset(GPIOC, GPIO_PIN_8); /* IN */
    gpio_mode_set(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_8);
    gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_8);

    /* TDO_CTL */
    gpio_mode_set(GPIOD, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO_PIN_6);

    /* TDO_OEN */
    gpio_bit_reset(GPIOA, GPIO_PIN_2); /* IN */
    gpio_mode_set(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_2);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_2);

    /* RTCK_CTL */
    gpio_mode_set(GPIOE, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO_PIN_2);

    /* RTCK_OEN */
    gpio_bit_reset(GPIOE, GPIO_PIN_4); /* IN */
    gpio_mode_set(GPIOE, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_4);
    gpio_output_options_set(GPIOE, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_4);

    /* DBGRQ_CTL */
    gpio_mode_set(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO_PIN_1);

    /* DBGRQ_OEN */
    gpio_bit_reset(GPIOC, GPIO_PIN_1); /* IN */
    gpio_mode_set(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_1);
    gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_1);

    /* TRST_CTL */
    gpio_mode_set(GPIOD, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO_PIN_14);

    /* TRST_OEN */
    gpio_bit_reset(GPIOB, GPIO_PIN_14); /* IN */
    gpio_mode_set(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_14);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_14);

    /* SRST_CTL NPN开漏输出 */
    gpio_mode_set(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_0);
    gpio_bit_reset(GPIOA, GPIO_PIN_0);
}

/**
 * @brief 阻塞延时
 *
 * @param delay_us 微秒
 */
void DAP_Port_Delay(uint32_t delay_us) {
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

/**
 * @brief 复位目标芯片
 *
 * @return uint8_t 1复位，0未实现
 */
uint8_t DAP_Port_ResetTarget(void) {
    // TODO
    return 0;
}

/**
 * @brief 设置引脚状态
 *
 * @param select
 * @param value
 */
void DAP_Port_SetPins(uint8_t select, uint8_t value) {
    /*
    Bit 0: SWCLK/TCK
    Bit 1: SWDIO/TMS
    Bit 2: TDI
    Bit 3: TDO
    Bit 5: nTRST
    Bit 7: nRESET
    */

    if (select & (1 << DAP_SWJ_nRESET)) {
        gpio_bit_write(GPIOA, GPIO_PIN_0, (value >> DAP_SWJ_nRESET) & 0x01);
    }

    if (select & (1 << DAP_SWJ_nTRST)) {
        gpio_bit_write(GPIOD, GPIO_PIN_14, (value >> DAP_SWJ_nTRST) & 0x01);
    }
}

/**
 * @brief 读取引脚状态
 *
 * @return uint8_t
 */
uint8_t DAP_Port_GetPins(void) {
    uint8_t pins_value = 0;
    /*
    Bit 0: SWCLK/TCK
    Bit 1: SWDIO/TMS
    Bit 2: TDI
    Bit 3: TDO
    Bit 5: nTRST
    Bit 7: nRESET
    */

    if (gpio_input_bit_get(GPIOA, GPIO_PIN_0)) {
        pins_value |= (1U << DAP_SWJ_nRESET);
    }

    return pins_value;
}

/**
 * @brief 设置时钟输出频率
 *
 * @param freq
 */
void DAP_Port_SWJ_SetClock(uint32_t freq) {
    if (freq == 0) {
        freq = DAP_DEFAULT_SWJ_CLOCK;
        return;
    }
    if (freq > SystemCoreClock / 4) {
        freq = SystemCoreClock / 4; /* 上限SYS/4 */
    }
    if (freq < 40 * 1000) { /* 下限40K */
        if (freq < 10) {
            freq = 10;
        }
    }

    /* 计算每个周期的计数值 */
    uint32_t timer0_period = SystemCoreClock / freq;

    if (timer0_period > 4000) {
        uint32_t timer_psc = timer0_period / 4000;
        timer_prescaler_config(TIMER0, timer_psc - 1, TIMER_PSC_RELOAD_NOW);
        timer_prescaler_config(TIMER3, timer_psc - 1, TIMER_PSC_RELOAD_NOW);

        timer0_period /= timer_psc;
    } else {
        timer_prescaler_config(TIMER0, 0, TIMER_PSC_RELOAD_NOW);
        timer_prescaler_config(TIMER3, 0, TIMER_PSC_RELOAD_NOW);
    }
    dap_data.frequency = freq;
    ULOG_DEBUG("dap freq: %d Hz\r\n", freq);

    /* 设置TIMER0的计数值和比较值 */
    timer_autoreload_value_config(TIMER0, timer0_period - 1U);
    timer_autoreload_value_config(TIMER3, (timer0_period * 8) - 1U);
    timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_0, timer0_period / 2U); /* 50%占空比 */
    T0_SOFT_UPDATE();                                                                /* 手动更新进去 */
    T0_UP_FLAG_CLEAR();

    swj_timer0_cnt = timer0_period;
}

/**
 * @brief 输出数据序列
 *
 * @param count 比特数
 * @param value 串行数据
 */
void DAP_Port_SWJ_Sequence(uint32_t count, uint8_t *value) {
    uint8_t n_bit;

    /* 0代表256 */
    if (count == 0) {
        count = 256;
    }

    SPI0_ENABLE();

    while (count > 0) {
        if (count > 8) {
            n_bit = 8;
        } else {
            n_bit = count;
        }
        count -= n_bit;

        spi_i2s_data_transmit(SPI0, *value);
        tck_cycle_n(n_bit);
        value++;
    }

    SPI0_DISABLE();
}

/**
 * @brief 输出SWD序列
 *
 * @param info
 * @param request
 * @param response
 */
void DAP_Port_SWD_Sequence(uint8_t info, const uint8_t *request, uint8_t *response) {
    uint8_t count = info & SWD_SEQUENCE_CLK;
    if (count == 0) {
        count = 64;
    }

    /* 需要捕获输入值 */
    uint8_t need_receive = ((info & SWD_SEQUENCE_DIN) > 0);
    TMS_OEN_DO() = !need_receive; /* 传输方向 */

    if (need_receive) {
        while (SPI0_RBNE()) {
        }
        SPI0_INT_RBNE_DISABLE();
    }

    SPI0_ENABLE();

    while (count > 0) {
        uint8_t n_bit;
        if (count > 8) {
            n_bit = 8;
        } else {
            n_bit = count;
        }
        count -= n_bit;

        if (need_receive) {
            spi_i2s_data_transmit(SPI0, 0x00);
        } else {
            spi_i2s_data_transmit(SPI0, *request); /* 输出 */
            request++;
        }

        tck_cycle_n(n_bit);

        if (need_receive) {
            while (!SPI0_RBNE()) {
            }
            *response = spi_i2s_data_receive(SPI0); /* 输入 */
            response++;
        }
    }

    SPI0_INT_RBNE_ENABLE(); /* 抛弃 */
    SPI0_DISABLE();
    TMS_OEN_DO() = 1;
}

/**
 * @brief SWD传输
 *
 * @param request
 * @param response
 * @return uint32_t
 */
uint8_t DAP_Port_SWD_Transfer(uint32_t request, uint8_t *response) {
    /* 计算奇偶校验，奇1偶0 */
    uint8_t req_raw = request & 0x0F;
    uint8_t req_parity = parity_mapping_table[req_raw] & 0x01;
    uint8_t req = 0x81 | ((req_raw) << 1) | (req_parity << 5); /* req 8bit */
    uint8_t read_n_write = (request & DAP_TRANSFER_RnW) ? 0x01 : 0x00;

    /* 发request **************************************************************/

    SPI0_ENABLE();
    spi_i2s_data_transmit(SPI0, req);
    tck_cycle_n(8);

    /* 收ACK ******************************************************************/

    uint8_t trn_num = (read_n_write == 1U) ? 1U : 2U;                /* W-R-R 1trn cycle / W-R-W 2trn cycle */
    uint8_t ack_cycle = 3U + dap_data.swd_conf.turnaround * trn_num; /* 3 ACK + trn */
    if (ack_cycle > 8U) {
        if (ack_cycle > 16U) {
            /* 不可能出现，trn周期最多只有4，4+3+4=11 */
            ULOG_ERROR("ACK over length\r\n");
            return DAP_TRANSFER_ERROR; /* 不支持 */
        }
        SPI0_DISABLE();
        spi_i2s_data_frame_format_config(SPI0, SPI_FRAMESIZE_16BIT);
        SPI0_ENABLE();
    }

    TMS_OEN_DO() = 0; /* 输入 */
    while (SPI0_RBNE()) {
    }
    SPI0_INT_RBNE_DISABLE(); /* 手动接收 */

    if (ack_cycle > 8) {
        tck_cycle_n(8);
        tck_cycle_n(ack_cycle - 8);
    } else {
        tck_cycle_n(ack_cycle);
    }

    while (!SPI0_RBNE()) {
    }
    uint16_t ack_raw = spi_i2s_data_receive(SPI0);
    uint8_t ack = 0x07 & (ack_raw >> dap_data.swd_conf.turnaround); /* 去掉trn */

    SPI0_INT_RBNE_ENABLE(); /* 抛弃 */

    /* 恢复8bit长度 */
    if (ack_cycle > 8U) {
        SPI0_DISABLE();
        spi_i2s_data_frame_format_config(SPI0, SPI_FRAMESIZE_8BIT);
        SPI0_ENABLE();
    }

    uint8_t need_data_phase = 1; /* 需要生成数据序列 */
    if (ack == DAP_TRANSFER_OK) {
        need_data_phase = 1;

        if (request & DAP_TRANSFER_TIMESTAMP) {
            dap_data.timestamp = DAP_Port_GetTimeStamp();
        }
    } else if ((ack == DAP_TRANSFER_WAIT) || (ack == DAP_TRANSFER_FAULT)) {
        need_data_phase = dap_data.swd_conf.data_phase;
    }

    /* data *******************************************************************/

    if (need_data_phase) {
        /* WDATA校验值 */
        uint8_t wdata_parity = 0x01 & (parity_mapping_table[response[0]] + parity_mapping_table[response[1]]  //
                                       + parity_mapping_table[response[2]] + parity_mapping_table[response[3]]);

        if (read_n_write) {
            TMS_OEN_DO() = 0; /* 输入 */

            while (SPI0_RBNE()) {
            }
            SPI0_INT_RBNE_DISABLE();
            dma_memory_address_config(DMA1, DMA_CH0, DMA_MEMORY_0, (uint32_t)spi0_data_buf); /* RX */
            dma_transfer_number_config(DMA1, DMA_CH0, 4);                                    /* SPI0 */
            DMA1_CH0_ENABLE();
            SPI0_DMA_RX_ENABLE(); /* RX */
        } else {
            TMS_OEN_DO() = 1; /* 输出 */

            dma_memory_address_config(DMA1, DMA_CH5, DMA_MEMORY_0, (uint32_t)response); /* TX */
            dma_transfer_number_config(DMA1, DMA_CH5, 4);                               /* SPI0 */
            DMA1_CH5_ENABLE();
            SPI0_DMA_TX_ENABLE(); /* TX */
        }

        tck_cycle_byte(4); /* 4*8=32 */

        if (read_n_write) {
            while (DMA1_CH0_IS_ENABLE()) {
            }

            SPI0_DMA_RX_DISABLE();
            response[0] = spi0_data_buf[0];
            response[1] = spi0_data_buf[1];
            response[2] = spi0_data_buf[2];
            response[3] = spi0_data_buf[3];
        } else {
            SPI0_DMA_TX_DISABLE();
        }

        SPI0_INT_RBNE_ENABLE();

        /* 1bit校验位 和 可能的转换位 *******************/

        while (SPI0_RBNE()) {
        }
        SPI0_INT_RBNE_DISABLE();
        spi_i2s_data_transmit(SPI0, wdata_parity);
        tck_cycle_n(dap_data.swd_conf.turnaround * read_n_write + 1U);

        if (read_n_write) {
            while (!SPI0_RBNE()) { /* 等数据 */
            }
            /* 计算校验值 */
            uint8_t rdata_parity = 0x01 & spi_i2s_data_receive(SPI0);
            uint8_t p = 0x01 & (parity_mapping_table[spi0_data_buf[0]] + parity_mapping_table[spi0_data_buf[1]]  //
                                + parity_mapping_table[spi0_data_buf[2]] + parity_mapping_table[spi0_data_buf[3]]);

            if (p != rdata_parity) {
                ack = DAP_TRANSFER_ERROR; /* 校验值错误 */
                ULOG_ERROR("data parity error\r\n");
            }
        }

        SPI0_INT_RBNE_ENABLE();
        TMS_OEN_DO() = 1; /* 恢复控制权 */

        /* 写指令后需要添加空闲周期 */
        uint8_t idle_count = dap_data.transfer.idle_cycles;
        if ((!read_n_write) && (idle_count > 0)) {
            spi_i2s_data_transmit(SPI0, 0x00);
            while (idle_count > 0) {
                uint8_t idle_bit;
                if (idle_count > 8) {
                    idle_bit = 8;
                } else {
                    idle_bit = idle_count;
                }
                idle_count -= idle_bit;
                tck_cycle_n(idle_bit);
            }
        }
    } else {
        if (read_n_write) {
            /* 读操作失败添加一个trn周期，重新掌握数据线控制权 */
            tck_cycle_n(dap_data.swd_conf.turnaround);
        }
    }

    TMS_OEN_DO() = 1; /* 输出 */
    SPI0_DISABLE();
    return ack;
}

/**
 * @brief 输出JTAG序列
 *
 * @param info
 * @param request
 * @param response
 */
void DAP_Port_JTAG_Sequence(uint8_t info, const uint8_t *request, uint8_t *response) {
    uint32_t n_bit;

    /* 周期数 */
    uint32_t count = info & JTAG_SEQUENCE_TCK;
    if (count == 0) {
        count = 64;
    }

    bool need_receive = (info & JTAG_SEQUENCE_TDO) > 0;

    if (need_receive) {
        SPI2_INT_RBNE_DISABLE();
    } else {
        SPI2_INT_RBNE_ENABLE();
    }

    SPI0_ENABLE();
    SPI2_ENABLE();
    while (count > 0) {
        if (count > 8) {
            n_bit = 8;
        } else {
            n_bit = count;
        }
        count -= n_bit;

        if (need_receive) {
            spi_i2s_data_transmit(SPI2, 0xFF);
        } else {
            spi_i2s_data_transmit(SPI2, *request); /* 发送 */
            request++;
        }
        spi_i2s_data_transmit(SPI0, (info & JTAG_SEQUENCE_TMS) ? 0xFF : 0x00); /* 保持TMS以维持TAP状态 */
        tck_cycle_n(n_bit);

        if (need_receive) {
            while (!SPI2_RBNE()) { /* 等TDO */
            }
            *response = spi_i2s_data_receive(SPI2); /* 接收 */
            response++;
        }
    }

    SPI0_DISABLE();
    SPI2_DISABLE();
    SPI2_INT_RBNE_ENABLE();
}

/**
 * @brief JTAG传输
 *
 * @param request
 * @param data
 * @return uint8_t
 */
uint8_t DAP_Port_JTAG_Transfer(uint32_t request, uint8_t *rwdata) {
    /* Run-Test/Idle -> Shift-DR **********************************************/

    SPI0_ENABLE();
    SPI2_ENABLE();
    spi_i2s_data_transmit(SPI0, B001); /* 1,0,0 */
    spi_i2s_data_transmit(SPI2, 0xFF); /* 无所谓 */
    tck_cycle_n(3);                    /* 不能多，多了就错位了 */

    /* TDO <- device[0] <- ... <- device[n] <- TDI */

    /* 把数据从Bypass中推出来 ****************************************************/
    uint8_t bypass_num = dap_data.jtag_dev.index;
    if (bypass_num) {
        uint8_t i;

        while (bypass_num) {
            if (bypass_num > 8) {
                i = 8;
            } else {
                i = bypass_num;
            }
            bypass_num -= i;

            spi_i2s_data_transmit(SPI0, 0x00); /* Shift-DR */
            spi_i2s_data_transmit(SPI2, 0xFF); /* 无所谓 */
            tck_cycle_n(i);
        }
    }

    /* 接收ACK并发送req *********************************************************/

    SPI2_INT_RBNE_DISABLE();                   /* 手动接收 */
    spi_i2s_data_transmit(SPI0, 0x00);         /* Shift-DR */
    spi_i2s_data_transmit(SPI2, request >> 1); /* REQ */
    tck_cycle_n(3);

    while (!SPI2_RBNE()) { /* 等数据 */
    }
    uint8_t ack = spi_i2s_data_receive(SPI2) & 0x07;
    ack = ((ack & 0x01) << 1) | ((ack & 0x02) >> 1) | (ack & 0x04); /* 交换[0][1] */
    SPI2_INT_RBNE_ENABLE();

    if (ack != DAP_TRANSFER_OK) {
        spi_i2s_data_transmit(SPI0, B011); /* 1,1,0 */
        spi_i2s_data_transmit(SPI2, 0xFF); /* 无所谓 */
        tck_cycle_n(8);                    /* 多了无所谓 */
        SPI0_DISABLE();
        SPI2_DISABLE();

        if (request & DAP_TRANSFER_TIMESTAMP) {
            dap_data.timestamp = DAP_Port_GetTimeStamp();
        }

        return ack;
    }

    if (request & DAP_TRANSFER_RnW) {
        /* 读操作 **************************************************************/

        SPI2_INT_RBNE_DISABLE();
        spi_i2s_data_transmit(SPI0, 0x00); /* Shift-DR */
        spi_i2s_data_transmit(SPI2, 0xFF); /*  */
        tck_cycle_n(8);
        while (!SPI2_RBNE()) {
        }
        rwdata[0] = spi_i2s_data_receive(SPI2);

        spi_i2s_data_transmit(SPI0, 0x00); /* Shift-DR */
        spi_i2s_data_transmit(SPI2, 0xFF); /*  */
        tck_cycle_n(8);
        while (!SPI2_RBNE()) {
        }
        rwdata[1] = spi_i2s_data_receive(SPI2);

        spi_i2s_data_transmit(SPI0, 0x00); /* Shift-DR */
        spi_i2s_data_transmit(SPI2, 0xFF); /*  */
        tck_cycle_n(8);
        while (!SPI2_RBNE()) {
        }
        rwdata[2] = spi_i2s_data_receive(SPI2);

        uint8_t n = dap_data.jtag_dev.count - dap_data.jtag_dev.index - 1U;
        if (n > 0) {
            /* 发送数据 ********************************************************/

            spi_i2s_data_transmit(SPI0, 0x00); /* Shift-DR */
            spi_i2s_data_transmit(SPI2, 0xFF); /*  */
            tck_cycle_n(8);
            while (!SPI2_RBNE()) {
            }
            rwdata[3] = spi_i2s_data_receive(SPI2);

            SPI2_INT_RBNE_ENABLE();

            /* 把数据推到合适的位置 **********************************************/

            while (n > 0) {
                uint8_t i;
                if (n > 8) {
                    i = 8;
                } else {
                    i = n;
                }
                n -= i;

                if (n) {
                    spi_i2s_data_transmit(SPI0, 0x00); /* Shift-DR */
                } else {
                    spi_i2s_data_transmit(SPI0, 0x01 << (i - 1)); /* Shift-DR -> Exit1-DR */
                }
                spi_i2s_data_transmit(SPI2, 0xFF); /*  */
                tck_cycle_n(i);
            }
        } else {
            /* 发送数据并进入下一个状态 *******************************************/

            spi_i2s_data_transmit(SPI0, 0x01 << 7); /* Shift-DR -> Exit1-DR */
            spi_i2s_data_transmit(SPI2, 0xFF);      /*  */
            tck_cycle_n(8);
            while (!SPI2_RBNE()) {
            }
            rwdata[3] = spi_i2s_data_receive(SPI2);

            SPI2_INT_RBNE_ENABLE();
        }
    } else {
        /* 写操作 **************************************************************/

        spi_i2s_data_transmit(SPI0, 0x00);      /* Shift-DR */
        spi_i2s_data_transmit(SPI2, rwdata[0]); /*  */
        tck_cycle_n(8);

        spi_i2s_data_transmit(SPI0, 0x00);      /* Shift-DR */
        spi_i2s_data_transmit(SPI2, rwdata[1]); /*  */
        tck_cycle_n(8);

        spi_i2s_data_transmit(SPI0, 0x00);      /* Shift-DR */
        spi_i2s_data_transmit(SPI2, rwdata[2]); /*  */
        tck_cycle_n(8);

        uint8_t n = dap_data.jtag_dev.count - dap_data.jtag_dev.index - 1U;
        if (n > 0) {
            spi_i2s_data_transmit(SPI0, 0x00);      /* Shift-DR */
            spi_i2s_data_transmit(SPI2, rwdata[3]); /*  */
            tck_cycle_n(8);

            /* 把数据推到合适的位置 **********************************************/
            while (n > 0) {
                uint8_t i;
                if (n > 8) {
                    i = 8;
                } else {
                    i = n;
                }
                n -= i;

                if (n) {
                    spi_i2s_data_transmit(SPI0, 0x00); /* Shift-DR */
                } else {
                    spi_i2s_data_transmit(SPI0, 0x01 << (i - 1)); /* Shift-DR -> Exit1-DR */
                }
                spi_i2s_data_transmit(SPI2, 0xFF); /*  */
                tck_cycle_n(i);
            }
        } else {
            spi_i2s_data_transmit(SPI0, 0x01 << 7); /* Shift-DR -> Exit1-DR */
            spi_i2s_data_transmit(SPI2, rwdata[3]); /*  */
            tck_cycle_n(8);
        }
    }

    /* 退出 ********************************************************************/

    spi_i2s_data_transmit(SPI0, B00000001); /* 1,0 */
    spi_i2s_data_transmit(SPI2, 0xFF);      /* 无所谓 */
    tck_cycle_n(8);                         /* 多几个bit也没问题 */
    SPI0_DISABLE();
    SPI2_DISABLE();

    if (request & DAP_TRANSFER_TIMESTAMP) {
        dap_data.timestamp = DAP_Port_GetTimeStamp();
    }

    return ack;
}

/**
 * @brief 写JTAG IR寄存器
 *
 * @param ir
 */
void DAP_Port_JTAG_IR(uint32_t ir) {
    uint32_t ir_len_befor = dap_data.jtag_dev.ir_before[dap_data.jtag_dev.index];
    uint32_t ir_len_now = dap_data.jtag_dev.ir_length[dap_data.jtag_dev.index];
    uint32_t ir_len_after = dap_data.jtag_dev.ir_after[dap_data.jtag_dev.index];

    /* Run-Test/Idle -> Shift-IR **********************************************/

    SPI0_ENABLE();
    SPI2_ENABLE();
    spi_i2s_data_transmit(SPI0, B0011); /* 1,1,0,0 */
    spi_i2s_data_transmit(SPI2, 0xFF);  /* 无所谓 */
    tck_cycle_n(4);                     /* 不能多，多了就写进IR了 */

    /* Shift-IR -> Exit1-IR ***************************************************/

    /* 前面的IR都塞满1，进入Bypass */
    while (ir_len_befor > 0) {
        uint32_t n_bit;
        if (ir_len_befor > 8) {
            n_bit = 8;
        } else {
            n_bit = ir_len_befor;
        }
        ir_len_befor -= n_bit; /* 剩余 */

        spi_i2s_data_transmit(SPI0, 0x00); /* 保持Shift-IR */
        spi_i2s_data_transmit(SPI2, 0xFF); /* 填充Bypass */
        tck_cycle_n(n_bit);
    }

    /* 写IR，最后1bit需要同时拉高TMS，进Exit1-IR */
    if (ir_len_after > 0) {
        /* 直接写IR */
        while (ir_len_now > 0) {
            uint8_t n_bit;
            if (ir_len_now > 8) {
                n_bit = 8;
            } else {
                n_bit = ir_len_now;
            }
            ir_len_now -= n_bit; /* 剩余 */

            spi_i2s_data_transmit(SPI0, 0x00);      /* 保持Shift-IR */
            spi_i2s_data_transmit(SPI2, ir & 0xFF); /* IR */
            tck_cycle_n(n_bit);

            ir = (ir >> n_bit);
        }

        /* 后面的IR都塞满1，进入Bypass，最后拉高TMS */
        while (ir_len_after > 0) {
            uint8_t n_bit;
            if (ir_len_after > 8) {
                n_bit = 8;
            } else {
                n_bit = ir_len_after;
            }
            ir_len_after -= n_bit; /* 剩余 */

            if (ir_len_after) {
                spi_i2s_data_transmit(SPI0, 0x00); /* 保持Shift-IR */
            } else {
                spi_i2s_data_transmit(SPI0, (1 << (n_bit - 1))); /* 最后一位拉起TMS */
            }
            spi_i2s_data_transmit(SPI2, 0xFF); /* Bypass */
            tck_cycle_n(n_bit);
        }
    } else {
        /* 写IR，最后1bit拉高TMS */
        while (ir_len_now > 0) {
            uint8_t n_bit;
            if (ir_len_now > 8) {
                n_bit = 8;
            } else {
                n_bit = ir_len_now;
            }
            ir_len_now -= n_bit; /* 剩余 */

            if (ir_len_now) {
                spi_i2s_data_transmit(SPI0, 0x00); /* 保持Shift-IR */
            } else {
                spi_i2s_data_transmit(SPI0, (1 << (n_bit - 1))); /* 最后一位拉起TMS */
            }
            spi_i2s_data_transmit(SPI2, ir & 0xFF);
            tck_cycle_n(n_bit);

            if (ir_len_now) {
                ir = (ir >> n_bit);
            }
        }
    }

    /* Exit1-IR -> Run-Test/Idle **********************************************/

    spi_i2s_data_transmit(SPI0, B00000001); /* 1,0 */
    spi_i2s_data_transmit(SPI2, 0xFF);      /* 无所谓 */
    tck_cycle_n(8);                         /* 多发0也没关系 */
    SPI0_DISABLE();
    SPI2_DISABLE();
}

/**
 * @brief JTAG读取IDCODE
 *
 * @return uint32_t
 */
uint32_t DAP_Port_JTAG_ReadIDCode(void) {
    uint8_t idcode[4];

    /* Run-Test/Idle -> Shift-IR **********************************************/

    SPI0_ENABLE();
    SPI2_ENABLE();
    spi_i2s_data_transmit(SPI0, B001);
    spi_i2s_data_transmit(SPI2, 0xFF);
    tck_cycle_n(3);

    /* 把数据从Bypass中推出来 ****************************************************/
    uint8_t bypass_num = dap_data.jtag_dev.index;
    if (bypass_num) {
        uint8_t i;

        while (bypass_num) {
            if (bypass_num > 8) {
                i = 8;
            } else {
                i = bypass_num;
            }
            bypass_num -= i;

            spi_i2s_data_transmit(SPI0, 0x00); /* Shift-DR */
            spi_i2s_data_transmit(SPI2, 0xFF);
            tck_cycle_n(i);
        }
    }

    /* Shift-IR -> Exit1-DR ***************************************************/

    SPI2_INT_RBNE_DISABLE(); /* 手动接收 */

    spi_i2s_data_transmit(SPI0, 0x00); /* Shift-DR */
    spi_i2s_data_transmit(SPI2, 0xFF);
    tck_cycle_n(8);
    while (!SPI2_RBNE()) {
    }
    idcode[0] = spi_i2s_data_receive(SPI2);

    spi_i2s_data_transmit(SPI0, 0x00); /* Shift-DR */
    spi_i2s_data_transmit(SPI2, 0xFF);
    tck_cycle_n(8);
    while (!SPI2_RBNE()) {
    }
    idcode[1] = spi_i2s_data_receive(SPI2);

    spi_i2s_data_transmit(SPI0, 0x00); /* Shift-DR */
    spi_i2s_data_transmit(SPI2, 0xFF);
    tck_cycle_n(8);
    while (!SPI2_RBNE()) {
    }
    idcode[2] = spi_i2s_data_receive(SPI2);

    spi_i2s_data_transmit(SPI0, 1 << 7); /* Shift-DR -> Exit1-DR */
    spi_i2s_data_transmit(SPI2, 0xFF);
    tck_cycle_n(8);
    while (!SPI2_RBNE()) {
    }
    idcode[3] = spi_i2s_data_receive(SPI2);

    SPI2_INT_RBNE_ENABLE();

    /* Exit1-DR -> Run-Test/Idle **********************************************/

    spi_i2s_data_transmit(SPI0, B00000001); /* 1,0 */
    spi_i2s_data_transmit(SPI2, 0xFF);      /* 无所谓 */
    tck_cycle_n(8);                         /* 多发0也没关系 */
    SPI0_DISABLE();
    SPI2_DISABLE();

    return (idcode[0]) | (idcode[1] << 8) | (idcode[2] << 16) | (idcode[3] << 24);
}

/**
 * @brief 终止JTAG操作
 *
 * @param data
 */
void DAP_Port_JTAG_WriteAbort(uint32_t data) {
    /* Run-Test/Idle -> Shift-DR **********************************************/

    SPI0_ENABLE();
    SPI2_ENABLE();
    spi_i2s_data_transmit(SPI0, B001); /* 1,0,0 */
    spi_i2s_data_transmit(SPI2, 0xFF); /* 无所谓 */
    tck_cycle_n(3);                    /* 不能多，多了就错位了 */

    /* TDO <- device[0] <- ... <- device[n] <- TDI */

    /* 把数据从Bypass中推出来 ****************************************************/
    uint8_t bypass_num = dap_data.jtag_dev.index;
    if (bypass_num) {
        uint8_t i;

        while (bypass_num) {
            if (bypass_num > 8) {
                i = 8;
            } else {
                i = bypass_num;
            }
            bypass_num -= i;

            SPI0_INT_RBNE_ENABLE();
            spi_i2s_data_transmit(SPI0, 0x00); /* Shift-DR */
            spi_i2s_data_transmit(SPI2, 0xFF);
            tck_cycle_n(i);
        }
    }

    /* 发送req *********************************************************/

    spi_i2s_data_transmit(SPI0, 0x00); /* Shift-DR */
    spi_i2s_data_transmit(SPI2, 0x00); /* 三个0 */
    tck_cycle_n(3);

    /* 写操作 **************************************************************/

    spi_i2s_data_transmit(SPI0, 0x00);        /* Shift-DR */
    spi_i2s_data_transmit(SPI2, data & 0xFF); /*  */
    tck_cycle_n(8);

    spi_i2s_data_transmit(SPI0, 0x00);               /* Shift-DR */
    spi_i2s_data_transmit(SPI2, (data >> 8) & 0xFF); /*  */
    tck_cycle_n(8);

    spi_i2s_data_transmit(SPI0, 0x00);                /* Shift-DR */
    spi_i2s_data_transmit(SPI2, (data >> 16) & 0xFF); /*  */
    tck_cycle_n(8);

    uint8_t n = dap_data.jtag_dev.count - dap_data.jtag_dev.index - 1U;
    if (n > 0) {
        spi_i2s_data_transmit(SPI0, 0x00);                /* Shift-DR */
        spi_i2s_data_transmit(SPI2, (data >> 24) & 0xFF); /*  */
        tck_cycle_n(8);

        /* 把数据推到合适的位置 **********************************************/
        while (n > 0) {
            uint8_t i;
            if (n > 8) {
                i = 8;
            } else {
                i = n;
            }
            n -= i;

            if (n) {
                spi_i2s_data_transmit(SPI0, 0x00); /* Shift-DR */
            } else {
                spi_i2s_data_transmit(SPI0, 0x01 << (i - 1)); /* Shift-DR -> Exit1-DR */
            }
            spi_i2s_data_transmit(SPI2, 0xFF); /*  */
            tck_cycle_n(i);
        }
    } else {
        spi_i2s_data_transmit(SPI0, 0x01 << 7);           /* Shift-DR -> Exit1-DR */
        spi_i2s_data_transmit(SPI2, (data >> 24) & 0xFF); /*  */
        tck_cycle_n(8);
    }

    /* 退出 ********************************************************************/

    spi_i2s_data_transmit(SPI0, B00000001); /* 1,0 */
    spi_i2s_data_transmit(SPI2, 0xFF);      /* 无所谓 */
    tck_cycle_n(8);                         /* 多几个bit也没问题 */
    SPI0_DISABLE();
    SPI2_DISABLE();
}

/**
 * @brief 读取时间戳
 *
 * @return uint32_t
 */
uint32_t DAP_Port_GetTimeStamp(void) {
    /* T4 32位计数器 */
    return timer_counter_read(TIMER4);  //
}

/* 中断处理*********************************************************************/

void DMA1_Channel5_IRQHandler(void) {
    // DMA1_CH5_DISABLE(); /* 自己会关 */
    dma_flag_clear(DMA1, DMA_CH5, DMA_FLAG_FTF);
    // ULOG_DEBUG("spi tx done(dma1 ch5)\r\n");
}

void DMA1_Channel0_IRQHandler(void) {
    // DMA1_CH0_DISABLE(); /* 自己会关 */
    dma_flag_clear(DMA1, DMA_CH0, DMA_FLAG_FTF);
    // ULOG_DEBUG("spi rx dma done(dma1 ch0)\r\n");
}

void DMA0_Channel6_IRQHandler(void) {
    // dma_channel_disable(DMA0, DMA_CH6); /* 自己会关 */
    dma_flag_clear(DMA0, DMA_CH6, DMA_FLAG_FTF);
    // ULOG_DEBUG("timer3 dma done(dma0 ch6)\r\n");
}

void TIMER0_UP_TIMER9_IRQHandler(void) {
    T3_DISABLE();
    T0_UP_FLAG_CLEAR();
    // timer_interrupt_flag_clear(TIMER0, TIMER_INT_FLAG_UP);
    // timer_flag_clear(TIMER0, TIMER_FLAG_UP);
}

void SPI0_IRQHandler(void) {
    /* 自动接收数据，防止出现RXOVER影响数据接收 */
    spi0_data_buf[0] = spi_i2s_data_receive(SPI0);
}

void SPI2_IRQHandler(void) {
    /* 自动接收数据，防止出现RXOVER影响数据接收 */
    spi2_data_buf[0] = spi_i2s_data_receive(SPI2);
}
