#include "dap_port.h"

#include "gd32f4xx.h"
#include "gd32f4xx_libopt.h"
#include "ulog.h"

extern void APP_Delay(uint32_t delay_us);

/* SWD和JTAG的相关配置信息，定义在dap.c */
extern dap_data_t dap_data;

static uint32_t swj_timer0_cnt = 0xFFFFFFFF;
static volatile uint8_t spi_rdata_buf[4] = {0};

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

/*
    SWD模式
    TCK_CLT:
        -PA8:  T0_CH0,    AF1  OUT 主时钟输出
        -PC10: SPI2_SCK,  AF6  IN  SPI从机时钟
        -PB3:  SPI0_SCK,  AF5  IN  SPI从机时钟
    SCK_OEN:
        -PD13: T3_CH1,    AF2  OUT 时钟长度调整
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
#define TMS_OEN_RESET() (BIT_BAND_PERIPH(((GPIOD) + 0x18U), 12 + 16) = 0x00) /* 高位清零 */
#define TMS_OEN() (BIT_BAND_PERIPH(((GPIOD) + 0x14U), 12))

/* TMS传输方向控制 */
static void TmsOen(bool bit_enable) {
#if 1
    TMS_OEN() = bit_enable;
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
    gpio_bit_write(GPIOA, GPIO_PIN_6, bit_out);
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
 * @brief 初始化有关硬件
 *
 */
void DAP_Port_InitHardware(void) {
    rcu_periph_clock_enable(RCU_SPI0);
    rcu_periph_clock_enable(RCU_SPI2);
    rcu_periph_clock_enable(RCU_TIMER0);
    rcu_periph_clock_enable(RCU_TIMER3);
    rcu_periph_clock_enable(RCU_TIMER4);

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
    spi_disable(SPI2);

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
    // nvic_irq_enable(TIMER0_UP_TIMER9_IRQn, 0, 1);

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

    /* TCK_CTL */
    gpio_bit_set(GPIOA, GPIO_PIN_8);
    gpio_af_set(GPIOA, GPIO_AF_1, GPIO_PIN_8);                      /* T0_CH0 */
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_8); /* T0 */
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_8);

    gpio_af_set(GPIOC, GPIO_AF_6, GPIO_PIN_10); /* SPI0 */
    gpio_mode_set(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_10);

    gpio_af_set(GPIOB, GPIO_AF_5, GPIO_PIN_3); /* SPI2 */
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_3);

    /* SCK_OEN */

    /* TCK_OEN */
    TckOen(1);

    /* TCK_DCTL */
    TckPull(1); /* 默认电平高 */

    /* TMS_CTL TMS_ERT */
    TmsAfMode(1); /* 数据交给SPI控制 */

    /* TMS_OEN */
    TmsOen(1);       /* 输出 */
    TmsOenAfMode(0); /* TMS_OEN手动控制 */
}

/**
 * @brief 初始化JTAG端口
 *
 */
void DAP_Port_JTAG_Connect(void) {
    // TODO
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
    APP_Delay(delay_us);  //
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

    /* 每个周期的计数值 */
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

    /* 设置TIMER0的计数值 */
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
    uint32_t n_bit;
    uint32_t n_bytes;

    TmsOen(1);

    /* 8的整数倍周期 **********************************************************/

    while (count > 7) {
        if (count >= 256) {
            n_bit = 256; /* 单次最大值 */
        } else {
            n_bit = count - (count % 8); /* 剩余整数部分 */
        }
        count -= n_bit;
        // ULOG_DEBUG("SWJ use %d, remain %d\r\n", n_bit, count);

        n_bytes = n_bit / 8;
        dma_memory_address_config(DMA1, DMA_CH5, DMA_MEMORY_0, (uint32_t)value); /* TX */
        value += n_bytes;                                                        /* 数据指针后移 */

        dma_transfer_number_config(DMA1, DMA_CH5, n_bytes);       /* SPI0 */
        timer_repetition_value_config(TIMER0, (n_bytes * 8) - 1); /* 时钟周期数 */
        T0_SOFT_UPDATE();                                         /* 手动更新进去 */
        T0_UP_FLAG_CLEAR();
        timer_channel_output_pulse_value_config(TIMER3, TIMER_CH_1, 0xFFFF); /* TCK输出 */
        // ULOG_DEBUG("transfer %d bytes\r\n", n_bytes);

        DMA1_CH5_ENABLE();    /* SPI0 */
        SPI0_DMA_TX_ENABLE(); /* 发送 */
        SPI0_ENABLE();        /* 填充tx buffer */
        T3_ENABLE();          /* 从 */
        T0_ENABLE();          /* 主 */
        while (T0_IS_ENABLE()) {
        }

        T3_DISABLE();
        T0_UP_FLAG_CLEAR();
        SPI0_DMA_TX_DISABLE();
        SPI0_DISABLE();
    }

    if (count == 0) {
        return;
    }

    /* 不满8周期处理 **********************************************************/

    timer_repetition_value_config(TIMER0, 8 - 1); /* 时钟周期数 */
    T0_SOFT_UPDATE();                             /* 手动更新进去 */
    T0_UP_FLAG_CLEAR();

    uint16_t cnt = swj_timer0_cnt * count + swj_timer0_cnt / 4;
    timer_channel_output_pulse_value_config(TIMER3, TIMER_CH_1, cnt); /* TCK */
    // ULOG_DEBUG("transfer %d bytes\r\n", n_bytes);

    spi_i2s_data_transmit(SPI0, *value); /* SPI0 */
    SPI0_ENABLE();
    T3_ENABLE(); /* 从 */
    T0_ENABLE(); /* 主 */
    while (T0_IS_ENABLE()) {
    }

    T3_DISABLE();
    T0_UP_FLAG_CLEAR();
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
    /*
    Sequence Info:
        bit[5:0]: TCK周期数量
            0 = 64
            1 = 1
            ...
            63 = 63
        bit[6]: 保留
        bit[7]: 输入/输出
            0 = 输出
            1 = 输入
    */

    uint32_t count = info & 0x3F; /* 周期数 */
    if (count == 0) {
        count = 64;
    }

    bool dir_out = !(info & (1 << 7)); /* 是否输出 */
    TmsOen(dir_out);

    timer_repetition_value_config(TIMER0, 8 - 1); /* 时钟周期数 */
    T0_SOFT_UPDATE();                             /* 手动更新进去 */
    T0_UP_FLAG_CLEAR();

    SPI0_ENABLE();
    SPI0_INT_RBNE_DISABLE();

    while (count > 0) {
        uint32_t n_bit;
        if (count > 7) {
            n_bit = 8;
        } else {
            n_bit = count;
        }
        count -= n_bit;

        if (n_bit == 8) {
            timer_channel_output_pulse_value_config(TIMER3, TIMER_CH_1, 0xFFFFFFFF);
        } else {
            uint16_t cnt = swj_timer0_cnt * n_bit + swj_timer0_cnt / 4;
            timer_channel_output_pulse_value_config(TIMER3, TIMER_CH_1, cnt);
        }

        if (dir_out) {
            spi_i2s_data_transmit(SPI0, *request); /* SPI0 */
            request++;
        }

        T3_ENABLE(); /* 从 */
        T0_ENABLE(); /* 主 */
        while (T0_IS_ENABLE()) {
        }

        T3_DISABLE();
        T0_UP_FLAG_CLEAR();

        if (!dir_out) {
            while (!SPI0_RBNE()) { /* 等待SPI数据 */
            }
            *response = spi_i2s_data_receive(SPI0); /* SPI0 */
            response++;
        }
    }

    SPI0_INT_RBNE_ENABLE();
    SPI0_DISABLE();
    TmsOen(1);
}

/**
 * @brief SWD传输
 *
 * @param request
 * @param response
 * @return uint32_t
 */
uint8_t DAP_Port_SWD_Transfer(uint32_t request, uint8_t *response) {
    uint32_t cnt; /* T3计数值 */

    /* 计算奇偶校验，奇1偶0 */
    uint8_t req_raw = request & 0x0F;
    uint8_t req_parity = parity_mapping_table[req_raw] & 0x01;
    uint8_t req = 0x81 | ((req_raw) << 1) | (req_parity << 5); /* req 8bit */
    uint8_t read_n_write = (request & DAP_TRANSFER_RnW) ? 0x01 : 0x00;

    /* request ****************************************************************/

    TmsOen(1);
    timer_channel_output_pulse_value_config(TIMER3, TIMER_CH_1, 0xFFFFFFFF);
    timer_repetition_value_config(TIMER0, 8 - 1); /* 时钟周期数 */
    T0_SOFT_UPDATE();                             /* 手动更新进去 */
    T0_UP_FLAG_CLEAR();
    SPI0_ENABLE();
    SPI0_INT_RBNE_ENABLE();           /* 抛弃接收的数据 */
    spi_i2s_data_transmit(SPI0, req); /* SPI0 */

    T3_ENABLE(); /* 从 */
    T0_ENABLE(); /* 主 */
    while (T0_IS_ENABLE()) {
    }

    T3_DISABLE();
    T0_UP_FLAG_CLEAR();

    /* ACK ********************************************************************/

    TmsOen(0);
    SPI0_DISABLE();

    uint8_t trn_num = (read_n_write == 1U) ? 1U : 2U;                /* W-R-R 1trn cycle / W-R-W 2trn cycle */
    uint8_t ack_cycle = 3U + dap_data.swd_conf.turnaround * trn_num; /* 3 ACK + trn */
    if (ack_cycle > 8U) {
        if (ack_cycle > 16U) {
            /* 不可能出现，trn周期最多只有4，4+3+4=11 */
            ULOG_ERROR("ACK over length\r\n");
            return DAP_TRANSFER_ERROR; /* 不支持 */
        }
        spi_i2s_data_frame_format_config(SPI0, SPI_FRAMESIZE_16BIT);
    }

    cnt = swj_timer0_cnt * ack_cycle + swj_timer0_cnt / 4;
    timer_channel_output_pulse_value_config(TIMER3, TIMER_CH_1, cnt);
    timer_repetition_value_config(TIMER0, 8 - 1); /* 时钟周期数 */
    T0_SOFT_UPDATE();                             /* 手动更新进去 */
    T0_UP_FLAG_CLEAR();

    SPI0_ENABLE();
    SPI0_INT_RBNE_DISABLE(); /* 手动接收 */

    T3_ENABLE(); /* 从 */
    T0_ENABLE(); /* 主 */
    while (T0_IS_ENABLE()) {
    }

    T3_DISABLE();
    T0_UP_FLAG_CLEAR();

    while (!SPI0_RBNE()) { /* 等待SPI数据清空 */
    }

    uint8_t ack = 0x07 & (spi_i2s_data_receive(SPI0) >> dap_data.swd_conf.turnaround); /* ACK[0:2] */

    SPI0_INT_RBNE_ENABLE(); /* 放开中断 */
    SPI0_DISABLE();

    if (ack_cycle > 8U) {
        /* 恢复8bit */
        spi_i2s_data_frame_format_config(SPI0, SPI_FRAMESIZE_8BIT);
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
            /* read */
            TmsOen(0);

            while (SPI0_RBNE()) {
            }
            SPI0_INT_RBNE_DISABLE();                                                         /* 关中断 */
            dma_memory_address_config(DMA1, DMA_CH0, DMA_MEMORY_0, (uint32_t)spi_rdata_buf); /* RX */
            dma_transfer_number_config(DMA1, DMA_CH0, 4);                                    /* SPI0 */
            DMA1_CH0_ENABLE();
            SPI0_DMA_RX_ENABLE(); /* RX */
        } else {
            /* write */
            TmsOen(1);

            dma_memory_address_config(DMA1, DMA_CH5, DMA_MEMORY_0, (uint32_t)response);
            dma_transfer_number_config(DMA1, DMA_CH5, 4); /* SPI0 */
            DMA1_CH5_ENABLE();
            SPI0_DMA_TX_ENABLE(); /* TX */
        }

        SPI0_ENABLE();
        timer_channel_output_pulse_value_config(TIMER3, TIMER_CH_1, 0xFFFF);
        timer_repetition_value_config(TIMER0, (4 * 8) - 1); /* 32bit */
        T0_SOFT_UPDATE();                                   /* 手动更新进去 */
        T0_UP_FLAG_CLEAR();

        T3_ENABLE();
        T0_ENABLE(); /* 数据阶段 */
        while (T0_IS_ENABLE()) {
        }

        T3_DISABLE();
        T0_UP_FLAG_CLEAR();

        if (read_n_write) {
            while (DMA1_CH0_IS_ENABLE()) {
            }

            SPI0_DMA_RX_DISABLE();
            response[0] = spi_rdata_buf[0];
            response[1] = spi_rdata_buf[1];
            response[2] = spi_rdata_buf[2];
            response[3] = spi_rdata_buf[3];
        } else {
            SPI0_DMA_TX_DISABLE();
        }

        SPI0_INT_RBNE_ENABLE();
        SPI0_DISABLE();

        /* 1bit校验位 和 可能的转换位 */
        cnt = swj_timer0_cnt * (dap_data.swd_conf.turnaround * read_n_write + 1U) + swj_timer0_cnt / 4;
        timer_channel_output_pulse_value_config(TIMER3, TIMER_CH_1, cnt);
        timer_repetition_value_config(TIMER0, 8 - 1); /* 时钟周期数 */
        T0_SOFT_UPDATE();                             /* 手动更新进去 */
        T0_UP_FLAG_CLEAR();

        SPI0_ENABLE();
        SPI0_INT_RBNE_DISABLE();
        spi_i2s_data_transmit(SPI0, wdata_parity); /* 读取时无效 */

        T3_ENABLE();
        T0_ENABLE();
        while (T0_IS_ENABLE()) {
        }

        T3_DISABLE();
        T0_UP_FLAG_CLEAR();

        while (!SPI0_RBNE()) { /* 等待SPI数据 */
        }

        if (read_n_write) {
            /* 计算校验值 */
            uint8_t rdata_parity = 0x01 & spi_i2s_data_receive(SPI0);
            uint8_t p = 0x01 & (parity_mapping_table[spi_rdata_buf[0]] + parity_mapping_table[spi_rdata_buf[1]]  //
                                + parity_mapping_table[spi_rdata_buf[2]] + parity_mapping_table[spi_rdata_buf[3]]);

            if (p != rdata_parity) {
                ack = DAP_TRANSFER_ERROR; /* 校验值错误 */
                ULOG_ERROR("data parity error\r\n");
            }
        }

        SPI0_INT_RBNE_ENABLE();
        TmsOen(1); /* 恢复控制权 */

        /* 写指令后需要添加空闲周期 */
        if ((!read_n_write) && (dap_data.transfer.idle_cycles > 0)) {
            TmsAfMode(0); /* 手动控制 */
            TmsWrite(0);

            uint8_t idle_count = dap_data.transfer.idle_cycles;
            while (idle_count > 0) {
                uint8_t idle_bit;
                if (idle_count > 8) {
                    idle_bit = 8;
                    cnt = 0xFFFF;
                } else {
                    idle_bit = idle_count;
                    cnt = swj_timer0_cnt * idle_bit + swj_timer0_cnt / 4;
                }
                idle_count -= idle_bit;

                timer_channel_output_pulse_value_config(TIMER3, TIMER_CH_1, cnt);
                timer_repetition_value_config(TIMER0, 8 - 1); /* 时钟周期数 */
                T0_SOFT_UPDATE();                             /* 手动更新进去 */
                T0_UP_FLAG_CLEAR();

                T3_ENABLE();
                T0_ENABLE();
                while (T0_IS_ENABLE()) {
                }

                T3_DISABLE();
                T0_UP_FLAG_CLEAR();
            }

            TmsWrite(1);
            TmsAfMode(1); /* SPI控制 */
        }
    } else {
        SPI0_DISABLE();
        if (read_n_write) {
            /* 读操作失败添加一个trn周期，重新掌握数据线控制权 */
            cnt = swj_timer0_cnt * dap_data.swd_conf.turnaround + swj_timer0_cnt / 4;
            timer_channel_output_pulse_value_config(TIMER3, TIMER_CH_1, cnt);
            timer_repetition_value_config(TIMER0, 8 - 1); /* 时钟周期数 */
            T0_SOFT_UPDATE();                             /* 手动更新进去 */
            T0_UP_FLAG_CLEAR();

            T3_ENABLE();
            T0_ENABLE();
            while (T0_IS_ENABLE()) {
            }

            T3_DISABLE();
            T0_UP_FLAG_CLEAR();
        }
    }
    SPI0_DISABLE();

    TmsOen(1);
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
    // TODO
}

/**
 * @brief JTAG传输
 *
 * @param request
 * @param response
 * @return uint32_t
 */
uint8_t DAP_Port_JTAG_Transfer(uint32_t request, uint8_t *response) {
    // TODO
    return 0;
}

/**
 * @brief 转换JTAG IR状态
 *
 * @param ir
 */
void DAP_Port_JTAG_IR(uint32_t ir) {
    // TODO
}

/**
 * @brief JTAG读取IDCODE
 *
 * @return uint32_t
 */
uint32_t DAP_Port_JTAG_ReadIDCode(void) {
    // TODO
    return 0;
}

/**
 * @brief 终止JTAG操作
 *
 * @param data
 */
void DAP_Port_JTAG_WriteAbort(uint32_t data) {
    // TODO
}

/**
 * @brief 读取时间戳
 *
 * @return uint32_t
 */
uint32_t DAP_Port_GetTimeStamp(void) { return timer_counter_read(TIMER4); /* T4 32位计数器 */ }

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
    timer_interrupt_flag_clear(TIMER0, TIMER_INT_FLAG_UP);
    timer_flag_clear(TIMER0, TIMER_FLAG_UP);
    // T3_DISABLE(); /* 不要在中断里进行这个操作，有难以预测的BUG */
}

void SPI0_IRQHandler(void) {
    /* 自动接收数据，防止出现RXOVER影响数据接收 */
    spi_rdata_buf[0] = spi_i2s_data_receive(SPI0);
}
