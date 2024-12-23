#include "app_adc.h"
#include "app_crc.h"
#include "app_dap.h"
#include "app_debug.h"
#include "app_delay.h"
#include "app_led.h"
#include "app_usb.h"
#include "dap_port.h"
#include "gd32f4xx_libopt.h"
#include "ulog.h"

int main() {
    SystemCoreClockUpdate();
    nvic_priority_group_set(NVIC_PRIGROUP_PRE2_SUB2);

    rcu_periph_clock_enable(RCU_DMA0);
    rcu_periph_clock_enable(RCU_DMA1);
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_GPIOC);
    rcu_periph_clock_enable(RCU_GPIOD);
    rcu_periph_clock_enable(RCU_GPIOE);
    rcu_periph_clock_enable(RCU_SYSCFG);

    APP_Delay_Init();
    APP_Debug_UartInit(115200);
    APP_Debug_PrintSclk();
    APP_CRC_Init();
    APP_USB_Init();
    APP_ADC_Init();

    uint32_t r = APP_ADC_Read(0);
    ULOG_INFO("VREF: %d mV\r\n", r * 3300 * 2 / 4096);

    APP_Delay(100 * 1000);

    APP_DAP_Init();

    while (1) {
        APP_DAP_Handle();
    }
}
