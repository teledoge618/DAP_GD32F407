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

uint8_t sdo[16] = {
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
};
uint8_t sdi[16] = {0};

extern dap_data_t dap_data;

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

    APP_Debug_UartInit(115200);
    APP_Debug_PrintSclk();
    APP_CRC_Init();
    APP_USB_Init();
    APP_ADC_Init();

    APP_DAP_Init();

#if 0
    DAP_Port_SWJ_Disconnect();
    DAP_Port_JTAG_Connect();
    DAP_Port_SWD_Sequence(51, sdo, sdi);
    sdo[0] = 0;
    DAP_Port_SWD_Sequence(8, sdo, sdi);

    dap_data.jtag_dev.count = 3;                              /*  */
    dap_data.jtag_dev.index = 1;                              /*  */
    dap_data.jtag_dev.ir_before[dap_data.jtag_dev.index] = 3; /* 前 */
    dap_data.jtag_dev.ir_after[dap_data.jtag_dev.index] = 3;  /* 后 */
    dap_data.jtag_dev.ir_length[dap_data.jtag_dev.index] = 3; /*  */

    // DAP_Port_JTAG_IR(0x02);
    uint32_t test_data1 = 0x12345678;
    DAP_Port_JTAG_Transfer(0x01, (uint8_t*)&test_data1);

#endif

#if 0
    DAP_Port_SWD_Connect();
    DAP_Port_SWJ_Sequence(7, sdo);
    DAP_Port_SWJ_Sequence(9, sdo);
#endif

    while (1) {
        APP_DAP_Handle();
    }
}
