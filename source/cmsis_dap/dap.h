#ifndef __DAP_H__
#define __DAP_H__

#include "dap_def.h"
#include "dap_port.h"
#include "stdint.h"
#include "stdio.h"

// DAP Data structure
typedef struct {
    uint8_t debug_port;       // 记录打开了哪个调试口
    uint32_t timestamp;       // 时间戳更新值
    uint32_t frequency;       // 频率
    struct {                  // Transfer Configuration
        uint8_t idle_cycles;  // Idle cycles after transfer
        uint8_t padding[3];
        uint16_t retry_count;  // Number of retries after WAIT response
        uint16_t match_retry;  // Number of retries if read value does not match
        uint32_t match_mask;   // Match Mask
    } transfer;
#if (DAP_SWD_SUPPORT != 0)
    struct {                 // SWD Configuration
        uint8_t turnaround;  // Turnaround period MAX4
        uint8_t data_phase;  // 是否无条件生成数据阶段
    } swd_conf;
#endif
#if (DAP_JTAG_SUPPORT != 0)
    struct {            // JTAG Device Chain
        uint8_t count;  // Number of devices
        uint8_t index;  // Device index (device at TDO has index 0)
#if (DAP_JTAG_DEV_CNT != 0)
        uint8_t ir_length[DAP_JTAG_DEV_CNT];   // IR Length in bits
        uint16_t ir_before[DAP_JTAG_DEV_CNT];  // Bits before IR
        uint16_t ir_after[DAP_JTAG_DEV_CNT];   // Bits after IR
#endif
    } jtag_dev;
#endif
} dap_data_t;

void DAP_Setup(void);
uint32_t DAP_ExecuteCommand(const uint8_t *request, uint8_t *response);

#endif  // !__DAP_CONFIG_H__