#ifndef __DAP_H__
#define __DAP_H__

#include "dap_def.h"
#include "dap_port.h"
#include "stdint.h"
#include "stdio.h"

// DAP Data structure
typedef struct {
    uint8_t debug_port;       // 记录打开了哪个调试口
    uint32_t timestamp;       // 存储时间戳的更新值
    uint32_t frequency;       // 时钟频率
    struct {                  // Transfer Configuration
        uint8_t idle_cycles;  // 传输之后的空周期
        uint8_t padding[3];
        uint16_t retry_count;  // 收到WAIT时的重试次数
        uint16_t match_retry;  // 不匹配时的重试次数
        uint32_t match_mask;   // 匹配掩码
    } transfer;
#if (DAP_SWD_SUPPORT != 0)
    struct {                 // SWD Configuration
        uint8_t turnaround;  // SWDIO方向改变时的空周期数 MAX=4
        uint8_t data_phase;  // 是否无条件生成数据阶段
    } swd_conf;
#endif
#if (DAP_JTAG_SUPPORT != 0)
    struct {            // JTAG Device Chain
        uint8_t count;  // JTAG链上的设备数
        uint8_t index;  // 当前设备在链上的索引（最接近TDO的是索引0）
#if (DAP_JTAG_DEV_CNT != 0)
        uint8_t ir_length[DAP_JTAG_DEV_CNT];   // IR寄存器bit长度
        uint16_t ir_before[DAP_JTAG_DEV_CNT];  // 此IR前的bit长度
        uint16_t ir_after[DAP_JTAG_DEV_CNT];   // 此IR后的bit长度
#endif
    } jtag_dev;
#endif

#if (DAP_SPI_SUPPORT != 0)
    struct {
        /* SPI操作一般是cmd + addr + data[] */
        uint8_t cmd_len;     // 指令长度，byte倍数
        uint8_t addr_len;    // 地址长度，byte倍数
        uint8_t format_msb;  // 高位优先
    } spi_conf;
#endif

#if (DAP_I2C_SUPPORT != 0)
    struct {
        /* I2C操作一般是 dev_addr + cmd + addr + data[] */
        uint8_t dev_addr_10bit;  // 8位地址还是10bit地址
        uint8_t addr_len;        // 地址长度，byte倍数
        uint8_t cmd_len;         // 指令长度，byte倍数
    } i2c_conf;
#endif
} dap_data_t;

void DAP_Setup(void);
uint32_t DAP_ExecuteCommand(const uint8_t *request, uint8_t *response);

#endif  // !__DAP_CONFIG_H__