#ifndef __DAP_CONFIG_H__
#define __DAP_CONFIG_H__

// clang-format off

/* SWD可用性(1=可用,0=不可用) */
#define DAP_SWD_SUPPORT         1

/* JTAG可用性(1=可用,0=不可用) */
#define DAP_JTAG_SUPPORT        1

/* JTAG链上的最大设备数，1-255，数字越大占用内存越多 */
#define DAP_JTAG_DEV_CNT        4

/* 默认功能是SWD还是JTAG(1=SWD,0=JTAG) */
#define DAP_DEFAULT_PORT        1

/* 默认TCK时钟速度(Hz) */
#define DAP_DEFAULT_SWJ_CLOCK   10000000U

/* 最大数据包长度(byte)，HID 1024, WINUSB 512 */
#define DAP_PACKET_SIZE         512

/* 包缓存个数 */
#define DAP_PACKET_COUNT        4U

/* SWO可用性(1=可用,0=不可用) */
#define SWO_UART_SUPPORT        0

/* SWO最大波特率(bps) */
#define SWO_UART_MAX_BAUDRATE   10000000U

/* SWO曼彻斯特编码模式可用性(1=可用,0=不可用) */
#define SWO_MANCHESTER_SUPPORT  0

/* SWO缓冲区长度(byte) */
#define SWO_BUFFER_SIZE         8192U

/* SWO Streaming Trace */
#define SWO_STREAM              0

/* 时间戳计时器频率(Hz)，0表示不支持 */
#define TIMESTAMP_CLOCK         1000000U

/* CDC UART CMSIS-DAP内置的协议，非普通串口 */
#define CDC_UART_SUPPORT        0

/* 厂商名 */
#define CMSIS_DAP_VENDOR_NAME   "ARM"

/* 产品名 */
#define CMSIS_DAP_PRODUCT_NAME  "CMSIS-DAP v2"

/* CMSIS-DAP固件版本 */
#define CMSIS_DAP_FW_VER        "2.1.0"

/* SPI可用性(1=可用,0=不可用) */
#define DAP_SPI_SUPPORT         0

/* I2C可用性(1=可用,0=不可用) */
#define DAP_I2C_SUPPORT         0

// clang-format on

#endif /* __DAP_CONFIG_H__ */
