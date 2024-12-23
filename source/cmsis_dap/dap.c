#include "dap.h"

/* 字符串长度最大值，数据包最短64，头最长0x7F+len+0x00+ID = 4 */
#define DAP_STR_LEN_MAX (60)

dap_data_t dap_data; /* DAP配置信息 */

volatile uint8_t dap_transfer_abort; /* DAP终止所有操作 */

static uint32_t DAP_Info(const uint8_t *request, uint8_t *response);
static uint32_t DAP_HostStatus(const uint8_t *request, uint8_t *response);
static uint32_t DAP_Connect(const uint8_t *request, uint8_t *response);
static uint32_t DAP_Disconnect(uint8_t *response);
static uint32_t DAP_Delay(const uint8_t *request, uint8_t *response);
static uint32_t DAP_ResetTarget(uint8_t *response);
static uint32_t DAP_SWJ_Pins(const uint8_t *request, uint8_t *response);
static uint32_t DAP_SWJ_Clock(const uint8_t *request, uint8_t *response);
static uint32_t DAP_SWJ_Sequence(const uint8_t *request, uint8_t *response);
static uint32_t DAP_SWD_Configure(const uint8_t *request, uint8_t *response);
static uint32_t DAP_SWD_Sequence(const uint8_t *request, uint8_t *response);
static uint32_t DAP_JTAG_Sequence(const uint8_t *request, uint8_t *response);
static uint32_t DAP_JTAG_Configure(const uint8_t *request, uint8_t *response);
static uint32_t DAP_JTAG_IDCode(const uint8_t *request, uint8_t *response);
static uint32_t DAP_TransferConfigure(const uint8_t *request,
                                      uint8_t *response);
static uint32_t DAP_Transfer(const uint8_t *request, uint8_t *response);
static uint32_t DAP_TransferBlock(const uint8_t *request, uint8_t *response);
static uint32_t DAP_WriteAbort(const uint8_t *request, uint8_t *response);

static uint8_t DAP_Info_GetVendorString(char *str);
static uint8_t DAP_Info_GetProductString(char *str);
static uint8_t DAP_Info_GetSerNumString(char *str);
static uint8_t DAP_Info_GetFwVerString(char *str);
static uint8_t DAP_Info_GetTargetDeviceVendorString(char *str);
static uint8_t DAP_Info_GetTargetDeviceNameString(char *str);
static uint8_t DAP_Info_GetTargetBoardVendorString(char *str);
static uint8_t DAP_Info_GetTargetBoardNameString(char *str);
static uint8_t DAP_Info_GetProductFirmwareVersionString(char *str);
static uint8_t DAP_Info_GetCappabilities(uint8_t *info);
static uint8_t DAP_Info_GetTestDomainTimerFreq(uint8_t *info);
static uint8_t DAP_Info_GetUartRxBufferSize(uint8_t *info);
static uint8_t DAP_Info_GetUartTxBufferSize(uint8_t *info);
static uint8_t DAP_Info_GetSWOBufferSize(uint8_t *info);
static uint8_t DAP_Info_GetPacketSize(uint8_t *info);
static uint8_t DAP_Info_GetCountSize(uint8_t *info);

#if (DAP_SWD_SUPPORT != 0)
static uint32_t DAP_SWD_Transfer(const uint8_t *request, uint8_t *response);
static uint32_t DAP_SWD_TransferBlock(const uint8_t *request,
                                      uint8_t *response);
static uint32_t DAP_SWD_WriteAbort(const uint8_t *request, uint8_t *response);
#endif

#if (DAP_JTAG_SUPPORT != 0)
static uint32_t DAP_JTAG_Transfer(const uint8_t *request, uint8_t *response);
static uint32_t DAP_JTAG_TransferBlock(const uint8_t *request,
                                       uint8_t *response);
static uint32_t DAP_JTAG_WriteAbort(const uint8_t *request, uint8_t *response);
#endif
static uint32_t DAP_Dummy_Transfer(const uint8_t *request, uint8_t *response);

static uint32_t DAP_ProcessCommand(const uint8_t *request, uint8_t *response);
static uint8_t DAP_ReturnString(char *dst, const char *src);

/**
 * @brief DAP初始化
 *
 */
void DAP_Setup(void) {
    // Default settings
    dap_data.debug_port = 0U;
    dap_data.frequency = DAP_DEFAULT_SWJ_CLOCK;
    dap_data.transfer.idle_cycles = 0U;
    dap_data.transfer.retry_count = 100U;
    dap_data.transfer.match_retry = 0U;
    dap_data.transfer.match_mask = 0x00000000U;
#if (DAP_SWD_SUPPORT != 0)
    dap_data.swd_conf.turnaround = 1U;
    dap_data.swd_conf.data_phase = 0U;
#endif
#if (DAP_JTAG_SUPPORT != 0)
    dap_data.jtag_dev.count = 0U;
#endif

    DAP_Port_InitHardware();
    DAP_Port_SWJ_Disconnect();
    DAP_Port_SWJ_SetClock(dap_data.frequency);
}

/**
 * @brief 执行dap指令
 *
 * @param request   主机发来的请求数据
 * @param response  返回给主机的数据
 * @return uint32_t 已使用的数据长度，高半字收，低半字发
 */
uint32_t DAP_ExecuteCommand(const uint8_t *request, uint8_t *response) {
    uint32_t cnt;  // 数据包内包含的指令条数
    uint32_t num;  // 数据数量
    uint32_t n;

    /* in:  0x7F + num + ...*/
    /* out: 0x7F + num + ... */

    /* 连续处理多条指令 */
    if (request[0] == ID_DAP_ExecuteCommands) {
        response[0] = request[0];  // 复制ID，返回主机
        response[1] = request[1];  // 复制指令数
        cnt = request[1];
        /* 跳到实际数据部分 */
        response += 2U;
        request += 2U;
        num = ((2U << 16) | 2U);
        /* 逐条处理 */
        while (cnt--) {
            n = DAP_ProcessCommand(request,
                                   response);  // 同样的返回格式

            num += n;  // 高低位各自相加，没有进位问题
            request += (uint16_t)(n >> 16);
            response += (uint16_t)n;
        }
        return (num);
    }

    return DAP_ProcessCommand(request, response);
}

/**
 * @brief 处理单条指令
 *
 * @param request   主机发来的请求数据
 * @param response  返回给主机的数据
 * @return uint32_t 已使用的数据长度，高半字收，低半字发
 */
static uint32_t DAP_ProcessCommand(const uint8_t *request, uint8_t *response) {
    uint32_t num = 0;

#if 0
    /* 优先处理自定义指令 */
    if ((*request >= ID_DAP_Vendor0) && (*request <= ID_DAP_Vendor31)) {
        return DAP_ProcessVendorCommand(request, response);
    }

    if ((*request >= ID_DAP_VendorExFirst) &&
        (*request <= ID_DAP_VendorExLast)) {
        return DAP_ProcessVendorCommandEx(request, response);
    }
#endif

    /* 复制ID */
    response[0] = request[0];
    uint8_t id = request[0];

    /* 根据ID进行处理，传给函数的是实际的数据部分，不包括命令ID */
    switch (id) {
        case ID_DAP_Info:
            num = DAP_Info(&request[1], &response[1]);
            break;
        case ID_DAP_HostStatus:
            num = DAP_HostStatus(&request[1], &response[1]);
            break;
        case ID_DAP_Connect:
            num = DAP_Connect(&request[1], &response[1]);
            break;
        case ID_DAP_Disconnect:
            num = DAP_Disconnect(&response[1]);
            break;
        case ID_DAP_Delay:
            num = DAP_Delay(&request[1], &response[1]);
            break;
        case ID_DAP_ResetTarget:
            num = DAP_ResetTarget(&response[1]);
            break;

        case ID_DAP_SWJ_Pins:
            num = DAP_SWJ_Pins(&request[1], &response[1]);
            break;
        case ID_DAP_SWJ_Clock:
            num = DAP_SWJ_Clock(&request[1], &response[1]);
            break;
        case ID_DAP_SWJ_Sequence:
            num = DAP_SWJ_Sequence(&request[1], &response[1]);
            break;

        case ID_DAP_SWD_Configure:
            num = DAP_SWD_Configure(&request[1], &response[1]);
            break;
        case ID_DAP_SWD_Sequence:
            num = DAP_SWD_Sequence(&request[1], &response[1]);
            break;

        case ID_DAP_JTAG_Sequence:
            num = DAP_JTAG_Sequence(&request[1], &response[1]);
            break;
        case ID_DAP_JTAG_Configure:
            num = DAP_JTAG_Configure(&request[1], &response[1]);
            break;
        case ID_DAP_JTAG_IDCODE:
            num = DAP_JTAG_IDCode(&request[1], &response[1]);
            break;

        case ID_DAP_TransferConfigure:
            num = DAP_TransferConfigure(&request[1], &response[1]);
            break;
        case ID_DAP_Transfer:
            num = DAP_Transfer(&request[1], &response[1]);
            break;
        case ID_DAP_TransferBlock:
            num = DAP_TransferBlock(&request[1], &response[1]);
            break;

        case ID_DAP_WriteABORT:
            num = DAP_WriteAbort(&request[1], &response[1]);
            break;

#if ((SWO_UART_SUPPORT != 0) || (SWO_MANCHESTER_SUPPORT != 0))
        case ID_DAP_SWO_Transport:
            num = SWO_Transport(&request[1], &response[1]);
            break;
        case ID_DAP_SWO_Mode:
            num = SWO_Mode(&request[1], &response[1]);
            break;
        case ID_DAP_SWO_Baudrate:
            num = SWO_Baudrate(&request[1], &response[1]);
            break;
        case ID_DAP_SWO_Control:
            num = SWO_Control(&request[1], &response[1]);
            break;
        case ID_DAP_SWO_Status:
            num = SWO_Status(&response[1]);
            break;
        case ID_DAP_SWO_ExtendedStatus:
            num = SWO_ExtendedStatus(&request[1], &response[1]);
            break;
        case ID_DAP_SWO_Data:
            num = SWO_Data(&request[1], &response[1]);
            break;
#endif

#if (DAP_UART != 0)
        case ID_DAP_UART_Transport:
            num = UART_Transport(&request[1], &response[1]);
            break;
        case ID_DAP_UART_Configure:
            num = UART_Configure(&request[1], &response[1]);
            break;
        case ID_DAP_UART_Control:
            num = UART_Control(&request[1], &response[1]);
            break;
        case ID_DAP_UART_Status:
            num = UART_Status(&response[1]);
            break;
        case ID_DAP_UART_Transfer:
            num = UART_Transfer(&request[1], &response[1]);
            break;
#endif

#if (DAP_SPI_SUPPORT != 0)
        case ID_SPI_Configure:

            break;
#endif

#if (DAP_I2C_SUPPORT != 0)
        case ID_I2C_Configure:

            break;
#endif

        default:
            response[0] = ID_DAP_Invalid;  // 无法处理的指令
    }

    return (((1U << 16) | 1U) + num);
}

/**
 * @brief 获取DAP信息
 *
 * @param request
 * @param response
 * @return uint32_t
 */
static uint32_t DAP_Info(const uint8_t *request, uint8_t *response) {
    /*
    | BYTE | BYTE |
    > 0x00 | ID   |
    |******|******|

    | BYTE | BYTE | BYTE |
    < 0x00 | Len  | Info |
    |******|******|++++++|
    */

    uint8_t length = 0U;
    uint8_t info_id = request[0];
    uint8_t *data_buffer = &response[1];

    switch (info_id) {
        case DAP_ID_VENDOR:
            length = DAP_Info_GetVendorString((char *)data_buffer);
            break;
        case DAP_ID_PRODUCT:
            length = DAP_Info_GetProductString((char *)data_buffer);
            break;
        case DAP_ID_SER_NUM:
            length = DAP_Info_GetSerNumString((char *)data_buffer);
            break;
        case DAP_ID_DAP_FW_VER:
            length = DAP_Info_GetFwVerString((char *)data_buffer);
            break;
        case DAP_ID_DEVICE_VENDOR:
            length = DAP_Info_GetTargetDeviceVendorString((char *)data_buffer);
            break;
        case DAP_ID_DEVICE_NAME:
            length = DAP_Info_GetTargetDeviceNameString((char *)data_buffer);
            break;
        case DAP_ID_BOARD_VENDOR:
            length = DAP_Info_GetTargetBoardVendorString((char *)data_buffer);
            break;
        case DAP_ID_BOARD_NAME:
            length = DAP_Info_GetTargetBoardNameString((char *)data_buffer);
            break;
        case DAP_ID_PRODUCT_FW_VER:
            length =
                DAP_Info_GetProductFirmwareVersionString((char *)data_buffer);
            break;
        case DAP_ID_CAPABILITIES:
            length = DAP_Info_GetCappabilities(data_buffer);
            break;
        case DAP_ID_TIMESTAMP_CLOCK:
            length = DAP_Info_GetTestDomainTimerFreq(data_buffer);
            break;
        case DAP_ID_UART_RX_BUFFER_SIZE:
            length = DAP_Info_GetUartRxBufferSize(data_buffer);
            break;
        case DAP_ID_UART_TX_BUFFER_SIZE:
            length = DAP_Info_GetUartTxBufferSize(data_buffer);
            break;
        case DAP_ID_SWO_BUFFER_SIZE:
            length = DAP_Info_GetSWOBufferSize(data_buffer);
            break;
        case DAP_ID_PACKET_SIZE:
            length = DAP_Info_GetPacketSize(data_buffer);
            break;
        case DAP_ID_PACKET_COUNT:
            length = DAP_Info_GetCountSize(data_buffer);
            break;
        default:
            break;
    }

    response[0] = length;

    return (((1U << 16) | 1U) + length);
}

/**
 * @brief 复制字符串并返回长度
 *
 * @param dst
 * @param src 源字符串
 * @return uint8_t 字符串长度
 */
static uint8_t DAP_ReturnString(char *dst, const char *src) {
    uint8_t i = 0;

    while (i < DAP_STR_LEN_MAX) {
        if (src[i] == '\0') {
            dst[i] = '\0';
            break;
        } else {
            dst[i] = src[i];
        }

        i++;
    }

    return (i + 1);
}

/**
 * @brief 获取厂商名
 *
 * @param str
 * @return uint8_t
 */
static uint8_t DAP_Info_GetVendorString(char *str) {
#if defined(CMSIS_DAP_VENDOR_NAME)
    return DAP_ReturnString(str, CMSIS_DAP_VENDOR_NAME);
#else
    (void)str;
    return (0U);
#endif
}

/**
 * @brief 获取产品名
 *
 * @param str
 * @return uint8_t
 */
static uint8_t DAP_Info_GetProductString(char *str) {
#if defined(CMSIS_DAP_PRODUCT_NAME)
    return DAP_ReturnString(str, CMSIS_DAP_PRODUCT_NAME);
#else
    (void)str;
    return (0U);
#endif
}

/**
 * @brief 获取序列号
 *
 * @param str
 * @return uint8_t
 */
static uint8_t DAP_Info_GetSerNumString(char *str) {
    extern char dap_uinque_serial_number[16];
    return DAP_ReturnString(str, dap_uinque_serial_number);
}

/**
 * @brief 获取版本号
 *
 * @param str
 * @return uint8_t
 */
static uint8_t DAP_Info_GetFwVerString(char *str) {
#if defined(CMSIS_DAP_FW_VER)
    return DAP_ReturnString(str, CMSIS_DAP_FW_VER);
#else
    (void)str;
    return 0;
#endif
}

/**
 * @brief 获取目标设备厂商名
 *
 * @param str
 * @return uint8_t
 */
static uint8_t DAP_Info_GetTargetDeviceVendorString(char *str) {
    (void)str;
    return 0;
}

/**
 * @brief 获取目标设备厂商名
 *
 * @param str
 * @return uint8_t
 */
static uint8_t DAP_Info_GetTargetDeviceNameString(char *str) {
    (void)str;
    return 0;
}

/**
 * @brief 获取目标设备产品名
 *
 * @param str
 * @return uint8_t
 */
static uint8_t DAP_Info_GetTargetBoardVendorString(char *str) {
    (void)str;
    return 0;
}

/**
 * @brief 获取目标设备名
 *
 * @param str
 * @return uint8_t
 */
static uint8_t DAP_Info_GetTargetBoardNameString(char *str) {
    (void)str;
    return 0;
}

/**
 * @brief 获取目标设备版本
 *
 * @param str
 * @return uint8_t
 */
static uint8_t DAP_Info_GetProductFirmwareVersionString(char *str) {
    (void)str;
    return 0;
}

/**
 * @brief 获取DAP支持的功能
 *
 * @return uint8_t
 */
static uint8_t DAP_Info_GetCappabilities(uint8_t *info) {
    /*
    bit0: DAP_SWD_SUPPORT
    bit1: DAP_JTAG_SUPPORT
    bit2: SWO_UART_SUPPORT
    bit3: SWO_MANCHESTER_SUPPORT
    bit4: Atomic Commands
    bit5: TIMESTAMP_CLOCK!=0
    bit6: SWO_STREAM
    bit7:
    */
    info[0] = ((DAP_SWD_SUPPORT != 0) ? (1U << 0) : 0U) |         //
              ((DAP_JTAG_SUPPORT != 0) ? (1U << 1) : 0U) |        //
              ((SWO_UART_SUPPORT != 0) ? (1U << 2) : 0U) |        //
              ((SWO_MANCHESTER_SUPPORT != 0) ? (1U << 3) : 0U) |  //
              (1U << 4) |                                         //
              ((TIMESTAMP_CLOCK != 0U) ? (1U << 5) : 0U) |        //
              ((SWO_STREAM != 0U) ? (1U << 6) : 0U);

    return 1U;
}

/**
 * @brief 获取计时器频率
 *
 * @param info
 * @return uint8_t
 */
static uint8_t DAP_Info_GetTestDomainTimerFreq(uint8_t *info) {
#if (TIMESTAMP_CLOCK != 0U)
    info[0] = (uint8_t)(TIMESTAMP_CLOCK >> 0);
    info[1] = (uint8_t)(TIMESTAMP_CLOCK >> 8);
    info[2] = (uint8_t)(TIMESTAMP_CLOCK >> 16);
    info[3] = (uint8_t)(TIMESTAMP_CLOCK >> 24);
#else
    info[0] = 0;
    info[1] = 0;
    info[2] = 0;
    info[3] = 0;
#endif
    return 4U;
}

/**
 * @brief 获取UART接收缓冲区大小
 *
 * @param info
 * @return uint8_t
 */
static uint8_t DAP_Info_GetUartRxBufferSize(uint8_t *info) {
#if ((SWO_UART_SUPPORT != 0) || (SWO_MANCHESTER_SUPPORT != 0))
    info[0] = (uint8_t)(SWO_BUFFER_SIZE >> 0);
    info[1] = (uint8_t)(SWO_BUFFER_SIZE >> 8);
    info[2] = (uint8_t)(SWO_BUFFER_SIZE >> 16);
    info[3] = (uint8_t)(SWO_BUFFER_SIZE >> 24);
#else
    info[0] = 0;
    info[1] = 0;
    info[2] = 0;
    info[3] = 0;
#endif
    return 4U;
}

/**
 * @brief 获取UART发送缓冲区大小
 *
 * @param info
 * @return uint8_t
 */
static uint8_t DAP_Info_GetUartTxBufferSize(uint8_t *info) {
#if ((SWO_UART_SUPPORT != 0) || (SWO_MANCHESTER_SUPPORT != 0))
    info[0] = (uint8_t)(SWO_BUFFER_SIZE >> 0);
    info[1] = (uint8_t)(SWO_BUFFER_SIZE >> 8);
    info[2] = (uint8_t)(SWO_BUFFER_SIZE >> 16);
    info[3] = (uint8_t)(SWO_BUFFER_SIZE >> 24);
#else
    info[0] = 0;
    info[1] = 0;
    info[2] = 0;
    info[3] = 0;
#endif
    return 4U;
}

/**
 * @brief 获取计时器频率
 *
 * @param info
 * @return uint8_t
 */
static uint8_t DAP_Info_GetSWOBufferSize(uint8_t *info) {
#if ((SWO_UART_SUPPORT != 0) || (SWO_MANCHESTER_SUPPORT != 0))
    info[0] = (uint8_t)(SWO_BUFFER_SIZE >> 0);
    info[1] = (uint8_t)(SWO_BUFFER_SIZE >> 8);
    info[2] = (uint8_t)(SWO_BUFFER_SIZE >> 16);
    info[3] = (uint8_t)(SWO_BUFFER_SIZE >> 24);
#else
    info[0] = 0;
    info[1] = 0;
    info[2] = 0;
    info[3] = 0;
#endif
    return 0U;
}

/**
 * @brief 获取包长度
 *
 * @param info
 * @return uint8_t
 */
static uint8_t DAP_Info_GetPacketSize(uint8_t *info) {
    info[0] = (uint8_t)(DAP_PACKET_SIZE >> 0);
    info[1] = (uint8_t)(DAP_PACKET_SIZE >> 8);
    return 2U;
}

/**
 * @brief 获取包缓冲区深度
 *
 * @param info
 * @return uint8_t
 */
static uint8_t DAP_Info_GetCountSize(uint8_t *info) {
    info[0] = DAP_PACKET_COUNT;
    return 1U;
}

/**
 * @brief 设置指示灯，表明状态
 *
 * @param request
 * @param response
 * @return uint32_t
 */
static uint32_t DAP_HostStatus(const uint8_t *request, uint8_t *response) {
    /*
    | BYTE | BYTE **| BYTE **|
    > 0x01 | Type   | Status |
    |******|********|********|

    | BYTE | BYTE **|
    < 0x01 | 0x00   |
    |******|********|
    */

    switch (request[0]) {
        case DAP_DEBUGGER_CONNECTED:
            DAP_Port_SetConnectedStatus(request[1]);
            break;
        case DAP_TARGET_RUNNING:
            DAP_Port_SetRunningStatus(request[1]);
            break;
        default:
            response[0] = DAP_ERROR;
            return ((2U << 16) | 1U);
    }

    response[0] = DAP_OK;
    return ((2U << 16) | 1U);
}

/**
 * @brief 初始化端口
 *
 * @param request
 * @param response
 * @return uint32_t
 */
static uint32_t DAP_Connect(const uint8_t *request, uint8_t *response) {
    /*
    | BYTE | BYTE |
    > 0x02 | Port |
    |******|******|

    | BYTE | BYTE |
    < 0x02 | Port |
    |******|******|
    */

    /*
    Port:
        0 = default
        1 = SWD
        2 = JTAG
    */

    uint32_t port;
    if (request[0] == DAP_PORT_AUTODETECT) { /* 使用默认端口 */
#if (DAP_DEFAULT_PORT == 1)
        port = DAP_PORT_SWD;
#else
        port = DAP_PORT_JTAG;
#endif
    } else {
        port = request[0];
    }

    switch (port) {
#if (DAP_SWD_SUPPORT != 0)
        case DAP_PORT_SWD:
            dap_data.debug_port = DAP_PORT_SWD;
            DAP_Port_SWD_Connect();
            break;
#endif
#if (DAP_JTAG_SUPPORT != 0)
        case DAP_PORT_JTAG:
            dap_data.debug_port = DAP_PORT_JTAG;
            DAP_Port_JTAG_Connect();
            break;
#endif
        default:
            port = DAP_PORT_DISABLED;  // 失败
            break;
    }

    response[0] = (uint8_t)port;
    return ((1U << 16) | 1U);
}

/**
 * @brief 断开端口
 *
 * @param response
 * @return uint32_t
 */
static uint32_t DAP_Disconnect(uint8_t *response) {
    /*
    | BYTE |
    > 0x03 |
    |******|

    | BYTE | BYTE   |
    < 0x03 | Status |
    |******|********|
    */

    dap_data.debug_port = DAP_PORT_DISABLED;
    DAP_Port_SWJ_Disconnect();

    response[0] = DAP_OK;
    return (1U);
}

/**
 * @brief 阻塞延时(微秒)
 *
 * @param request
 * @param response
 * @return uint32_t
 */
static uint32_t DAP_Delay(const uint8_t *request, uint8_t *response) {
    /*
    | BYTE | SHORT |
    > 0x09 | Delay |
    |******|*******|

    | BYTE | BYTE   |
    < 0x09 | Status |
    |******|********|
    */

    uint32_t delay = (uint32_t)(request[0] << 0) | (uint32_t)(request[1] << 8);
    DAP_Port_Delay(delay);

    response[0] = DAP_OK;
    return ((2U << 16) | 1U);
}

/**
 * @brief 复位目标芯片
 *
 * @param response
 * @return uint32_t
 */
static uint32_t DAP_ResetTarget(uint8_t *response) {
    /*
    | BYTE |
    > 0x0A |
    |******|

    | BYTE | BYTE   | BYTE    |
    < 0x0A | Status | Execute |
    |******|********|*********|
    */

    response[0] = DAP_OK;
    response[1] = DAP_Port_ResetTarget();

    return (2U);
}

/**
 * @brief 设置引脚
 *
 * @param request
 * @param response
 * @return uint32_t
 */
static uint32_t DAP_SWJ_Pins(const uint8_t *request, uint8_t *response) {
    /*
    | BYTE | BYTE ******| BYTE ******| Word ****|
    > 0x10 | Pin Output | Pin Select | Pin Wait |
    |******|************|************|**********|

    | BYTE | BYTE *****|
    < 0x10 | Pin Input |
    |******|***********|
    */

#if ((DAP_SWD_SUPPORT != 0) || (DAP_JTAG_SUPPORT != 0))
    uint32_t delay = (uint32_t)(response[2] << 0) |   //
                     (uint32_t)(response[3] << 8) |   //
                     (uint32_t)(response[4] << 16) |  //
                     (uint32_t)(response[5] << 24);
    uint8_t output_data = request[0];
    uint8_t pin_mask = request[1];
    DAP_Port_SetPins(pin_mask, output_data);

    DAP_Port_Delay(delay);

    response[0] = DAP_Port_GetPins();

#else
    response[0] = 0U;
#endif

    return ((6U << 16) | 1U);
}

/**
 * @brief 设置输出时钟频率
 *
 * @param request
 * @param response
 * @return uint32_t
 */
static uint32_t DAP_SWJ_Clock(const uint8_t *request, uint8_t *response) {
    /*
    | BYTE | WORD *|
    > 0x11 | Clock |
    |******|*******|

    | BYTE | BYTE **|
    < 0x11 | Status |
    |******|********|
    */

#if ((DAP_SWD_SUPPORT != 0) || (DAP_JTAG_SUPPORT != 0))
    uint32_t clock = (uint32_t)(request[0] << 0) |   //
                     (uint32_t)(request[1] << 8) |   //
                     (uint32_t)(request[2] << 16) |  //
                     (uint32_t)(request[3] << 24);

    if (clock == 0U) {
        response[0] = DAP_ERROR;
        return ((4U << 16) | 1U);
    }

    DAP_Port_SWJ_SetClock(clock);

    response[0] = DAP_OK;
#else
    response[0] = DAP_ERROR;
#endif

    return ((4U << 16) | 1U);
}

/**
 * @brief 输出一串序列
 *
 * @param request
 * @param response
 * @return uint32_t
 */
static uint32_t DAP_SWJ_Sequence(const uint8_t *request, uint8_t *response) {
    /*
    | BYTE | BYTE **************| BYTE *************|
    > 0x12 | Sequence Bit Count | Sequence Bit Data |
    |******|********************|+++++++++++++++++++|

    | BYTE | BYTE **|
    < 0x12 | Status |
    |******|********|
    */

    uint32_t count = request[0];

    /* 0表示256 */
    if (count == 0U) {
        count = 256U;
    }

#if ((DAP_SWD_SUPPORT != 0) || (DAP_JTAG_SUPPORT != 0))
    DAP_Port_SWJ_Sequence(count, (uint8_t *)&request[1]);
    response[0] = DAP_OK;
#else
    response[0] = DAP_ERROR;
#endif

    /* 不满1字节的数据按1字节占位 */
    count = (count + 7U) >> 3;

    return (((count + 1U) << 16) | 1U);
}

/**
 * @brief 设置SWD参数
 *
 * @param request
 * @param response
 * @return uint32_t
 */
static uint32_t DAP_SWD_Configure(const uint8_t *request, uint8_t *response) {
    /*
    | BYTE | BYTE *********|
    > 0x13 | Configuration |
    |******|***************|

    | BYTE | BYTE **|
    < 0x13 | Status |
    |******|********|
    */

    /*
    Configuration:
        bit[1:0]: 读写切换时，Trn周期数量
            00 = 1(default)
            01 = 2
            10 = 3
            11 = 4
        bit[2]: SWD收的到ACK是WAIT时，是否继续输出DATA
            0 = no(default)
            1 = yes
    */

#if (DAP_SWD_SUPPORT != 0)
    uint8_t value = request[0];
    dap_data.swd_conf.turnaround = (value & 0x03U) + 1U;
    dap_data.swd_conf.data_phase = (value & 0x04U) ? 1U : 0U;

    response[0] = DAP_OK;
#else
    response[0] = DAP_ERROR;
#endif

    return ((1U << 16) | 1U);
}

/**
 * @brief 输出一定长的SWD序列，读/写
 *
 * @param request
 * @param response
 * @return uint32_t
 */
static uint32_t DAP_SWD_Sequence(const uint8_t *request, uint8_t *response) {
    /*
    | BYTE | BYTE **********| BYTE *********| BYTE ******|
    > 0x1D | Sequence Count | Sequence Info | SWDIO Data |
    |******|****************|

    | BYTE | BYTE **| BYTE ******|
    < 0x1D | Status | SWDIO Data |
    |******|********|++++++++++++|
    */

    /*
    Sequence Info:
        bit[5:0]: TCK周期数量
            0 = 1
            ...
            64 = 0
        bit[6]: 保留
        bit[7]: 输入/输出
            0 = 输出
            1 = 输入
    */

    /* 全部 */
    uint32_t sequence_num;
    uint32_t request_count;
    uint32_t response_count;
    /* 每个序列 */
    uint8_t info;
    uint8_t count;

    sequence_num = request[0];  // 序列个数
#if (DAP_SWD_SUPPORT != 0)
    response[0] = DAP_OK;
#else
    response[0] = DAP_ERROR;
#endif

    /* 跳到序列数据部分 */
    request_count = 1U;
    response_count = 1U;

    while (sequence_num--) {
        info = request[request_count];    // 当前序列的配置
        count = info & SWD_SEQUENCE_CLK;  // 当前序列里TCK的数量

        if (count == 0U) {
            count = 64U;
        }

#if (DAP_SWD_SUPPORT != 0)
        DAP_Port_SWD_Sequence(info, &request[request_count + 1],
                              &response[response_count]);
#endif
        /* 计算占用字节数量 */
        count = (count + 7U) >> 3;

        /* 是否捕获DIO */
        if (info & SWD_SEQUENCE_DIN) {
            /* 有捕获数据，回复字节数 */
#if (DAP_SWD_SUPPORT != 0)
            response_count += count;
#endif
            request_count++;  // INFO
        } else {
            /* 没有捕获数据，消耗字节数 */
            request_count += (count + 1);
        }
    }

    return (((request_count + 1) << 16) | (response_count + 1));
}

/**
 * @brief 输出一定长度的JTAG序列，可读/写，TMS电平固定
 *
 * @param request
 * @param response
 * @return uint32_t
 */
static uint32_t DAP_JTAG_Sequence(const uint8_t *request, uint8_t *response) {
    /* in:  0x14 + Count  + {Info + Data[]} */
    /* out: 0x14 + Status + Data[](input mode only) */

    /*
    Info.bit[5:0]: TCK周期数量
        0 = 1
        ...
        64 = 0
    Info.bit[6]: TMS，固定值
    Info.bit[7]: 输入/输出
        0 = 输出
        1 = 输入
    */

    /* 全部 */
    uint32_t sequence_num;
    uint32_t request_count;
    uint32_t response_count;
    /* 每个序列 */
    uint8_t info;
    uint8_t count;

    sequence_num = request[0];  // 序列个数
#if (DAP_JTAG_SUPPORT != 0)
    response[0] = DAP_OK;
#else
    response[0] = DAP_ERROR;
#endif

    /* 跳到序列数据部分 */
    request_count = 1U;
    response_count = 1U;

    while (sequence_num--) {
        info = request[request_count];    // 当前序列的配置
        count = info & SWD_SEQUENCE_CLK;  // 当前序列里TCK的数量

        if (count == 0U) {
            count = 64U;
        }

#if (DAP_JTAG_SUPPORT != 0)
        DAP_Port_JTAG_Sequence(info, &request[request_count + 1],
                               &response[response_count]);
#endif
        /* 计算占用字节数量 */
        count = (count + 7U) >> 3;

        /* 是否捕获DIO */
        if (info & JTAG_SEQUENCE_TDO) {
            /* 有捕获数据，回复字节数 */
#if (DAP_JTAG_SUPPORT != 0)
            response_count += count;
#endif
            request_count++;  // INFO
        } else {
            /* 没有捕获数据，消耗字节数 */
            request_count += (count + 1);
        }
    }

    return (((request_count + 1) << 16) | (response_count + 1));
}

/**
 * @brief 配置JTAG参数
 *
 * @param request
 * @param response
 * @return uint32_t
 */
static uint32_t DAP_JTAG_Configure(const uint8_t *request, uint8_t *response) {
    /* in:  0x15 + Count  + IR Length */
    /* out: 0x15 + Status */

    uint32_t count;
#if (DAP_JTAG_SUPPORT != 0)
    uint32_t length;
    uint32_t bits;
    uint32_t n;

    count = *request++;
    dap_data.jtag_dev.count = (uint8_t)count;

    bits = 0U;
    for (n = 0U; n < count; n++) {
        length = *request++;
        dap_data.jtag_dev.ir_length[n] = (uint8_t)length;
        dap_data.jtag_dev.ir_before[n] = (uint16_t)bits;
        bits += length;
    }
    for (n = 0U; n < count; n++) {
        bits -= dap_data.jtag_dev.ir_length[n];
        dap_data.jtag_dev.ir_after[n] = (uint16_t)bits;
    }

    *response = DAP_OK;
#else
    count = *request;
    *response = DAP_ERROR;
#endif

    return (((count + 1U) << 16) | 1U);
}

/**
 * @brief 获取JTAG链上设备的IDCODE
 *
 * @param request
 * @param response
 * @return uint32_t
 */
static uint32_t DAP_JTAG_IDCode(const uint8_t *request, uint8_t *response) {
    /* in:  0x16 + JTAG Index */
    /* out: 0x16 + Status + (uint32_t)IDCODE*/

#if (DAP_JTAG_SUPPORT != 0)
    uint32_t data;
    /* 没打开JTAG口 */
    if (dap_data.debug_port != DAP_PORT_JTAG) {
        goto id_error;
    }

    // Device index (JTAP TAP)
    dap_data.jtag_dev.index = *request;
    if (dap_data.jtag_dev.index >= dap_data.jtag_dev.count) {
        goto id_error;
    }

    // Select JTAG chain
    DAP_Port_JTAG_IR(JTAG_IDCODE);

    // Read IDCODE register
    data = DAP_Port_JTAG_ReadIDCode();

    // Store Data
    *(response + 0) = DAP_OK;
    *(response + 1) = (uint8_t)(data >> 0);
    *(response + 2) = (uint8_t)(data >> 8);
    *(response + 3) = (uint8_t)(data >> 16);
    *(response + 4) = (uint8_t)(data >> 24);

    return ((1U << 16) | 5U);

id_error:
#endif
    *response = DAP_ERROR;
    return ((1U << 16) | 1U);
}

/**
 * @brief 配置传输参数
 *
 * @param request
 * @param response
 * @return uint32_t
 */
static uint32_t DAP_TransferConfigure(const uint8_t *request,
                                      uint8_t *response) {
    /*
    | BYTE | BYTE ******** SHORT *****| SHORT ******|
    > 0x04 | Idle Cycles | WAIT Retry | Match Retry |
    |******|*************|************|*************|

    | BYTE | BYTE **|
    < 0x04 | Status |
    |******|********|
    */

    dap_data.transfer.idle_cycles = request[0];
    dap_data.transfer.retry_count =
        (uint16_t)request[1] | (uint16_t)(request[2] << 8);
    dap_data.transfer.match_retry =
        (uint16_t)request[3] | (uint16_t)(request[4] << 8);

    *response = DAP_OK;
    return ((5U << 16) | 1U);
}

/**
 * @brief 读/写寄存器
 *
 * @param request
 * @param response
 * @return uint32_t
 */
static uint32_t DAP_Transfer(const uint8_t *request, uint8_t *response) {
    /*
    | BYTE | BYTE *****| BYTE **********| BYTE *************| WORD *********|
    > 0x05 | DAP Index | Transfer Count | Transfer Request  | Transfer Data |
    |******|***********|****************|+++++++++++++++++++++++++++++++++++|

    DAP Index:
        从0开始的JTAG链上的设备位置。
    Transfer Count:
        传输数量，每个传输都包含一个Request和Data。
    Transfer Request:
        bit[0]: APnDP
        bit[1]: RnW
        bit[2]: A2
        bit[3]: A3
        bit[4]: 匹配模式，检验读出数据
        bit[5]: 匹配掩码
        bit[6]:
        bit[7]: 是否包含时间戳
    Transfer Data:
        写寄存器时，将这个值写入寄存器。
        匹配掩码时，这个值是掩码。
        匹配时，这个值是匹配值。
        其他不包含数据

    | BYTE  | BYTE **********| BYTE *************| WORD ********| WORD ********|
     < 0x05 | Transfer Count | Transfer Response | TD_TimeStamp |Transfer Data |
    |******|****************|*******************|

    Transfer Count:
        数量
    Transfer Response:
        bit[2:0]:
            1: OK
            2: WAIT
            3: FAULT
            7: NO_ACK
        bit[3]:
            1: SWD ERROR
        bit[4]:
            1: 不匹配（匹配模式）
    TD_TimeStamp:
        时间戳
    Transfer Data:
        仅读操作有返回值
    */

    uint32_t num;

    switch (dap_data.debug_port) {
#if (DAP_SWD_SUPPORT != 0)
        case DAP_PORT_SWD:
            num = DAP_SWD_Transfer(request, response);
            break;
#endif
#if (DAP_JTAG_SUPPORT != 0)
        case DAP_PORT_JTAG:
            num = DAP_JTAG_Transfer(request, response);
            break;
#endif
        default:
            num = DAP_Dummy_Transfer(request, response);
            break;
    }

    return (num);
}

#if (DAP_SWD_SUPPORT != 0)
/**
 * @brief SWD读写寄存器
 *
 * @param request
 * @param response
 * @return uint32_t
 */
static uint32_t DAP_SWD_Transfer(const uint8_t *request, uint8_t *response) {
    const uint8_t *request_head;
    uint32_t request_count;
    uint32_t request_value;
    uint8_t *response_head;
    uint32_t response_count;
    uint32_t response_value;
    uint32_t post_read;
    uint32_t check_write;
    uint32_t match_value;
    uint32_t match_retry;
    uint32_t retry;
    volatile uint32_t data;
#if (TIMESTAMP_CLOCK != 0U)
    uint32_t timestamp;
#endif

    request_head = request;

    response_count = 0U;
    response_value = 0U;
    response_head = response;
    response += 2;

    dap_transfer_abort = 0U;

    post_read = 0U;
    check_write = 0U;

    request++;  // Ignore DAP index

    request_count = *request++;

    while (request_count != 0) {
        request_count--;
        request_value = *request++;
        if ((request_value & DAP_TRANSFER_RnW) != 0U) {
            // Read register
            if (post_read) {
                // Read was posted before
                retry = dap_data.transfer.retry_count;
                if ((request_value &
                     (DAP_TRANSFER_APnDP | DAP_TRANSFER_MATCH_VALUE)) ==
                    DAP_TRANSFER_APnDP) {
                    // Read previous AP data and post next AP read
                    do {
                        response_value = DAP_Port_SWD_Transfer(
                            request_value, (uint8_t *)&data);
                    } while ((response_value == DAP_TRANSFER_WAIT) && retry-- &&
                             !dap_transfer_abort);
                } else {
                    // Read previous AP data
                    do {
                        response_value = DAP_Port_SWD_Transfer(
                            DP_RDBUFF | DAP_TRANSFER_RnW, (uint8_t *)&data);
                    } while ((response_value == DAP_TRANSFER_WAIT) && retry-- &&
                             !dap_transfer_abort);
                    post_read = 0U;
                }
                if (response_value != DAP_TRANSFER_OK) {
                    break;
                }
                // Store previous AP data
                *response++ = (uint8_t)data;
                *response++ = (uint8_t)(data >> 8);
                *response++ = (uint8_t)(data >> 16);
                *response++ = (uint8_t)(data >> 24);
#if (TIMESTAMP_CLOCK != 0U)
                if (post_read) {
                    // Store Timestamp of next AP read
                    if ((request_value & DAP_TRANSFER_TIMESTAMP) != 0U) {
                        timestamp = dap_data.timestamp;
                        *response++ = (uint8_t)timestamp;
                        *response++ = (uint8_t)(timestamp >> 8);
                        *response++ = (uint8_t)(timestamp >> 16);
                        *response++ = (uint8_t)(timestamp >> 24);
                    }
                }
#endif
            }
            if ((request_value & DAP_TRANSFER_MATCH_VALUE) != 0U) {
                // Read with value match
                match_value = (uint32_t)(*(request + 0) << 0) |
                              (uint32_t)(*(request + 1) << 8) |
                              (uint32_t)(*(request + 2) << 16) |
                              (uint32_t)(*(request + 3) << 24);
                request += 4;
                match_retry = dap_data.transfer.match_retry;
                if ((request_value & DAP_TRANSFER_APnDP) != 0U) {
                    // Post AP read
                    retry = dap_data.transfer.retry_count;
                    do {
                        response_value =
                            DAP_Port_SWD_Transfer(request_value, NULL);
                    } while ((response_value == DAP_TRANSFER_WAIT) && retry-- &&
                             !dap_transfer_abort);
                    if (response_value != DAP_TRANSFER_OK) {
                        break;
                    }
                }
                do {
                    // Read register until its value matches or retry counter
                    // expires
                    retry = dap_data.transfer.retry_count;
                    do {
                        response_value = DAP_Port_SWD_Transfer(
                            request_value, (uint8_t *)&data);
                    } while ((response_value == DAP_TRANSFER_WAIT) && retry-- &&
                             !dap_transfer_abort);
                    if (response_value != DAP_TRANSFER_OK) {
                        break;
                    }
                } while (
                    ((data & dap_data.transfer.match_mask) != match_value) &&
                    match_retry-- && !dap_transfer_abort);
                if ((data & dap_data.transfer.match_mask) != match_value) {
                    response_value |= DAP_TRANSFER_MISMATCH;
                }
                if (response_value != DAP_TRANSFER_OK) {
                    break;
                }
            } else {
                // Normal read
                retry = dap_data.transfer.retry_count;
                if ((request_value & DAP_TRANSFER_APnDP) != 0U) {
                    // Read AP register
                    if (post_read == 0U) {
                        // Post AP read
                        do {
                            response_value =
                                DAP_Port_SWD_Transfer(request_value, NULL);
                        } while ((response_value == DAP_TRANSFER_WAIT) &&
                                 retry-- && !dap_transfer_abort);
                        if (response_value != DAP_TRANSFER_OK) {
                            break;
                        }
#if (TIMESTAMP_CLOCK != 0U)
                        // Store Timestamp
                        if ((request_value & DAP_TRANSFER_TIMESTAMP) != 0U) {
                            timestamp = dap_data.timestamp;
                            *response++ = (uint8_t)timestamp;
                            *response++ = (uint8_t)(timestamp >> 8);
                            *response++ = (uint8_t)(timestamp >> 16);
                            *response++ = (uint8_t)(timestamp >> 24);
                        }
#endif
                        post_read = 1U;
                    }
                } else {
                    // Read DP register
                    do {
                        response_value = DAP_Port_SWD_Transfer(
                            request_value, (uint8_t *)&data);
                    } while ((response_value == DAP_TRANSFER_WAIT) && retry-- &&
                             !dap_transfer_abort);
                    if (response_value != DAP_TRANSFER_OK) {
                        break;
                    }
#if (TIMESTAMP_CLOCK != 0U)
                    // Store Timestamp
                    if ((request_value & DAP_TRANSFER_TIMESTAMP) != 0U) {
                        timestamp = dap_data.timestamp;
                        *response++ = (uint8_t)timestamp;
                        *response++ = (uint8_t)(timestamp >> 8);
                        *response++ = (uint8_t)(timestamp >> 16);
                        *response++ = (uint8_t)(timestamp >> 24);
                    }
#endif
                    // Store data
                    *response++ = (uint8_t)data;
                    *response++ = (uint8_t)(data >> 8);
                    *response++ = (uint8_t)(data >> 16);
                    *response++ = (uint8_t)(data >> 24);
                }
            }
            check_write = 0U;
        } else {
            // Write register
            if (post_read) {
                // Read previous data
                retry = dap_data.transfer.retry_count;
                do {
                    response_value = DAP_Port_SWD_Transfer(
                        DP_RDBUFF | DAP_TRANSFER_RnW, (uint8_t *)&data);
                } while ((response_value == DAP_TRANSFER_WAIT) && retry-- &&
                         !dap_transfer_abort);
                if (response_value != DAP_TRANSFER_OK) {
                    break;
                }
                // Store previous data
                *response++ = (uint8_t)data;
                *response++ = (uint8_t)(data >> 8);
                *response++ = (uint8_t)(data >> 16);
                *response++ = (uint8_t)(data >> 24);
                post_read = 0U;
            }
            // Load data
            data = (uint32_t)(*(request + 0) << 0) |
                   (uint32_t)(*(request + 1) << 8) |
                   (uint32_t)(*(request + 2) << 16) |
                   (uint32_t)(*(request + 3) << 24);
            request += 4;
            if ((request_value & DAP_TRANSFER_MATCH_MASK) != 0U) {
                // Write match mask
                dap_data.transfer.match_mask = data;
                response_value = DAP_TRANSFER_OK;
            } else {
                // Write DP/AP register
                retry = dap_data.transfer.retry_count;
                do {
                    response_value =
                        DAP_Port_SWD_Transfer(request_value, (uint8_t *)&data);
                } while ((response_value == DAP_TRANSFER_WAIT) && retry-- &&
                         !dap_transfer_abort);
                if (response_value != DAP_TRANSFER_OK) {
                    break;
                }
#if (TIMESTAMP_CLOCK != 0U)
                // Store Timestamp
                if ((request_value & DAP_TRANSFER_TIMESTAMP) != 0U) {
                    timestamp = dap_data.timestamp;
                    *response++ = (uint8_t)timestamp;
                    *response++ = (uint8_t)(timestamp >> 8);
                    *response++ = (uint8_t)(timestamp >> 16);
                    *response++ = (uint8_t)(timestamp >> 24);
                }
#endif
                check_write = 1U;
            }
        }
        response_count++;
        if (dap_transfer_abort) {
            break;
        }
    }

    while (request_count != 0) {
        // Process canceled requests
        request_count--;
        request_value = *request++;
        if ((request_value & DAP_TRANSFER_RnW) != 0U) {
            // Read register
            if ((request_value & DAP_TRANSFER_MATCH_VALUE) != 0U) {
                // Read with value match
                request += 4;
            }
        } else {
            // Write register
            request += 4;
        }
    }

    if (response_value == DAP_TRANSFER_OK) {
        if (post_read) {
            // Read previous data
            retry = dap_data.transfer.retry_count;
            do {
                response_value = DAP_Port_SWD_Transfer(
                    DP_RDBUFF | DAP_TRANSFER_RnW, (uint8_t *)&data);
            } while ((response_value == DAP_TRANSFER_WAIT) && retry-- &&
                     !dap_transfer_abort);
            if (response_value != DAP_TRANSFER_OK) {
                goto end;
            }
            // Store previous data
            *response++ = (uint8_t)data;
            *response++ = (uint8_t)(data >> 8);
            *response++ = (uint8_t)(data >> 16);
            *response++ = (uint8_t)(data >> 24);
        } else if (check_write) {
            // Check last write
            retry = dap_data.transfer.retry_count;
            do {
                response_value =
                    DAP_Port_SWD_Transfer(DP_RDBUFF | DAP_TRANSFER_RnW, NULL);
            } while ((response_value == DAP_TRANSFER_WAIT) && retry-- &&
                     !dap_transfer_abort);
        }
    }

end:
    *(response_head + 0) = (uint8_t)response_count;
    *(response_head + 1) = (uint8_t)response_value;

    return (((uint32_t)(request - request_head) << 16) |
            (uint32_t)(response - response_head));
}
#endif

#if (DAP_JTAG_SUPPORT != 0)
/**
 * @brief JTAG读写寄存器
 *
 * @param request
 * @param response
 * @return uint32_t
 */
static uint32_t DAP_JTAG_Transfer(const uint8_t *request, uint8_t *response) {
    const uint8_t *request_head;
    uint32_t request_count;
    uint32_t request_value;
    uint32_t request_ir;
    uint8_t *response_head;
    uint32_t response_count;
    uint32_t response_value;
    uint32_t post_read;
    uint32_t match_value;
    uint32_t match_retry;
    uint32_t retry;
    uint32_t data;
    uint32_t ir;
#if (TIMESTAMP_CLOCK != 0U)
    uint32_t timestamp;
#endif

    request_head = request;

    response_count = 0U;
    response_value = 0U;
    response_head = response;
    response += 2;

    dap_transfer_abort = 0U;

    ir = 0U;
    post_read = 0U;

    // Device index (JTAP TAP)
    dap_data.jtag_dev.index = *request++;
    if (dap_data.jtag_dev.index >= dap_data.jtag_dev.count) {
        goto end;
    }

    request_count = *request++;

    for (; request_count != 0U; request_count--) {
        request_value = *request++;
        request_ir =
            (request_value & DAP_TRANSFER_APnDP) ? JTAG_APACC : JTAG_DPACC;
        if ((request_value & DAP_TRANSFER_RnW) != 0U) {
            // Read register
            if (post_read) {
                // Read was posted before
                retry = dap_data.transfer.retry_count;
                if ((ir == request_ir) &&
                    ((request_value & DAP_TRANSFER_MATCH_VALUE) == 0U)) {
                    // Read previous data and post next read
                    do {
                        response_value = DAP_Port_JTAG_Transfer(
                            request_value, (uint8_t *)&data);
                    } while ((response_value == DAP_TRANSFER_WAIT) && retry-- &&
                             !dap_transfer_abort);
                } else {
                    // Select JTAG chain
                    if (ir != JTAG_DPACC) {
                        ir = JTAG_DPACC;
                        DAP_Port_JTAG_IR(ir);
                    }
                    // Read previous data
                    do {
                        response_value = DAP_Port_JTAG_Transfer(
                            DP_RDBUFF | DAP_TRANSFER_RnW, (uint8_t *)&data);
                    } while ((response_value == DAP_TRANSFER_WAIT) && retry-- &&
                             !dap_transfer_abort);
                    post_read = 0U;
                }
                if (response_value != DAP_TRANSFER_OK) {
                    break;
                }
                // Store previous data
                *response++ = (uint8_t)data;
                *response++ = (uint8_t)(data >> 8);
                *response++ = (uint8_t)(data >> 16);
                *response++ = (uint8_t)(data >> 24);
#if (TIMESTAMP_CLOCK != 0U)
                if (post_read) {
                    // Store Timestamp of next AP read
                    if ((request_value & DAP_TRANSFER_TIMESTAMP) != 0U) {
                        timestamp = dap_data.timestamp;
                        *response++ = (uint8_t)timestamp;
                        *response++ = (uint8_t)(timestamp >> 8);
                        *response++ = (uint8_t)(timestamp >> 16);
                        *response++ = (uint8_t)(timestamp >> 24);
                    }
                }
#endif
            }
            if ((request_value & DAP_TRANSFER_MATCH_VALUE) != 0U) {
                // Read with value match
                match_value = (uint32_t)(*(request + 0) << 0) |
                              (uint32_t)(*(request + 1) << 8) |
                              (uint32_t)(*(request + 2) << 16) |
                              (uint32_t)(*(request + 3) << 24);
                request += 4;
                match_retry = dap_data.transfer.match_retry;
                // Select JTAG chain
                if (ir != request_ir) {
                    ir = request_ir;
                    DAP_Port_JTAG_IR(ir);
                }
                // Post DP/AP read
                retry = dap_data.transfer.retry_count;
                do {
                    response_value =
                        DAP_Port_JTAG_Transfer(request_value, (uint8_t *)NULL);
                } while ((response_value == DAP_TRANSFER_WAIT) && retry-- &&
                         !dap_transfer_abort);
                if (response_value != DAP_TRANSFER_OK) {
                    break;
                }
                do {
                    // Read register until its value matches or
                    // retry counter expires
                    retry = dap_data.transfer.retry_count;
                    do {
                        response_value = DAP_Port_JTAG_Transfer(
                            request_value, (uint8_t *)&data);
                    } while ((response_value == DAP_TRANSFER_WAIT) && retry-- &&
                             !dap_transfer_abort);
                    if (response_value != DAP_TRANSFER_OK) {
                        break;
                    }
                } while (
                    ((data & dap_data.transfer.match_mask) != match_value) &&
                    match_retry-- && !dap_transfer_abort);
                if ((data & dap_data.transfer.match_mask) != match_value) {
                    response_value |= DAP_TRANSFER_MISMATCH;
                }
                if (response_value != DAP_TRANSFER_OK) {
                    break;
                }
            } else {
                // Normal read
                if (post_read == 0U) {
                    // Select JTAG chain
                    if (ir != request_ir) {
                        ir = request_ir;
                        DAP_Port_JTAG_IR(ir);
                    }
                    // Post DP/AP read
                    retry = dap_data.transfer.retry_count;
                    do {
                        response_value = DAP_Port_JTAG_Transfer(
                            request_value, (uint8_t *)NULL);
                    } while ((response_value == DAP_TRANSFER_WAIT) && retry-- &&
                             !dap_transfer_abort);
                    if (response_value != DAP_TRANSFER_OK) {
                        break;
                    }
#if (TIMESTAMP_CLOCK != 0U)
                    // Store Timestamp
                    if ((request_value & DAP_TRANSFER_TIMESTAMP) != 0U) {
                        timestamp = dap_data.timestamp;
                        *response++ = (uint8_t)timestamp;
                        *response++ = (uint8_t)(timestamp >> 8);
                        *response++ = (uint8_t)(timestamp >> 16);
                        *response++ = (uint8_t)(timestamp >> 24);
                    }
#endif
                    post_read = 1U;
                }
            }
        } else {
            // Write register
            if (post_read) {
                // Select JTAG chain
                if (ir != JTAG_DPACC) {
                    ir = JTAG_DPACC;
                    DAP_Port_JTAG_IR(ir);
                }
                // Read previous data
                retry = dap_data.transfer.retry_count;
                do {
                    response_value = DAP_Port_JTAG_Transfer(
                        DP_RDBUFF | DAP_TRANSFER_RnW, (uint8_t *)&data);
                } while ((response_value == DAP_TRANSFER_WAIT) && retry-- &&
                         !dap_transfer_abort);
                if (response_value != DAP_TRANSFER_OK) {
                    break;
                }
                // Store previous data
                *response++ = (uint8_t)data;
                *response++ = (uint8_t)(data >> 8);
                *response++ = (uint8_t)(data >> 16);
                *response++ = (uint8_t)(data >> 24);
                post_read = 0U;
            }
            // Load data
            data = (uint32_t)(*(request + 0) << 0) |
                   (uint32_t)(*(request + 1) << 8) |
                   (uint32_t)(*(request + 2) << 16) |
                   (uint32_t)(*(request + 3) << 24);
            request += 4;
            if ((request_value & DAP_TRANSFER_MATCH_MASK) != 0U) {
                // Write match mask
                dap_data.transfer.match_mask = data;
                response_value = DAP_TRANSFER_OK;
            } else {
                // Select JTAG chain
                if (ir != request_ir) {
                    ir = request_ir;
                    DAP_Port_JTAG_IR(ir);
                }
                // Write DP/AP register
                retry = dap_data.transfer.retry_count;
                do {
                    response_value =
                        DAP_Port_JTAG_Transfer(request_value, (uint8_t *)&data);
                } while ((response_value == DAP_TRANSFER_WAIT) && retry-- &&
                         !dap_transfer_abort);
                if (response_value != DAP_TRANSFER_OK) {
                    break;
                }
#if (TIMESTAMP_CLOCK != 0U)
                // Store Timestamp
                if ((request_value & DAP_TRANSFER_TIMESTAMP) != 0U) {
                    timestamp = dap_data.timestamp;
                    *response++ = (uint8_t)timestamp;
                    *response++ = (uint8_t)(timestamp >> 8);
                    *response++ = (uint8_t)(timestamp >> 16);
                    *response++ = (uint8_t)(timestamp >> 24);
                }
#endif
            }
        }
        response_count++;
        if (dap_transfer_abort) {
            break;
        }
    }

    for (; request_count != 0U; request_count--) {
        // Process canceled requests
        request_value = *request++;
        if ((request_value & DAP_TRANSFER_RnW) != 0U) {
            // Read register
            if ((request_value & DAP_TRANSFER_MATCH_VALUE) != 0U) {
                // Read with value match
                request += 4;
            }
        } else {
            // Write register
            request += 4;
        }
    }

    if (response_value == DAP_TRANSFER_OK) {
        // Select JTAG chain
        if (ir != JTAG_DPACC) {
            ir = JTAG_DPACC;
            DAP_Port_JTAG_IR(ir);
        }
        if (post_read) {
            // Read previous data
            retry = dap_data.transfer.retry_count;
            do {
                response_value = DAP_Port_JTAG_Transfer(
                    DP_RDBUFF | DAP_TRANSFER_RnW, (uint8_t *)&data);
            } while ((response_value == DAP_TRANSFER_WAIT) && retry-- &&
                     !dap_transfer_abort);
            if (response_value != DAP_TRANSFER_OK) {
                goto end;
            }
            // Store previous data
            *response++ = (uint8_t)data;
            *response++ = (uint8_t)(data >> 8);
            *response++ = (uint8_t)(data >> 16);
            *response++ = (uint8_t)(data >> 24);
        } else {
            // Check last write
            retry = dap_data.transfer.retry_count;
            do {
                response_value =
                    DAP_Port_JTAG_Transfer(DP_RDBUFF | DAP_TRANSFER_RnW, NULL);
            } while ((response_value == DAP_TRANSFER_WAIT) && retry-- &&
                     !dap_transfer_abort);
        }
    }

end:
    *(response_head + 0) = (uint8_t)response_count;
    *(response_head + 1) = (uint8_t)response_value;

    return (((uint32_t)(request - request_head) << 16) |
            (uint32_t)(response - response_head));
}
#endif

/**
 * @brief 读/写数据
 *
 * @param request
 * @param response
 * @return uint32_t
 */
static uint32_t DAP_TransferBlock(const uint8_t *request, uint8_t *response) {
    uint32_t num;

    switch (dap_data.debug_port) {
#if (DAP_SWD_SUPPORT != 0)
        case DAP_PORT_SWD:
            num = DAP_SWD_TransferBlock(request, response);
            break;
#endif
#if (DAP_JTAG_SUPPORT != 0)
        case DAP_PORT_JTAG:
            num = DAP_JTAG_TransferBlock(request, response);
            break;
#endif
        default:
            *(response + 0) = 0U;  // Response count [7:0]
            *(response + 1) = 0U;  // Response count[15:8]
            *(response + 2) = 0U;  // Response value
            num = 3U;
            break;
    }

    if ((*(request + 3) & DAP_TRANSFER_RnW) != 0U) {
        // Read register block
        num |= 4U << 16;
    } else {
        // Write register block
        num |= (4U + (((uint32_t)(*(request + 1)) |
                       (uint32_t)(*(request + 2) << 8)) *
                      4))
               << 16;
    }

    return (num);
}

#if (DAP_SWD_SUPPORT != 0)
static uint32_t DAP_SWD_TransferBlock(const uint8_t *request,
                                      uint8_t *response) {
    uint32_t request_count;
    uint32_t request_value;
    uint32_t response_count;
    uint32_t response_value;
    uint8_t *response_head;
    uint32_t retry;
    uint32_t data = 0;

    response_count = 0U;
    response_value = 0U;
    response_head = response;
    response += 3;

    dap_transfer_abort = 0U;

    request++;  // Ignore DAP index

    request_count =
        (uint32_t)(*(request + 0) << 0) | (uint32_t)(*(request + 1) << 8);
    request += 2;
    if (request_count == 0U) {
        goto end;
    }

    request_value = *request++;
    if ((request_value & DAP_TRANSFER_RnW) != 0U) {
        // Read register block
        if ((request_value & DAP_TRANSFER_APnDP) != 0U) {
            // Post AP read
            retry = dap_data.transfer.retry_count;
            do {
                response_value = DAP_Port_SWD_Transfer(request_value, NULL);
            } while ((response_value == DAP_TRANSFER_WAIT) && retry-- &&
                     !dap_transfer_abort);
            if (response_value != DAP_TRANSFER_OK) {
                goto end;
            }
        }
        while (request_count--) {
            // Read DP/AP register
            if ((request_count == 0U) &&
                ((request_value & DAP_TRANSFER_APnDP) != 0U)) {
                // Last AP read
                request_value = DP_RDBUFF | DAP_TRANSFER_RnW;
            }
            retry = dap_data.transfer.retry_count;
            do {
                response_value =
                    DAP_Port_SWD_Transfer(request_value, (uint8_t *)&data);
            } while ((response_value == DAP_TRANSFER_WAIT) && retry-- &&
                     !dap_transfer_abort);
            if (response_value != DAP_TRANSFER_OK) {
                goto end;
            }
            // Store data
            *response++ = (uint8_t)data;
            *response++ = (uint8_t)(data >> 8);
            *response++ = (uint8_t)(data >> 16);
            *response++ = (uint8_t)(data >> 24);
            response_count++;
        }
    } else {
        // Write register block
        while (request_count--) {
            // Load data
            data = (uint32_t)(*(request + 0) << 0) |
                   (uint32_t)(*(request + 1) << 8) |
                   (uint32_t)(*(request + 2) << 16) |
                   (uint32_t)(*(request + 3) << 24);
            request += 4;
            // Write DP/AP register
            retry = dap_data.transfer.retry_count;
            do {
                response_value =
                    DAP_Port_SWD_Transfer(request_value, (uint8_t *)&data);
            } while ((response_value == DAP_TRANSFER_WAIT) && retry-- &&
                     !dap_transfer_abort);
            if (response_value != DAP_TRANSFER_OK) {
                goto end;
            }
            response_count++;
        }
        // Check last write
        retry = dap_data.transfer.retry_count;
        do {
            response_value =
                DAP_Port_SWD_Transfer(DP_RDBUFF | DAP_TRANSFER_RnW, NULL);
        } while ((response_value == DAP_TRANSFER_WAIT) && retry-- &&
                 !dap_transfer_abort);
    }

end:
    *(response_head + 0) = (uint8_t)(response_count >> 0);
    *(response_head + 1) = (uint8_t)(response_count >> 8);
    *(response_head + 2) = (uint8_t)response_value;

    return ((uint32_t)(response - response_head));
}
#endif

#if (DAP_JTAG_SUPPORT != 0)
/**
 * @brief
 *
 * @param request
 * @param response
 * @return uint32_t
 */
static uint32_t DAP_JTAG_TransferBlock(const uint8_t *request,
                                       uint8_t *response) {
    uint32_t request_count;
    uint32_t request_value;
    uint32_t response_count;
    uint32_t response_value;
    uint8_t *response_head;
    uint32_t retry;
    uint32_t data;
    uint32_t ir;

    response_count = 0U;
    response_value = 0U;
    response_head = response;
    response += 3;

    dap_transfer_abort = 0U;

    // Device index (JTAP TAP)
    dap_data.jtag_dev.index = *request++;
    if (dap_data.jtag_dev.index >= dap_data.jtag_dev.count) {
        goto end;
    }

    request_count =
        (uint32_t)(*(request + 0) << 0) | (uint32_t)(*(request + 1) << 8);
    request += 2;
    if (request_count == 0U) {
        goto end;
    }

    request_value = *request++;

    // Select JTAG chain
    ir = (request_value & DAP_TRANSFER_APnDP) ? JTAG_APACC : JTAG_DPACC;
    DAP_Port_JTAG_IR(ir);

    if ((request_value & DAP_TRANSFER_RnW) != 0U) {
        // Post read
        retry = dap_data.transfer.retry_count;
        do {
            response_value = DAP_Port_JTAG_Transfer(request_value, NULL);
        } while ((response_value == DAP_TRANSFER_WAIT) && retry-- &&
                 !dap_transfer_abort);
        if (response_value != DAP_TRANSFER_OK) {
            goto end;
        }
        // Read register block
        while (request_count--) {
            // Read DP/AP register
            if (request_count == 0U) {
                // Last read
                if (ir != JTAG_DPACC) {
                    DAP_Port_JTAG_IR(JTAG_DPACC);
                }
                request_value = DP_RDBUFF | DAP_TRANSFER_RnW;
            }
            retry = dap_data.transfer.retry_count;
            do {
                response_value =
                    DAP_Port_JTAG_Transfer(request_value, (uint8_t *)&data);
            } while ((response_value == DAP_TRANSFER_WAIT) && retry-- &&
                     !dap_transfer_abort);
            if (response_value != DAP_TRANSFER_OK) {
                goto end;
            }
            // Store data
            *response++ = (uint8_t)data;
            *response++ = (uint8_t)(data >> 8);
            *response++ = (uint8_t)(data >> 16);
            *response++ = (uint8_t)(data >> 24);
            response_count++;
        }
    } else {
        // Write register block
        while (request_count--) {
            // Load data
            data = (uint32_t)(*(request + 0) << 0) |
                   (uint32_t)(*(request + 1) << 8) |
                   (uint32_t)(*(request + 2) << 16) |
                   (uint32_t)(*(request + 3) << 24);
            request += 4;
            // Write DP/AP register
            retry = dap_data.transfer.retry_count;
            do {
                response_value =
                    DAP_Port_JTAG_Transfer(request_value, (uint8_t *)&data);
            } while ((response_value == DAP_TRANSFER_WAIT) && retry-- &&
                     !dap_transfer_abort);
            if (response_value != DAP_TRANSFER_OK) {
                goto end;
            }
            response_count++;
        }
        // Check last write
        if (ir != JTAG_DPACC) {
            DAP_Port_JTAG_IR(JTAG_DPACC);
        }
        retry = dap_data.transfer.retry_count;
        do {
            response_value =
                DAP_Port_JTAG_Transfer(DP_RDBUFF | DAP_TRANSFER_RnW, NULL);
        } while ((response_value == DAP_TRANSFER_WAIT) && retry-- &&
                 !dap_transfer_abort);
    }

end:
    *(response_head + 0) = (uint8_t)(response_count >> 0);
    *(response_head + 1) = (uint8_t)(response_count >> 8);
    *(response_head + 2) = (uint8_t)response_value;

    return ((uint32_t)(response - response_head));
}
#endif

/**
 * @brief
 *
 * @param request
 * @param response
 * @return uint32_t
 */
static uint32_t DAP_WriteAbort(const uint8_t *request, uint8_t *response) {
    /*
    | BYTE | BYTE *****| WORD *|
    > 0x08 | DAP Index | Abort |
    |******|***********|*******|

    | BYTE | BYTE **|
    < 0x08 | Status |
    |******|********|
    */

    uint32_t num;

    switch (dap_data.debug_port) {
#if (DAP_SWD_SUPPORT != 0)
        case DAP_PORT_SWD:
            num = DAP_SWD_WriteAbort(request, response);
            break;
#endif
#if (DAP_JTAG_SUPPORT != 0)
        case DAP_PORT_JTAG:
            num = DAP_JTAG_WriteAbort(request, response);
            break;
#endif
        default:
            *response = DAP_ERROR;
            num = 1U;
            break;
    }
    return ((5U << 16) | num);
}

#if (DAP_SWD_SUPPORT != 0)
/**
 * @brief
 *
 * @param request
 * @param response
 * @return uint32_t
 */
static uint32_t DAP_SWD_WriteAbort(const uint8_t *request, uint8_t *response) {
    uint32_t data;

    // Load data (Ignore DAP index)
    data = (uint32_t)(*(request + 1) << 0) | (uint32_t)(*(request + 2) << 8) |
           (uint32_t)(*(request + 3) << 16) | (uint32_t)(*(request + 4) << 24);

    // Write Abort register
    DAP_Port_SWD_Transfer(DP_ABORT, (uint8_t *)&data);

    *response = DAP_OK;
    return (1U);
}
#endif

#if (DAP_JTAG_SUPPORT != 0)
/**
 * @brief
 *
 * @param request
 * @param response
 * @return uint32_t
 */
static uint32_t DAP_JTAG_WriteAbort(const uint8_t *request, uint8_t *response) {
    uint32_t data;

    // Device index (JTAP TAP)
    dap_data.jtag_dev.index = *request;
    if (dap_data.jtag_dev.index >= dap_data.jtag_dev.count) {
        *response = DAP_ERROR;
        return (1U);
    }

    // Select JTAG chain
    DAP_Port_JTAG_IR(JTAG_ABORT);

    // Load data
    data = (uint32_t)(*(request + 1) << 0) | (uint32_t)(*(request + 2) << 8) |
           (uint32_t)(*(request + 3) << 16) | (uint32_t)(*(request + 4) << 24);

    // Write Abort register
    DAP_Port_JTAG_WriteAbort(data);

    *response = DAP_OK;
    return (1U);
}
#endif

/**
 * @brief
 *
 * @param request
 * @param response
 * @return uint32_t
 */
static uint32_t DAP_Dummy_Transfer(const uint8_t *request, uint8_t *response) {
    const uint8_t *request_head;
    uint32_t request_count;
    uint32_t request_value;

    request_head = request;

    request++;  // Ignore DAP index

    request_count = *request++;

    for (; request_count != 0U; request_count--) {
        // Process dummy requests
        request_value = *request++;
        if ((request_value & DAP_TRANSFER_RnW) != 0U) {
            // Read register
            if ((request_value & DAP_TRANSFER_MATCH_VALUE) != 0U) {
                // Read with value match
                request += 4;
            }
        } else {
            // Write register
            request += 4;
        }
    }

    *(response + 0) = 0U;  // Response count
    *(response + 1) = 0U;  // Response value

    return (((uint32_t)(request - request_head) << 16) | 2U);
}
