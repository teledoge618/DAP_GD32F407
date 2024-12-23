#include "app_dap.h"

#include "DAP.h"
#include "DAP_config.h"
#include "app_crc.h"
#include "printf.h"
#include "usbd_cdc.h"
#include "usbd_core.h"

/*
Version	Description

V2.1.1	Allow default clock frequency to use fast clock mode

V2.1.0	Added: UART COM Commands to support target communication via extra UART
        Added: UART Receive/Transmit Buffer Size values in the command DAP_Info
        Added: Target Board Vendor and Target Board Name strings in the command
        DAP_Info Added: Product Firmware Version string in the command DAP_Info
        Changed: String encoding in DAP_Info from ASCII to UTF-8

V2.0.0	Changed: Communication via USB bulk endpoints to achieve high-speed
        transfer rates Added: Streaming SWO via separate USB bulk endpoint
        Added: DAP_SWO_Transport extended with transport mode 2 - Send trace
        data via separate USB bulk endpoint

V1.3.0	Added: Target Board Vendor and Target Board Name strings in the command
        DAP_Info Added: Product Firmware Version string in the command DAP_Info
        Changed: String encoding in DAP_Info from ASCII to UTF-8

V1.2.0	Added: dap_swd_sequence to enable SWD multi-drop target selection
        Added: Test Domain Timer values in the commands DAP_Info, dap_transfer

V1.1.0	Added: SWO Commands to support Serial Wire Output (SWO) in UART mode
        Added: Atomic Commands support for executing time critical DAP commands

V1.0.0	Version 1.0.0 was never released; version number skipped.

V0.02	Renamed DAP_LED to DAP_HostStatus.

V0.01	Beta Release.

*/

extern uint8_t dap_transfer_abort;

/* 唯一序列号字符串 */
char dap_uinque_serial_number[32] = "99A123456789";

/* GD32F407 有6个双向端点 */

#define DAP_IN_EP 0x81  /* PC 2 DAP */
#define DAP_OUT_EP 0x01 /* DAP 2 PC */

#if (SWO_UART_SUPPORT)
#define SWO_IN_EP 0x02 /* SWO */
#endif

#define CDC_IN_EP 0x83
#define CDC_OUT_EP 0x03
#define CDC_INT_EP 0x84

#define USBD_VID 0x0D28
#define USBD_PID 0x0204
#define USBD_MAX_POWER 300      /* 300mA */
#define USBD_LANGID_STRING 1033 /* 1033 = English */

/* 接口描述符长度 */
#define DAP_INTERFACE_SIZE ((9 + 7 + 7) + (7 * SWO_UART_SUPPORT))
#define CDC_INTERFACE_SIZE (CDC_ACM_DESCRIPTOR_LEN)

/* 配置描述符总长度 */
#define USB_CONFIG_SIZE (9 + DAP_INTERFACE_SIZE + CDC_INTERFACE_SIZE)
#define INTF_NUM (1 + 2) /* WINUSB*1 + CDC*2 */

#ifdef CONFIG_USB_HS
#if DAP_PACKET_SIZE != 512
#error "DAP_PACKET_SIZE must be 512 in hs"
#endif
#else /* CONFIG_USB_HS */
#if DAP_PACKET_SIZE != 64
#error "DAP_PACKET_SIZE must be 64 in fs"
#endif
#endif /* !CONFIG_USB_HS */

#define USBD_WINUSB_VENDOR_CODE 0x20
#define FUNCTION_SUBSET_LEN (160)
#define DEVICE_INTERFACE_GUIDS_FEATURE_LEN 132

/* WINUSB描述符长度 */
#define USBD_WINUSB_DESC_SET_LEN (WINUSB_DESCRIPTOR_SET_HEADER_SIZE + FUNCTION_SUBSET_LEN)

// clang-format off

/* BOS描述符 */
static const uint8_t usbd_msosv2_descriptor_raw[] = {
    WBVAL(WINUSB_DESCRIPTOR_SET_HEADER_SIZE),  /* wLength */
    WBVAL(WINUSB_SET_HEADER_DESCRIPTOR_TYPE),  /* wDescriptorType */
    0x00, 0x00, 0x03, 0x06,                    /* dwWindowsVersion*/
    WBVAL(USBD_WINUSB_DESC_SET_LEN),           /* wDescriptorSetTotalLength */

    WBVAL(WINUSB_FUNCTION_SUBSET_HEADER_SIZE), /* wLength */
    WBVAL(WINUSB_SUBSET_HEADER_FUNCTION_TYPE), /* wDescriptorType */
    0,                                         /* bFirstInterface */
    0,                                         /* bReserved */
    WBVAL(FUNCTION_SUBSET_LEN),                /* wSubsetLength */

    WBVAL(WINUSB_FEATURE_COMPATIBLE_ID_SIZE),  /* wLength */
    WBVAL(WINUSB_FEATURE_COMPATIBLE_ID_TYPE),  /* wDescriptorType */
    'W', 'I', 'N', 'U', 'S', 'B', 0, 0,        /* CompatibleId*/
    0, 0, 0, 0, 0, 0, 0, 0,                    /* SubCompatibleId*/
    WBVAL(DEVICE_INTERFACE_GUIDS_FEATURE_LEN), /* wLength */
    WBVAL(WINUSB_FEATURE_REG_PROPERTY_TYPE),   /* wDescriptorType */
    WBVAL(WINUSB_PROP_DATA_TYPE_REG_MULTI_SZ), /* wPropertyDataType */
    WBVAL(42),                                 /* wPropertyNameLength */
    'D', 0, 'e', 0, 'v', 0, 'i', 0, 
    'c', 0, 'e', 0, 'I', 0, 'n', 0, 
    't', 0, 'e', 0, 'r', 0, 'f', 0, 
    'a', 0, 'c', 0, 'e', 0, 'G', 0, 
    'U', 0, 'I', 0, 'D', 0, 's', 0,
    0, 0, 
    WBVAL(80), /* wPropertyDataLength */
    '{', 0, 'C', 0, 'D', 0, 'B', 0, 
    '3', 0, 'B', 0, '5', 0, 'A', 0, 
    'D', 0, '-', 0, '2', 0, '9', 0, 
    '3', 0, 'B', 0, '-', 0, '4', 0, 
    '6', 0, '6', 0, '3', 0, '-', 0, 
    'A', 0, 'A', 0, '3', 0, '6', 0, 
    '-', 0, '1', 0, 'A', 0, 'A', 0, 
    'E', 0, '4', 0, '6', 0, '4', 0, 
    '6', 0, '3', 0, '7', 0, '7', 0, 
    '6', 0, '}', 0,
    0, 0, 
    0, 0
};
// clang-format on

#define USBD_NUM_DEV_CAPABILITIES (1)
#define USBD_WINUSB_DESC_LEN 28U

/* BOS描述符长度 */
#define USBD_BOS_WTOTALLENGTH (0x05 + USBD_WINUSB_DESC_LEN)

// clang-format off

static const uint8_t usbd_bos_descriptor_raw[] = {
    0x05,                         /* bLength */
    0x0f,                         /* bDescriptorType */
    WBVAL(USBD_BOS_WTOTALLENGTH), /* wTotalLength */
    USBD_NUM_DEV_CAPABILITIES,    /* bNumDeviceCaps */

    USBD_WINUSB_DESC_LEN,           /* bLength */
    0x10,                           /* bDescriptorType */
    USB_DEVICE_CAPABILITY_PLATFORM, /* bDevCapabilityType */
    0x00,                           /* bReserved */
    0xDF, 0x60, 0xDD, 0xD8,         /* PlatformCapabilityUUID */
    0x89, 0x45, 0xC7, 0x4C, 
    0x9C, 0xD2, 0x65, 0x9D, 
    0x9E, 0x64, 0x8A, 0x9F,
    0x00, 0x00, 0x03, 0x06,          /* dwWindowsVersion*/
    WBVAL(USBD_WINUSB_DESC_SET_LEN), /* wDescriptorSetTotalLength */
    USBD_WINUSB_VENDOR_CODE,         /* bVendorCode */
    0                                /* bAltEnumCode */
};
// clang-format on

static struct usb_msosv2_descriptor usbd_msosv2_descriptor = {  //
    .vendor_code = USBD_WINUSB_VENDOR_CODE,                     //
    .compat_id = usbd_msosv2_descriptor_raw,                    //
    .compat_id_len = USBD_WINUSB_DESC_SET_LEN};

static struct usb_bos_descriptor usbd_bos_descriptor = {  //
    .string = usbd_bos_descriptor_raw,                    //
    .string_len = USBD_BOS_WTOTALLENGTH};

/* DAP包缓冲区管理 */
static volatile uint32_t request_index_i;  // Request  Index In
static volatile uint32_t request_index_o;  // Request  Index Out
static volatile uint32_t request_count_i;  // Request  Count In
static volatile uint32_t request_count_o;  // Request  Count Out
static volatile uint8_t request_idle;      // Request  Idle  Flag

static volatile uint32_t response_index_i;  // Response Index In
static volatile uint32_t response_index_o;  // Response Index Out
static volatile uint32_t response_count_i;  // Response Count In
static volatile uint32_t response_count_o;  // Response Count Out
static volatile uint8_t response_idle;      // Response Idle  Flag

static volatile __ALIGNED(4) uint8_t request_buff[DAP_PACKET_COUNT][DAP_PACKET_SIZE];
static volatile __ALIGNED(4) uint8_t response_buff[DAP_PACKET_COUNT][DAP_PACKET_SIZE];
static volatile uint16_t resp_size[DAP_PACKET_COUNT];

/* USB事件处理 */
void usbd_event_handler(uint8_t busid, uint8_t event) {
    switch (event) {
        case USBD_EVENT_RESET:
            break;
        case USBD_EVENT_CONNECTED:
            break;
        case USBD_EVENT_DISCONNECTED:
            break;
        case USBD_EVENT_RESUME:
            break;
        case USBD_EVENT_SUSPEND:
            break;
        case USBD_EVENT_CONFIGURED:
            request_idle = 0U;
            /* 开始接收 */
            usbd_ep_start_read(0, DAP_OUT_EP, (uint8_t *)&request_buff[0][0], DAP_PACKET_SIZE);
            break;
        case USBD_EVENT_SET_REMOTE_WAKEUP:
            break;
        case USBD_EVENT_CLR_REMOTE_WAKEUP:
            break;

        default:
            break;
    }
}

/**
 * @brief 计算MCU唯一序列号
 *
 */
static void DAP_GetSerialNumber(void) {
    /* 用GD32F407的唯一ID的CRC值 */
    uint32_t num = APP_CRC_Get(96 / 32, (uint32_t *)0x1FFF7A10);
    snprintf(dap_uinque_serial_number, sizeof(dap_uinque_serial_number), "99A1%08X", num);
    ULOG_INFO("dap uinque serial number: %s\r\n", dap_uinque_serial_number);
}

/**
 * @brief DAP数据接收回调
 *
 * @param busid
 * @param ep
 * @param nbytes
 */
static void APP_DAP_OutCallback(uint8_t busid, uint8_t ep, uint32_t nbytes) {
    if (request_buff[request_index_i][0] == ID_DAP_TransferAbort) {
        dap_transfer_abort = 1U;
    } else {
        request_index_i++;
        if (request_index_i == DAP_PACKET_COUNT) {
            request_index_i = 0U;
        }
        request_count_i++;
    }

    /* Preparing to receive the next data packet */
    if ((uint16_t)(request_count_i - request_count_o) != DAP_PACKET_COUNT) {
        usbd_ep_start_read(0, DAP_OUT_EP, (uint8_t *)&request_buff[request_index_i][0], DAP_PACKET_SIZE);
    } else {
        request_idle = 1U;
    }
}

/**
 * @brief DAP数据发送完毕回调
 *
 * @param busid
 * @param ep
 * @param nbytes
 */
static void APP_DAP_InCallback(uint8_t busid, uint8_t ep, uint32_t nbytes) {
    if (response_count_i != response_count_o) {
        /* Sending packets in the buffer */
        usbd_ep_start_write(0, DAP_IN_EP, (uint8_t *)&response_buff[response_index_o][0], resp_size[response_index_o]);
        response_index_o++;
        if (response_index_o == DAP_PACKET_COUNT) {
            response_index_o = 0U;
        }
        response_count_o++;
    } else {
        response_idle = 1U;
    }
}

// TODO: CDC回调函数

static struct usbd_interface dap_intf;
static struct usbd_interface cdc_intf1;
static struct usbd_interface cdc_intf2;

static struct usbd_endpoint dap_out_ep = {.ep_addr = DAP_OUT_EP, .ep_cb = APP_DAP_OutCallback};
static struct usbd_endpoint dap_in_ep = {.ep_addr = DAP_IN_EP, .ep_cb = APP_DAP_InCallback};
static struct usbd_endpoint cdc_out_ep = {.ep_addr = CDC_OUT_EP, .ep_cb = NULL};
static struct usbd_endpoint cdc_in_ep = {.ep_addr = CDC_IN_EP, .ep_cb = NULL};

/* 设备描述符 */
static const uint8_t usbd_device_descriptor[] = {  //
    USB_DEVICE_DESCRIPTOR_INIT(                    //
        USB_2_1,                                   /* bcdUSB */
        0xEF,                                      /* bDeviceClass */
        0x02,                                      /* bDeviceSubClass */
        0x01,                                      /* bDeviceProtocol */
        USBD_VID,                                  /* idVendor */
        USBD_PID,                                  /* idProduct */
        0x0100,                                    /* bcdDevice */
        0x01                                       /* bNumConfigurations */
        )};

/* 配置描述符 */
static const uint8_t usbd_config_descriptor[] = {
    /* Configuration 0 9bytes */
    USB_CONFIG_DESCRIPTOR_INIT(  //
        USB_CONFIG_SIZE,         /* wTotalLength */
        INTF_NUM,                /* bNumInterfaces */
        0x01,                    /* bConfigurationValue */
        USB_CONFIG_BUS_POWERED,  /* bmAttributes */
        USBD_MAX_POWER           /* bMaxPower */
        ),

    /* Interface 0 9bytes */
    USB_INTERFACE_DESCRIPTOR_INIT(  //
        0x00,                       /* bInterfaceNumber */
        0x00,                       /* bAlternateSetting */
        0x02,                       /* bNumEndpoints */
        0xFF,                       /* bInterfaceClass */
        0x00,                       /* bInterfaceSubClass */
        0x00,                       /* bInterfaceProtocol */
        0x02                        /* iInterface */
        ),

    /* Endpoint OUT 7bytes */
    USB_ENDPOINT_DESCRIPTOR_INIT(  //
        DAP_OUT_EP,                /* bEndpointAddress, */
        USB_ENDPOINT_TYPE_BULK,    /* bmAttributes */
        DAP_PACKET_SIZE,           /* wMaxPacketSize */
        0x00                       /* bInterval */
        ),

    /* Endpoint IN 7bytes */
    USB_ENDPOINT_DESCRIPTOR_INIT(  //
        DAP_IN_EP,                 /* bEndpointAddress, */
        USB_ENDPOINT_TYPE_BULK,    /* bmAttributes */
        DAP_PACKET_SIZE,           /* wMaxPacketSize */
        0x00                       /* bInterval */
        ),

#if (SWO_UART_SUPPORT)
    /* Endpoint IN 7bytes */
    USB_ENDPOINT_DESCRIPTOR_INIT(  //
        SWO_IN_EP,                 /* bEndpointAddress, */
        USB_ENDPOINT_TYPE_BULK,    /* bmAttributes */
        DAP_PACKET_SIZE,           /* wMaxPacketSize */
        0x00                       /* bInterval */
        ),
#endif

    /* CDC */
    CDC_ACM_DESCRIPTOR_INIT(  //
        0x01,                 /*  */
        CDC_INT_EP,           /*  */
        CDC_OUT_EP,           /*  */
        CDC_IN_EP,            /*  */
        DAP_PACKET_SIZE,      /*  */
        0x00                  /*  */
        )};

/* 设备限定描述符 */
static const uint8_t usbd_device_quality_descriptor[] = {  //
    USB_DEVICE_QUALIFIER_DESCRIPTOR_INIT(                  //
        USB_2_1,                                           /* bcdUSB */
        0x00,                                              /* bDeviceClass */
        0x00,                                              /* bDeviceSubClass */
        0x00,                                              /* bDeviceProtocol */
        0x00                                               /* bNumConfigurations */
        )};

/* 字符串描述符 */
static const char *usbd_string_descriptors[] = {  //
    (const char[]){WBVAL(USBD_LANGID_STRING)},    /**/
    CMSIS_DAP_VENDOR_NAME,                        /**/
    CMSIS_DAP_PRODUCT_NAME,                       /**/
    dap_uinque_serial_number};

static const uint8_t *device_descriptor_callback(uint8_t speed) {
    if (speed == USB_SPEED_HIGH) {
        return usbd_device_descriptor;
    } else {
        return NULL;
    }
}

static const uint8_t *config_descriptor_callback(uint8_t speed) {
    if (speed == USB_SPEED_HIGH) {
        return usbd_config_descriptor;
    } else {
        return NULL;
    }
}

static const uint8_t *device_quality_descriptor_callback(uint8_t speed) {
    if (speed == USB_SPEED_HIGH) {
        return usbd_device_quality_descriptor;
    } else {
        return NULL;
    }
}

static const uint8_t *other_speed_descriptor_callback(uint8_t speed) {
    return NULL;  //
}

static const char *string_descriptor_callback(uint8_t speed, uint8_t index) {
    if (index > 3) {
        return NULL;
    }
    return usbd_string_descriptors[index];
}

/* 描述符汇总 */
static struct usb_descriptor usbd_desc = {
    .device_descriptor_callback = device_descriptor_callback,
    .config_descriptor_callback = config_descriptor_callback,
    .device_quality_descriptor_callback = device_quality_descriptor_callback,
    .other_speed_descriptor_callback = other_speed_descriptor_callback,
    .string_descriptor_callback = string_descriptor_callback,
    .msosv1_descriptor = NULL,                    /* MS V1 */
    .msosv2_descriptor = &usbd_msosv2_descriptor, /* MS V2 */
    .webusb_url_descriptor = NULL,                /* WEB */
    .bos_descriptor = &usbd_bos_descriptor        /* BOS */
};

/**
 * @brief 初始化状态量
 *
 */
static void APP_DAP_StateInit(void) {
    // Initialize variables
    request_index_i = 0U;
    request_index_o = 0U;
    request_count_i = 0U;
    request_count_o = 0U;
    request_idle = 1U;
    response_index_i = 0U;
    response_index_o = 0U;
    response_count_i = 0U;
    response_count_o = 0U;
    response_idle = 1U;
}

/**
 * @brief 初始化DAP
 *
 */
void APP_DAP_Init(void) {
    DAP_GetSerialNumber();
    DAP_Setup();
    APP_DAP_StateInit();

    usbd_desc_register(0, &usbd_desc);

    /*!< winusb */
    usbd_add_interface(0, &dap_intf);
    usbd_add_endpoint(0, &dap_out_ep);
    usbd_add_endpoint(0, &dap_in_ep);

    /* CDC */
    usbd_add_interface(0, usbd_cdc_acm_init_intf(0, &cdc_intf1));
    usbd_add_interface(0, usbd_cdc_acm_init_intf(0, &cdc_intf2));
    usbd_add_endpoint(0, &cdc_out_ep);
    usbd_add_endpoint(0, &cdc_in_ep);

    usbd_initialize(0, CONFIG_REG_BASE, usbd_event_handler);
}

/**
 * @brief DAP包处理
 *
 */
void APP_DAP_Handle(void) {
    uint32_t n;

    /* Process pending requests */
    while (request_count_i != request_count_o) {
        /* Handle Queue Commands */
        n = request_index_o;
        while (request_buff[n][0] == ID_DAP_QueueCommands) {
            request_buff[n][0] = ID_DAP_ExecuteCommands;
            n++;
            if (n == DAP_PACKET_COUNT) {
                n = 0U;
            }
            if (n == request_index_i) {
                // flags = osThreadFlagsWait(0x81U, osFlagsWaitAny,
                // osWaitForever); if (flags & 0x80U) {
                //     break;
                // }
            }
        }

        /* Execute DAP Command */
        resp_size[response_index_i] = (uint16_t)DAP_ExecuteCommand((uint8_t *)&request_buff[request_index_o][0], (uint8_t *)&response_buff[response_index_i][0]);

        /* Update Request Index and Count */
        request_index_o++;
        if (request_index_o == DAP_PACKET_COUNT) {
            request_index_o = 0U;
        }
        request_count_o++;

        if (request_idle) {
            if ((uint16_t)(request_count_i - request_count_o) != DAP_PACKET_COUNT) {
                request_idle = 0U;
                usbd_ep_start_read(0, DAP_OUT_EP, (uint8_t *)&request_buff[request_index_i][0], DAP_PACKET_SIZE);
            }
        }

        response_index_i++;
        if (response_index_i == DAP_PACKET_COUNT) {
            response_index_i = 0U;
        }
        response_count_i++;

        if (response_idle) {
            if (response_count_i != response_count_o) {
                // Load data from response buffer to be sent back
                n = response_index_o++;
                if (response_index_o == DAP_PACKET_COUNT) {
                    response_index_o = 0U;
                }
                response_count_o++;
                response_idle = 0U;
                usbd_ep_start_write(0, DAP_IN_EP, (uint8_t *)&response_buff[n][0], resp_size[n]);
            }
        }
    }
}
