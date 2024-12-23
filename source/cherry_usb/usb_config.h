/*
 * Copyright (c) 2022, sakumisu
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef __CHERRYUSB_CONFIG_H__
#define __CHERRYUSB_CONFIG_H__

#define CONFIG_USB_HS

#define CONFIG_REG_BASE 0x40040000UL

/* ================ USB common Configuration ================ */

/* data align size when use dma */
#ifndef CONFIG_USB_ALIGN_SIZE
#define CONFIG_USB_ALIGN_SIZE 4
#endif

/* 高级描述符 */
#define CONFIG_USBDEV_ADVANCE_DESC

/* attribute data into no cache ram */
#define USB_NOCACHE_RAM_SECTION __attribute__((section(".ram1")))

/* ================= USB Device Stack Configuration ================ */

#define CONFIG_USBDEV_MAX_BUS 1  // for now, bus num must be 1 except hpm ip

/* Ep0 max transfer buffer, specially for receiving data from ep0 out */
#define CONFIG_USBDEV_REQUEST_BUFFER_LEN 256

/* Setup packet log for debug */
// #define CONFIG_USBDEV_SETUP_LOG_PRINT

/* Enable test mode */
// #define CONFIG_USBDEV_TEST_MODE

/* ================ USB Device Port Configuration ================*/

#ifndef CONFIG_USBDEV_EP_NUM
#define CONFIG_USBDEV_EP_NUM 6
#endif

#endif  // !__CHERRYUSB_CONFIG_H__
