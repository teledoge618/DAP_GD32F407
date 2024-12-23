#ifndef __APP_CRC_H__
#define __APP_CRC_H__

#include <stdint.h>

void APP_CRC_Init(void);
uint32_t APP_CRC_Get(uint32_t len, uint32_t *src_data);

#endif /* !__APP_CRC_H__ */
