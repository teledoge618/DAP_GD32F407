#ifndef __APP_ADC_H__
#define __APP_ADC_H__

#include <stdint.h>

void APP_ADC_Init(void);
uint16_t APP_ADC_Read(uint8_t ch_index);

#endif /* !__APP_ADC_H__ */
