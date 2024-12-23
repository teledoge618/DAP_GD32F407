#include "app_crc.h"

#include "gd32f4xx_libopt.h"

/**
 * @brief 初始化CRC外设
 *
 */
void APP_CRC_Init(void) {
    rcu_periph_clock_enable(RCU_CRC);
    crc_data_register_reset();
}

/**
 * @brief 计算连续数据的CRC
 *
 * @param len 长度
 * @param src_data 数据地址
 * @return uint32_t CRC结果
 */
uint32_t APP_CRC_Get(uint32_t len, uint32_t *src_data) {
    crc_data_register_reset();
    return crc_block_data_calculate(src_data, len);
}
