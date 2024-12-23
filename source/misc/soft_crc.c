#include "soft_crc.h"

uint32_t soft_crc32(uint8_t *data, uint32_t len) {
    uint32_t crc32;
    uint32_t n;

    crc32 = 0xFFFFFFFFU;
    while (len != 0U) {
        crc32 ^= ((uint32_t)*data++) << 24U;
        for (n = 8U; n; n--) {
            if (crc32 & 0x80000000U) {
                crc32 <<= 1U;
                crc32 ^= 0x04C11DB7U;
            } else {
                crc32 <<= 1U;
            }
        }
        len--;
    }
    return (crc32);
}
