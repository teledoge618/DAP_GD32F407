#include "dap.h"

uint32_t DAP_ProcessVendorCommand(const uint8_t *request, uint8_t *response) {
    uint32_t num = (1U << 16) | 1U;

    response[0] = request[0];
    uint8_t id = request[1];
    request += 2;

    switch (id) {
#if (DAP_SPI_SUPPORT != 0)
        case ID_SPI_Configure:
            break;
#endif

#if (DAP_I2C_SUPPORT != 0)
        case ID_I2C_Configure:
            break;
#endif
        default:
            response[0] = ID_DAP_Invalid;
            break;
    }

    return (num);
}

uint32_t DAP_ProcessVendorCommandEx(const uint8_t *request, uint8_t *response) {
    uint32_t num = (1U << 16) | 1U;

    response[0] = request[0];
    uint8_t id = request[1];

    switch (id) {
        default:
            response[0] = ID_DAP_Invalid;
            break;
    }

    return (num);
}
