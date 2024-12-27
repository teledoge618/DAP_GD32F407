#ifndef __DAP_PORT_H__
#define __DAP_PORT_H__

#include "dap.h"
#include "misc.h"

void DAP_Port_InitHardware(void);
void DAP_Port_SetRunningStatus(uint8_t bit);
void DAP_Port_SetConnectedStatus(uint8_t bit);
void DAP_Port_Delay(uint32_t delay_us);
uint8_t DAP_Port_ResetTarget(void);
void DAP_Port_SetPins(uint8_t select, uint8_t value);
uint8_t DAP_Port_GetPins(void);
uint32_t DAP_Port_GetTimeStamp(void);

void DAP_Port_SWJ_SetClock(uint32_t freq);
void DAP_Port_SWJ_Sequence(uint32_t count, uint8_t *value);
void DAP_Port_SWJ_Disconnect(void);

void DAP_Port_SWD_Connect(void);
void DAP_Port_SWD_Sequence(uint8_t info, const uint8_t *request, uint8_t *response);
uint8_t DAP_Port_SWD_Transfer(uint32_t request, uint8_t *response);

void DAP_Port_JTAG_Connect(void);
void DAP_Port_JTAG_Sequence(uint8_t info, const uint8_t *request, uint8_t *response);
uint8_t DAP_Port_JTAG_Transfer(uint32_t request, uint8_t *data);
void DAP_Port_JTAG_IR(uint32_t ir);
uint32_t DAP_Port_JTAG_ReadIDCode(void);
void DAP_Port_JTAG_WriteAbort(uint32_t data);

#endif  // !__DAP_PORT_H__
