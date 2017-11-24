
#ifndef _UART_TRSR_RECV_H_
#define _UART_TRSR_RECV_H_

#include "pios.h"
#include <pios_stm32.h>

#define BLE_MAX_NUM_CHANNELS	8
#define RC_MIN_VALUE			1000
#define RC_MID_VALUE			1500
#define RC_MAX_VALUE			2000

enum {
	ROLL,
	PITCH,
	YAW,
	THROTTLE,
	AUX1,
	AUX2,
	AUX3,
	AUX4
};

struct uart_rcvr_dev {
	uint32_t channelVal[BLE_MAX_NUM_CHANNELS];
};

extern const struct pios_rcvr_driver uart_rcvr_driver;
extern uint8_t PIOS_UART_RCVR_Init(uint32_t *uart_rcvr_id);
extern void OnEvaluateCommand(uint8_t cmd);
extern int32_t Write(uint8_t *pData, uint16_t len);
extern uint16_t Read(uint8_t *buf, uint16_t buf_len, uint32_t timeout_ms);

#endif


