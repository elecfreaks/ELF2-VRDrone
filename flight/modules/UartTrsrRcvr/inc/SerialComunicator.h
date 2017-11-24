#ifndef _SERIALCOMUNICATOR_H_
#define _SERIALCOMUNICATOR_H_

#include <openpilot.h>
#include "pios.h"
#include <pios_stm32.h>


#define INBUF_SIZE 64

#define MSP_SET_RAW_RC_TINY			150
#define MSP_ARMING               	161
#define MSP_ARM                  	151
#define MSP_DISARM               	152
#define MSP_BATTERY_VOLTAGE		 	153
#define MSP_GET_VERSION				154 // 0x9A
#define MSP_CALIBRATION				155
#define MSP_ALTITUDE				156
#define MSP_GET_FLIGHT_STATUS		157
#define MSP_ONE_KEY_TAKEOFF		    158
#define MSP_ONE_KEY_LANDING			159
#define MSP_TAKEOFFED				160
#define MSP_LANDED					162
#define MSP_TRIM_ROLL_ADD			163
#define MSP_TRIM_ROLL_SUB			164
#define MSP_TRIM_PITCH_ADD			165
#define MSP_TRIM_PITCH_SUB			166
#define MSP_TRIM_RESTORE			167
#define MSP_TRIM_GET				168
#define MSP_TRIM_SAVE				169
#define MSP_ALT_VEL_ACC				170

extern void recievedByte(uint8_t byte);
extern uint32_t read32();
extern uint16_t read16();
extern uint8_t read8();

extern void sendCommand(uint8_t cmd);
extern void sendData(uint8_t cmd, uint8_t *dat, uint8_t len);

#if defined(PIOS_INCLUDE_ESP8266)
extern void esp8266_recievedByte(uint8_t byte);
extern void initEps8266();
extern void checkSendBuffer();
#endif

#endif


