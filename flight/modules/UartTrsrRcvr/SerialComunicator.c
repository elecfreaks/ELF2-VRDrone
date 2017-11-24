/**
 ******************************************************************************
 * @brief SerialComunicator test
 *
 * @file       SerialComunicator.c
 * @author     Richile.
 * @brief      SerialComunicator test
 *
 *****************************************************************************/

#include "SerialComunicator.h"

#if defined(PIOS_INCLUDE_UART_RCVR)

static uint8_t inBuf[INBUF_SIZE];
static uint8_t dataPackage[20];
static uint8_t indRX = 0;
static uint8_t dataLength = 0;
static uint8_t cmd = 0;
static uint8_t idx = 0;
static uint8_t checksum = 0;

#if defined(PIOS_INCLUDE_ESP8266)

static bool esp8266_IsLinked;

struct esp8266_SendDatas{
	uint8_t esp8266_SendBuf[12];
	uint8_t esp8266_SendBufLen;
};
#define SEND_DATAS_SIZE		5
static struct esp8266_SendDatas sendDatas[SEND_DATAS_SIZE];
static uint8_t sendIndex = SEND_DATAS_SIZE;
static bool sendOk = true;
static xSemaphoreHandle sem_busy;
#endif
extern void MSPCommandCallback(uint8_t cmd);

enum MSP_STATE {
	MSP_STATE_HEAD0,
	MSP_STATE_HEAD1,
	MSP_STATE_HEAD2,
	MSP_STATE_DATA_LENGTH,
	MSP_STATE_CMD,
	MSP_STATE_DATA,
	MSP_STATE_CHECKSUM
};

enum MSP_STATE msp_state = MSP_STATE_HEAD0;


#if defined(PIOS_INCLUDE_ESP8266)

static bool isHaveString(uint8_t byte, const char *string, uint8_t *index);

enum RECV_STATE {
	RECV_STATE_F0,
	RECV_STATE_I,
	RECV_STATE_P,
	RECV_STATE_D,
	RECV_STATE_F1,
	RECV_STATE_0,
	RECV_STATE_F2,
	RECV_STATE_LEN,
	RECV_STATE_DATA
};
enum RECV_STATE recv_state = RECV_STATE_F0;

enum SEND_START_STATE {
	SEND_START_STATE_A,
	SEND_START_STATE_T,
	SEND_START_STATE_F0,
	SEND_START_STATE_C,
	SEND_START_STATE_I,
	SEND_START_STATE_P,
	SEND_START_STATE_S,
	SEND_START_STATE_E,
	SEND_START_STATE_N,
	SEND_START_STATE_D,
	SEND_START_STATE_X,
	SEND_START_STATE_F1
};
enum SEND_START_STATE send_start_state = SEND_START_STATE_A;


#endif

void recievedByte(uint8_t byte)
{
	// example:24 4D 3C 05 22 01 02 03 04 05 checksum
	switch (msp_state){
		case MSP_STATE_HEAD0:
			if(byte == 0x24)
				msp_state = MSP_STATE_HEAD1;
		break;
		case MSP_STATE_HEAD1:
			if(byte == 0x4D)
				msp_state = MSP_STATE_HEAD2;
			else
				msp_state = MSP_STATE_HEAD0;
		break;
		case MSP_STATE_HEAD2:
			if(byte == 0x3C){
				msp_state = MSP_STATE_DATA_LENGTH;
				checksum = 0;
			}else
				msp_state = MSP_STATE_HEAD0;
		break;
		case MSP_STATE_DATA_LENGTH:
			dataLength = byte;
			checksum ^= byte;
			msp_state = MSP_STATE_CMD;
		break;
		case MSP_STATE_CMD:
			cmd = byte;
			checksum ^= byte;
			if(dataLength == 0)
				msp_state = MSP_STATE_CHECKSUM;
			else
				msp_state = MSP_STATE_DATA;
			idx = 0;
			indRX = 0;
		break;
		case MSP_STATE_DATA:
			inBuf[idx++] = byte;
			checksum ^= byte;
			if(idx == dataLength)
				msp_state = MSP_STATE_CHECKSUM;
			else if(idx >= INBUF_SIZE){
				msp_state = MSP_STATE_HEAD0;
				idx = 0;
			}
		break;
		case MSP_STATE_CHECKSUM:
			if(checksum == byte)
				MSPCommandCallback(cmd);
			dataLength = 0;
			cmd = 0;
			idx = 0;
			indRX = 0;
			checksum = 0;
			msp_state = MSP_STATE_HEAD0;
		break;
	}
}

uint32_t read32() {
	uint32_t t = read16();
	t+= (uint32_t)read16()<<16;
	return t;
}
uint16_t read16() {
	uint16_t t = read8();
	t+= (uint16_t)read8()<<8;
	return t;
}
uint8_t read8()  {
	if(indRX < INBUF_SIZE)
		return inBuf[indRX++]&0xff;
	else
		return 0;
}

void sendCommand(uint8_t cmd)
{
	dataPackage[0] = '$';
	dataPackage[1] = 'M';
	dataPackage[2] = '>';
	dataPackage[3] = 0;
	dataPackage[4] = cmd;
	dataPackage[5] = cmd;

#if defined(PIOS_INCLUDE_ESP8266)
	/* Send data */
	if (esp8266_IsLinked && sem_busy != NULL && xSemaphoreTake(sem_busy, 1000 / portTICK_RATE_MS) == pdTRUE){
		for(uint8_t i=0; i<SEND_DATAS_SIZE; i++){
			if(sendDatas[i].esp8266_SendBufLen == 0){
				memcpy(sendDatas[i].esp8266_SendBuf, dataPackage, 6);
				sendDatas[i].esp8266_SendBufLen = 6;
				break;
			}
		}
		xSemaphoreGive(sem_busy);
	}
#else	
	PIOS_COM_SendBuffer(PIOS_COM_UART_RCVR, dataPackage, 6);
#endif
}

void sendData(uint8_t cmd, uint8_t *dat, uint8_t len)
{
	dataPackage[0] = '$';
	dataPackage[1] = 'M';
	dataPackage[2] = '>';
	dataPackage[3] = len;
	dataPackage[4] = cmd;
	uint8_t checkSum = 0;
	uint8_t i=0;
	checkSum ^= dataPackage[3];
	checkSum ^= dataPackage[4];
	for(i=0; i<len; i++){
		dataPackage[5+i] = dat[i];
		checkSum ^= dat[i];
	}
	dataPackage[5+i] = checkSum;

#if defined(PIOS_INCLUDE_ESP8266)
	/* Send data */
	if (esp8266_IsLinked && sem_busy != NULL && xSemaphoreTake(sem_busy, 1000 / portTICK_RATE_MS) == pdTRUE){
		for(uint8_t i=0; i<SEND_DATAS_SIZE; i++){
			if(sendDatas[i].esp8266_SendBufLen == 0){
				memcpy(sendDatas[i].esp8266_SendBuf, dataPackage, len+6);
				sendDatas[i].esp8266_SendBufLen = len+6;
				break;
			}
		}
		xSemaphoreGive(sem_busy);
	}
#else	
	PIOS_COM_SendBuffer(PIOS_COM_UART_RCVR, dataPackage, len+6);
#endif
}

#if defined(PIOS_INCLUDE_ESP8266)
void esp8266_recievedByte(uint8_t byte)
{
	static uint8_t linkIndex = 0;
	static uint8_t unlinkIndex = 0;
	static uint8_t sendOkIndex = 0;
	
	isHaveString(byte, "Link", &linkIndex);
	if(linkIndex == 4){
		esp8266_IsLinked = true;
		linkIndex = 0;
	}

	isHaveString(byte, "Unlink", &unlinkIndex);
	if(unlinkIndex == 6){
		esp8266_IsLinked = false;
		if (sem_busy != NULL && xSemaphoreTake(sem_busy, 1000 / portTICK_RATE_MS) == pdTRUE){
			sendOk = true;
			sendIndex = SEND_DATAS_SIZE;
			xSemaphoreGive(sem_busy);
		}
		unlinkIndex = 0;
	}

	isHaveString(byte, "SEND OK", &sendOkIndex);
	if(sendOkIndex == 7){
		if (sem_busy != NULL && xSemaphoreTake(sem_busy, 1000 / portTICK_RATE_MS) == pdTRUE){
			sendOk = true;
			sendDatas[sendIndex].esp8266_SendBufLen = 0;
			sendIndex = SEND_DATAS_SIZE;
			xSemaphoreGive(sem_busy);
		}
		sendOkIndex = 0;
	}


	static bool start_recv = false;
	static uint8_t dataLength = 0;
	static uint8_t dataCount = 0;
	if(byte == '+' || byte == 'I' || byte == 'P' || byte == 'D' 
		|| (byte >= '0' && byte <= '9')
		|| byte == ',' || byte == ':' || start_recv){
		switch (recv_state){
			case RECV_STATE_F0:
				if(byte == '+') recv_state = RECV_STATE_I;
				break;
			case RECV_STATE_I:
				if(byte == 'I') recv_state = RECV_STATE_P;
				else			recv_state = RECV_STATE_F0;
				break;
			case RECV_STATE_P:
				if(byte == 'P') recv_state = RECV_STATE_D;
				else			recv_state = RECV_STATE_F0;
				break;
			case RECV_STATE_D:
				if(byte == 'D') recv_state = RECV_STATE_F1;
				else			recv_state = RECV_STATE_F0;
				break;
			case RECV_STATE_F1:
				if(byte == ',') recv_state = RECV_STATE_0;
				else			recv_state = RECV_STATE_F0;
				break;
			case RECV_STATE_0:
				if(byte >= '0' && byte <= '9') 	recv_state = RECV_STATE_F2;
				else							recv_state = RECV_STATE_F0;
				break;
			case RECV_STATE_F2:
				if(byte == ',') recv_state = RECV_STATE_LEN;
				else			recv_state = RECV_STATE_F0;
				break;
			case RECV_STATE_LEN:
				if(byte >= '0' && byte <= '9'){
					dataLength = dataLength * 10 + (byte - '0');
				}else{
					if(byte != ':')	recv_state = RECV_STATE_F0;
					else			{recv_state = RECV_STATE_DATA;start_recv = true;}
				}
				break;
			case RECV_STATE_DATA:
				recievedByte(byte);
				dataCount++;
				if(dataCount == dataLength){
					dataCount = 0;
					dataLength = 0;
					recv_state = RECV_STATE_F0;
					start_recv = false;
				}
				break;
		}
	}else{
		recv_state = RECV_STATE_F0;
	}

	if(byte == 'A' || byte == 'T' || byte == '+' || byte == 'C' 
		|| byte == 'I' || byte == 'P' || byte == 'S' || byte == 'E' || byte == 'N' || byte == 'D' 
		|| byte == '=' || (byte >= '0' && byte <= '9')
		|| byte == ',' || byte == '\r' || byte == '\n' || byte == '>' || byte == 0x20){
		switch (send_start_state){
			case SEND_START_STATE_A:
				if(byte == 'A') send_start_state = SEND_START_STATE_T;
				break;
			case SEND_START_STATE_T:
				if(byte == 'T') send_start_state = SEND_START_STATE_F0;
				else			send_start_state = SEND_START_STATE_A;
				break;
			case SEND_START_STATE_F0:
				if(byte == '+') send_start_state = SEND_START_STATE_C;
				else			send_start_state = SEND_START_STATE_A;
				break;
			case SEND_START_STATE_C:
				if(byte == 'C') send_start_state = SEND_START_STATE_I;
				else			send_start_state = SEND_START_STATE_A;
				break;
			case SEND_START_STATE_I:
				if(byte == 'I') send_start_state = SEND_START_STATE_P;
				else			send_start_state = SEND_START_STATE_A;
				break;
			case SEND_START_STATE_P:
				if(byte == 'P') send_start_state = SEND_START_STATE_S;
				else			send_start_state = SEND_START_STATE_A;
				break;
			case SEND_START_STATE_S:
				if(byte == 'S') send_start_state = SEND_START_STATE_E;
				else			send_start_state = SEND_START_STATE_A;
				break;
			case SEND_START_STATE_E:
				if(byte == 'E') send_start_state = SEND_START_STATE_N;
				else			send_start_state = SEND_START_STATE_A;
				break;
			case SEND_START_STATE_N:
				if(byte == 'N') send_start_state = SEND_START_STATE_D;
				else			send_start_state = SEND_START_STATE_A;
				break;
			case SEND_START_STATE_D:
				if(byte == 'D') send_start_state = SEND_START_STATE_X;
				else			send_start_state = SEND_START_STATE_A;
				break;
			case SEND_START_STATE_X:
				if(byte == '>') send_start_state = SEND_START_STATE_F1;
				break;
			case SEND_START_STATE_F1:
				if (sem_busy != NULL && xSemaphoreTake(sem_busy, 1000 / portTICK_RATE_MS) == pdTRUE){
					if(byte == 0x20 && sendIndex < SEND_DATAS_SIZE){
						PIOS_COM_SendBuffer(PIOS_COM_UART_RCVR, sendDatas[sendIndex].esp8266_SendBuf, sendDatas[sendIndex].esp8266_SendBufLen);
					}else{
						PIOS_COM_SendBuffer(PIOS_COM_UART_RCVR, (const uint8_t *)"test123", 7);
					}
					xSemaphoreGive(sem_busy);
				}else{
					PIOS_COM_SendBuffer(PIOS_COM_UART_RCVR, (const uint8_t *)"test123", 7);
				}
				send_start_state = SEND_START_STATE_A;
				break;
		}
	}else{
		send_start_state = SEND_START_STATE_A;
	}

}

static bool isHaveString(uint8_t byte, const char *string, uint8_t *index)
{
	(*index)++;
	if(byte == *(((uint8_t *)string) + ((*index)-1)))
		return true;
	else{
		(*index) = 0;
		
		return false;
	}
}

void initEps8266()
{
	sem_busy = xSemaphoreCreateMutex();
	memset(sendDatas, 0, sizeof(sendDatas));

	vTaskDelay(500 / portTICK_RATE_MS);
	vTaskDelay(500 / portTICK_RATE_MS);
	vTaskDelay(500 / portTICK_RATE_MS);
	vTaskDelay(500 / portTICK_RATE_MS);
	vTaskDelay(500 / portTICK_RATE_MS);
	vTaskDelay(500 / portTICK_RATE_MS);

	uint8_t tempBuf[60];
	
	/* Start setup ESP8266 */
	/* 1. Set mux */
	PIOS_COM_SendBuffer(PIOS_COM_UART_RCVR, (const uint8_t *)"AT+CIPMUX=1\r\n", 13);
	vTaskDelay(100 / portTICK_RATE_MS);
	PIOS_COM_ReceiveBuffer(PIOS_COM_UART_RCVR, tempBuf, 60, 500);
	
	/* 2. Enable server */
	PIOS_COM_SendBuffer(PIOS_COM_UART_RCVR, (const uint8_t *)"AT+CIPSERVER=1,7777\r\n", 21);
	vTaskDelay(100 / portTICK_RATE_MS);
	PIOS_COM_ReceiveBuffer(PIOS_COM_UART_RCVR, tempBuf, 60, 500);
	
	/* 3. Set timeout */
	PIOS_COM_SendBuffer(PIOS_COM_UART_RCVR, (const uint8_t *)"AT+CIPSTO=2880\r\n", 16);
	vTaskDelay(100 / portTICK_RATE_MS);
	PIOS_COM_ReceiveBuffer(PIOS_COM_UART_RCVR, tempBuf, 60, 500);
	PIOS_COM_ReceiveBuffer(PIOS_COM_UART_RCVR, tempBuf, 60, 100);
	PIOS_COM_ReceiveBuffer(PIOS_COM_UART_RCVR, tempBuf, 60, 100);
	PIOS_COM_ReceiveBuffer(PIOS_COM_UART_RCVR, tempBuf, 60, 100);
	PIOS_COM_ReceiveBuffer(PIOS_COM_UART_RCVR, tempBuf, 60, 100);
	PIOS_COM_ReceiveBuffer(PIOS_COM_UART_RCVR, tempBuf, 60, 100);
	PIOS_COM_ReceiveBuffer(PIOS_COM_UART_RCVR, tempBuf, 60, 100);
	PIOS_COM_ReceiveBuffer(PIOS_COM_UART_RCVR, tempBuf, 60, 100);
}

void checkSendBuffer()
{
	static uint32_t lastSendTime = 0;
	if (esp8266_IsLinked && sem_busy != NULL && xSemaphoreTake(sem_busy, 1000 / portTICK_RATE_MS) == pdTRUE){
		uint8_t i=0;
		for(i=0; i<SEND_DATAS_SIZE && sendOk == true; i++){
			if(sendDatas[i].esp8266_SendBufLen != 0){
				PIOS_COM_SendFormattedString(PIOS_COM_UART_RCVR, "AT+CIPSEND=0,%d\r\n", sendDatas[i].esp8266_SendBufLen);
				sendIndex = i;
				sendOk = false;
				lastSendTime = PIOS_DELAY_GetRaw();
				break;
			}
		}
		// Retry
		if(sendOk == false && PIOS_DELAY_DiffuS(lastSendTime) > 1000000){
			sendOk = true;
		}
		xSemaphoreGive(sem_busy);
	}
}


#endif

#endif

