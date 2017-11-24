/**
 ******************************************************************************
 * @brief BleRcvr test
 *
 * @file       UartRcvr.c
 * @author     Richile.
 * @brief      UartRcvr test
 *
 *****************************************************************************/

#include <openpilot.h>
#include <string.h>
#include "hwsettings.h"
#include "taskinfo.h"
#include "UartTrsrRcvr.h"
#include "SerialComunicator.h"
#include "flightstatus.h"

#include "objectpersistence.h"
#include "accelgyrosettings.h"
#include "uavobjectmanager.h"
#include "attitudesettings.h"
#include "gyrostate.h"
#include "accelstate.h"
#include "attitudestate.h"

#if (PIOS_ELFX_VERSION >= 36)
#include "velocitystate.h"
#include "CoordinateConversions.h"
#include "fastmath.h"
#include <pios_math.h>
#include "positionstate.h"
#include "altitudeholddesired.h"
#include "stabilizationdesired.h"
#include "attitudesettings.h"
#endif

#include <stdbool.h>
#include <pios_mem.h>


// Private functions
static void uartRcvrTask(void *parameters);
static void additionalTask( void *parameters);
static void updateSettings();
static void FlightStatusUpdate(UAVObjEvent *ev);

#if defined(PIOS_INCLUDE_ESP8266)
#define UART_RCVR_BUF_LEN       		60
#define ADD_WIFI_STACK					100
#else
#define UART_RCVR_BUF_LEN       		20
#define ADD_WIFI_STACK					0
#endif

// Private constants
#if (PIOS_ELFX_VERSION >= 36)
#define UART_RCVR_STACK_SIZE_BYTES		(174+ADD_WIFI_STACK)
#define ADDITIONAL_STACK_SIZE_BYTES		(214+ADD_WIFI_STACK)
#else
#define UART_RCVR_STACK_SIZE_BYTES		(78+ADD_WIFI_STACK)
#define ADDITIONAL_STACK_SIZE_BYTES		(140+ADD_WIFI_STACK)
#endif

#define TASK_PRIORITY        			(tskIDLE_PRIORITY + 1)
#define GRAVITY           				9.80665f

static FlightStatusData flightStatus;
static xTaskHandle uartRcvrTaskHandle;
static xTaskHandle additionalTaskHandle;
static xSemaphoreHandle sem_busy;


/* Com port */
static uint32_t usart_port;
static uint8_t uart_rcvr_buf[UART_RCVR_BUF_LEN];
static bool uart_rcvr_enabled = true;

/* Arm control */
static uint8_t enableDisarm = false;
/* Calibration */
static bool enableCalibration = false;
#if (PIOS_ELFX_VERSION >= 36)
/* Calibration */
static uint8_t enableOneKeyTakeOff = false;
static uint8_t enableOneKeyLanding = false;
#define ONE_KEY_TAKE_OFF_ALTITUDE	1.3f
#define ONE_KEY_LANDING_THROTTLE	(RC_MIN_VALUE + 200) // 200
#endif

#if (PIOS_ELFX_VERSION != 36) && defined(PIOS_INCLUDE_ADC)
/* PIOS_ADC_BATTERY_PIN defined in $PROJECT/flight/targets/boards/$BOARD/pios_board.h */
static int8_t voltageADCPin = PIOS_ADC_BATTERY_PIN;
static uint8_t batteryVoltage = 1;
static float batteryVoltageFloat = 0;
#endif

/* Uart rcvr */
static struct uart_rcvr_dev *uart_dev = 0;
static int32_t Uart_Get(uint32_t rcvr_id, uint8_t channel);
const struct pios_rcvr_driver uart_rcvr_driver = {
    .read          = Uart_Get,
};

#if (PIOS_ELFX_VERSION >= 36)
/* Trim roll and pitch */
static void sendTrimData();
static void saveTrimData();
#endif

/**
 * Get channel value 
 * \return channel value
 */
static int32_t Uart_Get(uint32_t rcvr_id, uint8_t channel)
{
	if(channel > BLE_MAX_NUM_CHANNELS || rcvr_id == 0)
		return PIOS_RCVR_INVALID;
	struct uart_rcvr_dev *rcvr_dev = (struct uart_rcvr_dev *)rcvr_id;

	return rcvr_dev->channelVal[channel];
}

/**
 * Start the module
 * \return -1 if initialisation failed
 * \return 0 on success
 */
static int32_t uartRcvrStart(void)
{
    if (uart_rcvr_enabled && usart_port) {
        // Start tasks
        xTaskCreate(uartRcvrTask, "UartRcvr", UART_RCVR_STACK_SIZE_BYTES, NULL, TASK_PRIORITY, &uartRcvrTaskHandle);
		xTaskCreate(additionalTask, "AdditionalTask", ADDITIONAL_STACK_SIZE_BYTES, NULL, TASK_PRIORITY, &additionalTaskHandle);
        return 0;
    }

    return -1;
}

/**
 * Initialise the module
 * \return -1 if initialisation failed
 * \return 0 on success
 */
static int32_t uartRcvrInitialize(void)
{
	usart_port     = PIOS_COM_UART_RCVR;
	
    if (uart_rcvr_enabled && usart_port) {
        updateSettings();
		sem_busy = xSemaphoreCreateMutex();
    }
	AccelGyroSettingsInitialize();
	ObjectPersistenceInitialize();
	
#if (PIOS_ELFX_VERSION >= 36)
	AltitudeHoldDesiredInitialize();
	PositionStateInitialize();	
#endif

	FlightStatusInitialize();
	FlightStatusConnectCallback(FlightStatusUpdate);
	FlightStatusEnableDisarmGet(&enableDisarm);
	
    return 0;
}
MODULE_INITCALL(uartRcvrInitialize, uartRcvrStart);

/**
 * Called if altitudeHoldDesired is updated
 */
static void FlightStatusUpdate(__attribute__((unused)) UAVObjEvent *ev)
{
	FlightStatusEnableDisarmGet(&enableDisarm);
	FlightStatusGet(&flightStatus);
}

/**
 * Additional task to send voltage of battery, 
 * calibrate and send version id
 */
static void additionalTask(__attribute__((unused)) void *parameters)
{
	portTickType lastSysTime;

    lastSysTime = xTaskGetTickCount();
	uint8_t calibrationProgress = 0;
	uint16_t gyroCount = 0, accelCount = 0;
	float accels[3] = {0.0f};
	float gyros[3] = {0.0f};
	AccelStateData accelStateData;
	GyroStateData gyrosData;
	xQueueHandle accelQueue = xQueueCreate(1, sizeof(UAVObjEvent));
	xQueueHandle gyroQueue = xQueueCreate(1, sizeof(UAVObjEvent));

#if defined(PIOS_INCLUDE_ESP8266)
	vTaskDelay(500 / portTICK_RATE_MS);
	vTaskDelay(500 / portTICK_RATE_MS);
	vTaskDelay(500 / portTICK_RATE_MS);
	vTaskDelay(500 / portTICK_RATE_MS);
	vTaskDelay(500 / portTICK_RATE_MS);
	vTaskDelay(500 / portTICK_RATE_MS);
	vTaskDelay(500 / portTICK_RATE_MS);
	vTaskDelay(500 / portTICK_RATE_MS);
#endif

#if (PIOS_ELFX_VERSION != 36)

#if defined(PIOS_INCLUDE_ADC)
	uint8_t cnt = 0;
	float batteryVoltageSum = 0.0f;
#endif

#endif	

	while(1){
#if defined(PIOS_INCLUDE_ESP8266)
		checkSendBuffer();
#endif
		// Calibrate accel and gyro
		if(enableCalibration){
			/* Start calibration */
			uint8_t biasCorrectGyro;
			AttitudeSettingsBiasCorrectGyroGet(&biasCorrectGyro);
			if(gyroCount == 0 && accelCount == 0 && biasCorrectGyro == true){
				biasCorrectGyro = false;
				AttitudeSettingsBiasCorrectGyroSet(&biasCorrectGyro);
				GyroStateConnectQueue(gyroQueue);
				AccelStateConnectQueue(accelQueue);
				for(uint8_t i=0; i<3; i++){
					gyros[i] = 0.0f;
					accels[i] = 0.0f;
				}
			}
			/* Get sensor data */
			UAVObjEvent ev;
			if (xQueueReceive(gyroQueue, &ev, 0) == true) {
				GyroStateGet(&gyrosData);
				float *gyro = &gyrosData.x;
				for(uint8_t i=0; i<3; i++){
					gyros[i] += gyro[i];
				}
				gyroCount++;
			}
			if (xQueueReceive(accelQueue, &ev, 0) == true) {
				AccelStateGet(&accelStateData);
				float *accel = &accelStateData.x;
				for(uint8_t i=0; i<3; i++){
					accels[i] += accel[i];
				}
				accelCount++;
			}
			
			/* Send progress */
			calibrationProgress = (uint8_t)((gyroCount+accelCount)/6);
			sendData(MSP_CALIBRATION, (unsigned char*)(&calibrationProgress), sizeof(calibrationProgress));
			/* Calibrate finished  */
			if(gyroCount >= 300 && accelCount >= 300){
				enableCalibration = false;
				UAVObjDisconnectQueue(GyroStateHandle(), gyroQueue);
				UAVObjDisconnectQueue(AccelStateHandle(), accelQueue);
				
				for(uint8_t i=0; i<3; i++){
					accels[i] /= (float)accelCount;
					gyros[i] /= (float)gyroCount;
				}
				gyroCount = 0;
				accelCount = 0;
				AttitudeSettingsData attitudeSettingsData;
				AttitudeSettingsGet(&attitudeSettingsData);
								
				AccelGyroSettingsData accelGyroSettings;
				AccelGyroSettingsGet(&accelGyroSettings);
				accelGyroSettings.gyro_bias.X = -gyros[0];
    			accelGyroSettings.gyro_bias.Y = -gyros[1];
    			accelGyroSettings.gyro_bias.Z = -gyros[2];
				accelGyroSettings.accel_bias.X += accels[0];
    			accelGyroSettings.accel_bias.Y += accels[1];
    			accelGyroSettings.accel_bias.Z += (accels[2] + 9.81f);
				AccelGyroSettingsSet(&accelGyroSettings);
				
				attitudeSettingsData.BiasCorrectGyro = true;
				AttitudeSettingsSet(&attitudeSettingsData);
				/* Save datas */
				ObjectPersistenceData objper;
				objper.Operation = OBJECTPERSISTENCE_OPERATION_SAVE;
				objper.Selection = OBJECTPERSISTENCE_SELECTION_SINGLEOBJECT;
				objper.ObjectID = UAVObjGetID(AccelGyroSettingsHandle());
				objper.InstanceID = 0;
				ObjectPersistenceSet(&objper);
				
				for(uint8_t i=0; i<3; i++){
					accels[i] = 0.0f;
					gyros[i] = 0.0f;
				}
			}
		}else{
#if (PIOS_ELFX_VERSION >= 36)
			float down;
			PositionStateDownGet(&down);
#endif
			if(enableDisarm){
				uart_dev->channelVal[ROLL] = RC_MAX_VALUE;
				uart_dev->channelVal[THROTTLE] = RC_MIN_VALUE;
			}else{
				uint8_t armed;
				FlightStatusArmedGet(&armed);
#if (PIOS_ELFX_VERSION >= 36)				
				// 开启一键起飞
				if(enableOneKeyTakeOff){
					if(armed != FLIGHTSTATUS_ARMED_ARMED){
						enableOneKeyTakeOff = false;
						FlightStatusEnableOneKeyTakeOffSet(&enableOneKeyTakeOff);
					}else{
						if(down < -1.0f)
							uart_dev->channelVal[THROTTLE] = RC_MID_VALUE + 350;
						else
							uart_dev->channelVal[THROTTLE] = RC_MID_VALUE + 400;
					}
				}
				// 开启一键降落
				static uint8_t scnt = 0;	// 用于发送降落时的姿态信息，加速度、速度、高度
				static uint8_t tcnt = 0;
				if(enableOneKeyLanding){
					float thrust;
					StabilizationDesiredThrustGet(&thrust);
					if(armed != FLIGHTSTATUS_ARMED_ARMED){
						tcnt = 0;
						enableOneKeyLanding = false;
						FlightStatusEnableOneKeyLandingSet(&enableOneKeyLanding);
						sendCommand(MSP_LANDED);
					}
					uart_dev->channelVal[THROTTLE] = ONE_KEY_LANDING_THROTTLE;
					if(armed == FLIGHTSTATUS_ARMED_ARMED && thrust <= 0.001f){
						tcnt++;
						if(tcnt > 100){
							tcnt = 100;
							enableOneKeyLanding = false;
							FlightStatusEnableOneKeyLandingSet(&enableOneKeyLanding);
							sendCommand(MSP_LANDED);
							uint8_t disarm = true;
							FlightStatusEnableDisarmSet(&disarm);
						}
					}
					scnt++;
					if(scnt >= 4){
						scnt = 0;
						int16_t attitudeDatas[3];
						float velocity, z_accel;
						float q[4];
						VelocityStateDownGet(&velocity);
						AccelStateGet(&accelStateData);
						AttitudeStateq1Get(&q[0]);
						AttitudeStateq2Get(&q[1]);
						AttitudeStateq3Get(&q[2]);
						AttitudeStateq4Get(&q[3]);
						z_accel = calc_ned_accel(q, &(accelStateData.x));
						attitudeDatas[0] = (int16_t)(-down*100);
						attitudeDatas[1] = (int16_t)(velocity*100);
						attitudeDatas[2] = (int16_t)(z_accel*100);
						sendData(MSP_ALT_VEL_ACC, (uint8_t *)&attitudeDatas, 6);
					}
				}else{
					scnt = 0;
					tcnt = 0;
				}
				// 手动起飞或者一键起飞
				float thrust;
				StabilizationDesiredThrustGet(&thrust);
				
				// 起飞完毕
				if(down < -ONE_KEY_TAKE_OFF_ALTITUDE && thrust > 0.01f){
					if(enableOneKeyTakeOff){
						enableOneKeyTakeOff = false;
						FlightStatusEnableOneKeyTakeOffSet(&enableOneKeyTakeOff);
						uart_dev->channelVal[THROTTLE] = RC_MID_VALUE;
					}
					sendCommand(MSP_TAKEOFFED);
				}
#endif
				// 如果飞行过程中飞机翻了就停止飞机
				float roll, pitch;
				AttitudeStateRollGet(&roll);
				AttitudeStatePitchGet(&pitch);
				if(armed == FLIGHTSTATUS_ARMED_ARMED && (fabsf(roll) > 75.0f || fabsf(pitch) > 75.0f)){
#if (PIOS_ELFX_VERSION >= 36)
					if(enableOneKeyTakeOff){
						enableOneKeyTakeOff = false;
						FlightStatusEnableOneKeyTakeOffSet(&enableOneKeyTakeOff);
					}
#endif
					if(!enableDisarm){
						uint8_t disarm = true;
						FlightStatusEnableDisarmSet(&disarm);
					}
				}
#if (PIOS_ELFX_VERSION != 36)
#if defined(PIOS_INCLUDE_ADC)
				if(cnt >= 30){ // 900 ms
					cnt = 0;
					// Get battery voltage			
					batteryVoltageFloat = batteryVoltageSum / 30.0f;
					if(batteryVoltageFloat > 0){
						batteryVoltage = (uint8_t)(batteryVoltageFloat*50);
					}
					batteryVoltageSum = 0.0f;
					// send battery voltage
					if (sem_busy != NULL && xSemaphoreTake(sem_busy, 1000 / portTICK_RATE_MS) == pdTRUE) {
						//sendData(MSP_BATTERY_VOLTAGE, &batteryVoltage, 1);
#if defined(PIOS_INCLUDE_BARO)
						//int16_t alt = (int16_t)(-down*10);
						//sendData(MSP_ALTITUDE, (uint8_t *)&alt, 2);
#endif
						xSemaphoreGive(sem_busy);
					}
				}	
				batteryVoltageSum += PIOS_ADC_PinGetVolt(voltageADCPin);
				cnt++;
#endif
#endif
			}
		}
		vTaskDelayUntil(&lastSysTime, 30 / portTICK_RATE_MS);
	}
}

/**
 * Get uart data task
 */
static void uartRcvrTask(__attribute__((unused)) void *parameters)
{
	FlightStatusGet(&flightStatus);
	uint8_t lastArmed = flightStatus.Armed;
	uint8_t uartIsBreaked = 0;
	uint8_t cnt = 0;
	uint32_t stabilizationThrottle = RC_MIN_VALUE;

#if defined(PIOS_INCLUDE_ESP8266)
	initEps8266();
#endif

    while (1) {
#if (PIOS_ELFX_VERSION >= 36)
		uint8_t breaked;
		AltitudeHoldDesiredConnectionBreakedGet(&breaked);
#endif
        uint32_t rx_bytes = 0;
		memset(uart_rcvr_buf, 0, sizeof(uart_rcvr_buf));
        rx_bytes = PIOS_COM_ReceiveBuffer(usart_port, uart_rcvr_buf, UART_RCVR_BUF_LEN, 500);
		
        if (rx_bytes > 0) {
			cnt = 0;
			uartIsBreaked = 0;
			stabilizationThrottle = RC_MIN_VALUE;
#if (PIOS_ELFX_VERSION >= 36)
			if(breaked != uartIsBreaked)
				AltitudeHoldDesiredConnectionBreakedSet(&uartIsBreaked);
#endif

#if defined(PIOS_INCLUDE_ESP8266)

			uint32_t rx_index = 0;
			for(rx_index = 0; rx_index<rx_bytes; rx_index++){
				esp8266_recievedByte(uart_rcvr_buf[rx_index]);
			}
#else
			uint32_t rx_index = 0;
			for(rx_index = 0; rx_index<rx_bytes; rx_index++){
				recievedByte(uart_rcvr_buf[rx_index]);
			}
#endif
        }else{
        	cnt++;
			if(cnt >= 3){
				cnt = 3;
				uartIsBreaked = 1;
#if (PIOS_ELFX_VERSION >= 36)
				stabilizationThrottle = ONE_KEY_LANDING_THROTTLE;
				if(breaked != uartIsBreaked)
					AltitudeHoldDesiredConnectionBreakedSet(&uartIsBreaked);
#else
				if(stabilizationThrottle == RC_MIN_VALUE && flightStatus.Armed == FLIGHTSTATUS_ARMED_ARMED && uart_dev->channelVal[THROTTLE] > (RC_MIN_VALUE +40)){
#if defined(PIOS_INCLUDE_ADC)
					if(batteryVoltageFloat > 0){
						stabilizationThrottle = RC_MAX_VALUE - 700 * (0.83f*batteryVoltageFloat-2.5f);
						if(batteryVoltageFloat < 3.8f)
							stabilizationThrottle -= 100 * (3.8f - batteryVoltageFloat);
					}else
						stabilizationThrottle = uart_dev->channelVal[THROTTLE] - 40;
#else
					stabilizationThrottle = uart_dev->channelVal[THROTTLE] - 40;
#endif		
				}
#endif
			}
		}
		if(!uart_dev)
			continue;
		if(uartIsBreaked == 1 && !enableDisarm){
			uart_dev->channelVal[THROTTLE] = stabilizationThrottle;
			uart_dev->channelVal[ROLL] = RC_MID_VALUE;
			uart_dev->channelVal[PITCH] = RC_MID_VALUE;
#if (PIOS_ELFX_VERSION >= 36)
			// 失控自动降落
			if(!enableOneKeyLanding){
				enableOneKeyLanding = true;
				FlightStatusEnableOneKeyLandingSet(&enableOneKeyLanding);
			}
#endif
		}
		
		FlightStatusGet(&flightStatus);
		if(lastArmed != flightStatus.Armed){
			if (sem_busy != NULL && xSemaphoreTake(sem_busy, 1000 / portTICK_RATE_MS) == pdTRUE) {
				if(flightStatus.Armed == FLIGHTSTATUS_ARMED_DISARMED){
					sendCommand(MSP_DISARM);
					uint8_t disarm = false;
					FlightStatusEnableDisarmSet(&disarm);
				}else if(flightStatus.Armed == FLIGHTSTATUS_ARMED_ARMING){
					sendCommand(MSP_ARMING);
				}else if(flightStatus.Armed == FLIGHTSTATUS_ARMED_ARMED){
					sendCommand(MSP_ARM);
				}
				xSemaphoreGive(sem_busy);
			}
			lastArmed = flightStatus.Armed;
		}
    }
}

static void updateSettings()
{
    if (usart_port) {
		PIOS_COM_ChangeBaud(usart_port, 115200);
    }
}

uint8_t PIOS_UART_RCVR_Init(uint32_t *uart_rcvr_id)
{
	PIOS_DEBUG_Assert(uart_rcvr_id);

    uart_dev = (struct uart_rcvr_dev *)pios_malloc(sizeof(struct uart_rcvr_dev));
    if (!uart_dev) {
        goto out_fail;
    }
	uart_dev->channelVal[ROLL] = RC_MID_VALUE;
	uart_dev->channelVal[PITCH] = RC_MID_VALUE;
	uart_dev->channelVal[YAW] = RC_MID_VALUE;
	uart_dev->channelVal[THROTTLE] = RC_MIN_VALUE;
	uart_dev->channelVal[AUX1] = RC_MIN_VALUE;
	uart_dev->channelVal[AUX2] = RC_MIN_VALUE;
	uart_dev->channelVal[AUX3] = RC_MIN_VALUE;
	uart_dev->channelVal[AUX4] = RC_MIN_VALUE;

	*uart_rcvr_id = (uint32_t)uart_dev;

    return 0;

out_fail:
    return -1;
}


void MSPCommandCallback(uint8_t cmd)
{
	switch(cmd){
		case MSP_SET_RAW_RC_TINY:{
			if(!enableDisarm){
				uart_dev->channelVal[ROLL] = RC_MIN_VALUE + read8() * 4;
			}else{
				read8();
			}

			uart_dev->channelVal[PITCH] = RC_MIN_VALUE + read8() * 4;
			uart_dev->channelVal[YAW] = RC_MIN_VALUE + read8() * 4;
			
			if(!enableDisarm){
				uint32_t temp = read8() * 4;
				float s = ((float)temp)/((float)(RC_MAX_VALUE-RC_MIN_VALUE));
				if(s < 0.05f){
					uart_dev->channelVal[THROTTLE] = RC_MIN_VALUE;
#if (PIOS_ELFX_VERSION >= 36)
					if(uart_dev->channelVal[ROLL] >= (RC_MID_VALUE + 370) && temp < 20){
						if(enableOneKeyLanding) {enableOneKeyLanding = false;FlightStatusEnableOneKeyLandingSet(&enableOneKeyLanding);}
						if(enableOneKeyTakeOff) {enableOneKeyTakeOff = false;FlightStatusEnableOneKeyTakeOffSet(&enableOneKeyTakeOff);}
					}
#endif
				}else{
#if (PIOS_ELFX_VERSION >= 36)
					if(!enableOneKeyTakeOff){
						if(!enableOneKeyLanding){
							uart_dev->channelVal[THROTTLE] = RC_MIN_VALUE + temp;
						}else if(uart_dev->channelVal[ROLL] >= (RC_MID_VALUE + 370) && temp < 20){
							enableOneKeyLanding = false;
							FlightStatusEnableOneKeyLandingSet(&enableOneKeyLanding);
							uart_dev->channelVal[THROTTLE] = RC_MIN_VALUE + temp;
						}
					}else{
						if(uart_dev->channelVal[ROLL] >= (RC_MID_VALUE + 370) && temp < 20){
							enableOneKeyTakeOff = false;
							FlightStatusEnableOneKeyTakeOffSet(&enableOneKeyTakeOff);
							uart_dev->channelVal[THROTTLE] = RC_MIN_VALUE + temp;
						}
					}
#else
					temp = sqrtf(s*100.0f)*81.0f;
					uart_dev->channelVal[THROTTLE] = RC_MIN_VALUE + 200 + temp;
#endif
					if(uart_dev->channelVal[THROTTLE] > RC_MAX_VALUE)
						uart_dev->channelVal[THROTTLE] = RC_MAX_VALUE;
				}

				FlightStatusGet(&flightStatus);
				if(flightStatus.Armed == FLIGHTSTATUS_ARMED_ARMED && !enableOneKeyTakeOff && 
					!(uart_dev->channelVal[THROTTLE] < (RC_MIN_VALUE+2) && uart_dev->channelVal[ROLL] >= (RC_MID_VALUE + 370))){	// 无头模式

					int16_t roll = RC_MID_VALUE - uart_dev->channelVal[ROLL]; //100
					int16_t pitch = RC_MID_VALUE - uart_dev->channelVal[PITCH]; //200
					
					float heading;
					AttitudeStateYawGet(&heading);
					float radDiff = heading * 0.0174533f; // where PI/180 ~= 0.0174533   90 180
				    float cosDiff = cosf(radDiff); //0 -1
				    float sinDiff = sinf(radDiff);
				    uart_dev->channelVal[ROLL] = RC_MID_VALUE - (roll*cosDiff - pitch*sinDiff);  //100*0-200*1=-200 100*-1-200*0=-100
				    uart_dev->channelVal[PITCH] = RC_MID_VALUE - (pitch*cosDiff + roll*sinDiff); //200*0+100*1=100  200*-1+100*0=-200
				}
			}else{
				read8();
			}
			
			unsigned char auxChannels = read8();
			unsigned char aux = (auxChannels & 0xc0) >> 6;
			if(aux == 0){
				uart_dev->channelVal[AUX1] = RC_MIN_VALUE;
 			}else if(aux == 1){
				uart_dev->channelVal[AUX1] = RC_MID_VALUE;
			}else{
				uart_dev->channelVal[AUX1] = RC_MAX_VALUE;
			}

			aux = (auxChannels & 0x30) >> 4;
			if(aux == 0){
				uart_dev->channelVal[AUX2] = RC_MIN_VALUE;
			}else if(aux == 1){
				uart_dev->channelVal[AUX2] = RC_MID_VALUE;
			}else{
				uart_dev->channelVal[AUX2] = RC_MAX_VALUE;
			}
     
			aux = (auxChannels & 0x0c) >> 2;
			if(aux == 0){
				uart_dev->channelVal[AUX3] = RC_MIN_VALUE;
			}else if(aux == 1){
				uart_dev->channelVal[AUX3] = RC_MID_VALUE;
			}else{
				uart_dev->channelVal[AUX3] = RC_MAX_VALUE;
			}

			aux = (auxChannels & 0x03);
			if(aux == 0){
				uart_dev->channelVal[AUX4] = RC_MIN_VALUE;
			}else if(aux == 1){
				uart_dev->channelVal[AUX4] = RC_MID_VALUE;
			}else{
				uart_dev->channelVal[AUX4] = RC_MAX_VALUE;
			}
		}break;
		case MSP_ARM:{
			sendCommand(MSP_ARM);
		}break;
		case MSP_DISARM:{
			sendCommand(MSP_DISARM);
		}break;
		case MSP_BATTERY_VOLTAGE:{
#if (PIOS_ELFX_VERSION != 36) && defined(PIOS_INCLUDE_ADC)
			sendData(MSP_BATTERY_VOLTAGE, &batteryVoltage, sizeof(batteryVoltage));
#endif
		}break;
		case MSP_GET_VERSION:{ 
			unsigned char versionId = PIOS_ELFX_VERSION;
			sendData(MSP_GET_VERSION, &versionId, sizeof(versionId));
		}break;
		case MSP_CALIBRATION:{
			if(!enableCalibration){
				enableCalibration = true;
			}
		}break;
#if (PIOS_ELFX_VERSION >= 36)
		case MSP_ONE_KEY_TAKEOFF:{
			if(!enableOneKeyTakeOff){
				enableOneKeyTakeOff = true;
				FlightStatusEnableOneKeyTakeOffSet(&enableOneKeyTakeOff);
			}
		}break;
		case MSP_ONE_KEY_LANDING:{
			if(!enableOneKeyLanding){
				enableOneKeyLanding = true;
				FlightStatusEnableOneKeyLandingSet(&enableOneKeyLanding);
			}
		}break;
		case MSP_TRIM_ROLL_ADD:{
			AttitudeSettingsBoardLevelTrimData trimData;
			AttitudeSettingsBoardLevelTrimGet(&trimData);
			trimData.Roll += 0.1f;
			AttitudeSettingsBoardLevelTrimSet(&trimData);
			sendTrimData();
		}break;
		case MSP_TRIM_ROLL_SUB:{
			AttitudeSettingsBoardLevelTrimData trimData;
			AttitudeSettingsBoardLevelTrimGet(&trimData);
			trimData.Roll -= 0.1f;
			AttitudeSettingsBoardLevelTrimSet(&trimData);
			sendTrimData();
		}break;
		case MSP_TRIM_PITCH_ADD:{
			AttitudeSettingsBoardLevelTrimData trimData;
			AttitudeSettingsBoardLevelTrimGet(&trimData);
			trimData.Pitch += 0.1f;
			AttitudeSettingsBoardLevelTrimSet(&trimData);
			sendTrimData();
		}break;
		case MSP_TRIM_PITCH_SUB:{
			AttitudeSettingsBoardLevelTrimData trimData;
			AttitudeSettingsBoardLevelTrimGet(&trimData);
			trimData.Pitch -= 0.1f;
			AttitudeSettingsBoardLevelTrimSet(&trimData);
			sendTrimData();
		}break;
		case MSP_TRIM_RESTORE:{
			AttitudeSettingsBoardLevelTrimData trimData;
			trimData.Roll = 0.0f;
			trimData.Pitch = 0.0f;
			AttitudeSettingsBoardLevelTrimSet(&trimData);
			sendTrimData();
		}break;
		case MSP_TRIM_GET:{
			sendTrimData();
		}break;
		case MSP_TRIM_SAVE:{
			saveTrimData();
			sendCommand(MSP_TRIM_SAVE);
		}break;
#endif
		case MSP_GET_FLIGHT_STATUS:{
			FlightStatusGet(&flightStatus);
			if (sem_busy != NULL && xSemaphoreTake(sem_busy, 1000 / portTICK_RATE_MS) == pdTRUE) {
				if(flightStatus.Armed == FLIGHTSTATUS_ARMED_DISARMED){
					sendCommand(MSP_DISARM);
				}else if(flightStatus.Armed == FLIGHTSTATUS_ARMED_ARMING){
					sendCommand(MSP_ARMING);
				}else if(flightStatus.Armed == FLIGHTSTATUS_ARMED_ARMED){
					sendCommand(MSP_ARM);
				}
				xSemaphoreGive(sem_busy);
			}
		}break;
		default:break;
	}
}

#if (PIOS_ELFX_VERSION >= 36)
static void sendTrimData()
{
	AttitudeSettingsBoardLevelTrimData trimData;
	AttitudeSettingsBoardLevelTrimGet(&trimData);
	int8_t trim[2];
	trim[0] = (int8_t)(trimData.Roll*10);
	trim[1] = (int8_t)(trimData.Pitch*10);
	sendData(MSP_TRIM_GET, (uint8_t *)trim, 2);
}

static void saveTrimData()
{
	/* Save datas */
	ObjectPersistenceData objper;
	objper.Operation = OBJECTPERSISTENCE_OPERATION_SAVE;
	objper.Selection = OBJECTPERSISTENCE_SELECTION_SINGLEOBJECT;
	objper.ObjectID = UAVObjGetID(AttitudeSettingsHandle());
	objper.InstanceID = 0;
	ObjectPersistenceSet(&objper);
}

#endif

