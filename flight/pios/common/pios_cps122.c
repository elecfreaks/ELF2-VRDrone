/**
 ******************************************************************************
 * @addtogroup PIOS PIOS Core hardware abstraction layer
 * @{
 * @addtogroup PIOS_CPS122 CPS122 Functions
 * @brief Hardware functions to deal with the altitude pressure sensor
 * @{
 *
 * @file       pios_CPS122.c
 * @author     elfx
 * @brief      CPS122 Pressure Sensor Routines
 * @see        * 
 ******************************************************************************/



/* Project Includes */
#include "pios.h"
#include "openpilot.h"

#if defined(PIOS_INCLUDE_CPS122)

#include "pios_cps122.h"
#include <barosensor.h>


/* Private constants */
#define CPS122_TASK_PRIORITY	(tskIDLE_PRIORITY + configMAX_PRIORITIES - 1)	// max priority
#define CPS122_TASK_STACK		128

/* CPS122 Addresses */
#define CPS122_I2C_ADDR       0x6D
#define CPS122_CTRL_ADDR      0x30
#define CPS122_START_CON      0x0A
#define CPS122_ADC_MSB		  0x06
#define CPS122_P0             101.3250f


/* Private methods */
static int32_t PIOS_CPS122_Read(uint8_t address, uint8_t *buffer, uint8_t len);
static int32_t PIOS_CPS122_WriteCommand(uint8_t address, uint8_t buffer);
static void PIOS_CPS122_Task(void *parameters);

/* Private types */

/* Local Types */
struct CPS122_dev {
	uint32_t i2c_id;
	xTaskHandle task;
	xSemaphoreHandle busy;

	int64_t pressure_unscaled;
	float temperature_unscaled;
};

static struct CPS122_dev *dev;

/**
 * @brief Allocate a new device
 */
static struct CPS122_dev *PIOS_CPS122_alloc(void)
{
   struct CPS122_dev *CPS122_dev;

	CPS122_dev = (struct CPS122_dev *)pvPortMalloc(sizeof(*CPS122_dev));
	if (!CPS122_dev) return (NULL);

	vSemaphoreCreateBinary(CPS122_dev->busy);
	PIOS_Assert(CPS122_dev->busy != NULL);
	
	return(CPS122_dev);
}

/**
 * @brief Validate the handle to the i2c device
 * @returns 0 for valid device or <0 otherwise
 */
static int32_t PIOS_CPS122_Validate(struct CPS122_dev *dev)
{
	if (dev == NULL)
		return -1;
	if (dev->i2c_id == 0)
		return -3;
	return 0;
}

/**
 * Initialise the CPS122 sensor
 */
int32_t PIOS_CPS122_Init(int32_t i2c_device)
{
	dev = (struct CPS122_dev *) PIOS_CPS122_alloc();
	if (dev == NULL)
		return -1;

	dev->i2c_id = i2c_device;

	xTaskCreate(PIOS_CPS122_Task, "pios_CPS122", CPS122_TASK_STACK, NULL, CPS122_TASK_PRIORITY, &dev->task);
	PIOS_Assert(dev->task != NULL);

	return 0;
}


/**
 * Claim the MS5611 device semaphore.
 * \return 0 if no error
 * \return -1 if timeout before claiming semaphore
 */
static int32_t PIOS_CPS122_ClaimDevice(void)
{
	PIOS_Assert(PIOS_CPS122_Validate(dev) == 0);

	if (xSemaphoreTake(dev->busy, 0xffff) != pdTRUE)
		return -1;

	return 0;
}

/**
 * Release the MS5611 device semaphore.
 * \return 0 if no error
 */
static int32_t PIOS_CPS122_ReleaseDevice(void)
{
	PIOS_Assert(PIOS_CPS122_Validate(dev) == 0);

	xSemaphoreGive(dev->busy);
	
	return 0;
}


/**
* Start the ADC conversion
* \param[in] PRESSURE_CONV or TEMPERATURE_CONV to select which measurement to make
* \return 0 for success, -1 for failure (conversion completed and not read)
*/
static int32_t PIOS_CPS122_StartADC()
{
	if (PIOS_CPS122_Validate(dev) != 0)
		return -1;

	while (PIOS_CPS122_WriteCommand(CPS122_CTRL_ADDR, CPS122_START_CON))
		continue;

	return 0;
}

/**
 * @brief Return the delay for the current osr
 */
static int32_t PIOS_CPS122_GetDelay() {
	if (PIOS_CPS122_Validate(dev) != 0)
		return 100;

	return 7;
}

/**
* Read the ADC conversion value (once ADC conversion has completed)
* \return 0 if successfully read the ADC, -1 if failed
*/
static int32_t PIOS_CPS122_ReadADC(void)
{
	if (PIOS_CPS122_Validate(dev) != 0)
		return -1;

	uint8_t data[5];

	/* Read and store the 16bit result */
	uint32_t raw_temperature;
	uint32_t raw_pressure;

	/* Read the pressure conversion */
	if (PIOS_CPS122_Read(CPS122_ADC_MSB, data, 5) != 0)
		return -1;

	raw_temperature = (data[3]<<8) | data[4];
	dev->temperature_unscaled = (float)raw_temperature / 256.0f; //°„C

	raw_pressure = (data[0]<<16) | (data[1]<<8) | data[2];
	dev->pressure_unscaled = (float)raw_pressure / 64.0f; // pa
		
	return 0;
}

/*
* Reads one or more bytes into a buffer
* \param[in] the command indicating the address to read
* \param[out] buffer destination buffer
* \param[in] len number of bytes which should be read
* \return 0 if operation was successful
* \return -1 if error during I2C transfer
*/
static int32_t PIOS_CPS122_Read(uint8_t address, uint8_t *buffer, uint8_t len)
{

	if (PIOS_CPS122_Validate(dev) != 0)
		return -1;

	const struct pios_i2c_txn txn_list[] = {
		{
			.info = __func__,
			.addr = CPS122_I2C_ADDR,
			.rw = PIOS_I2C_TXN_WRITE,
			.len = 1,
			.buf = &address,
		}
		,
		{
		 .info = __func__,
		 .addr = CPS122_I2C_ADDR,
		 .rw = PIOS_I2C_TXN_READ,
		 .len = len,
		 .buf = buffer,
		 }
	};

	return PIOS_I2C_Transfer(dev->i2c_id, txn_list, NELEMENTS(txn_list));
}

/**
* Writes one or more bytes to the CPS122
* \param[in] address Register address
* \param[in] buffer source buffer
* \return 0 if operation was successful
* \return -1 if error during I2C transfer
*/
static int32_t PIOS_CPS122_WriteCommand(uint8_t address, uint8_t buffer)
{
	uint8_t data[] = {
		address,
		buffer,
	};

	if (PIOS_CPS122_Validate(dev) != 0)
		return -1;

	const struct pios_i2c_txn txn_list[] = {
		{
		 .info = __func__,
		 .addr = CPS122_I2C_ADDR,
		 .rw = PIOS_I2C_TXN_WRITE,
		 .len = sizeof(data),
		 .buf = data,
		 }
		,
	};

	return PIOS_I2C_Transfer(dev->i2c_id, txn_list, NELEMENTS(txn_list));
}

/**
* @brief Run self-test operation.
* \return 0 if self-test succeed, -1 if failed
*/
int32_t PIOS_CPS122_Test()
{
	if (PIOS_CPS122_Validate(dev) != 0)
		return -1;

	// TODO: Is there a better way to test this than just checking that pressure/temperature has changed?
	int32_t cur_value = 0;

	PIOS_CPS122_ClaimDevice();
	cur_value = dev->temperature_unscaled;
	PIOS_CPS122_StartADC();
	vTaskDelay(10 / portTICK_RATE_MS);
	PIOS_CPS122_ReadADC();
	PIOS_CPS122_ReleaseDevice();
	if (cur_value == dev->temperature_unscaled)
		return -1;

	return 0;
}
static void PIOS_CPS122_Task(__attribute__((unused)) void *parameters)
{
	int32_t read_adc_result = 0;
	float lastPressure = 0.0f;
	float pressureArray[2] = {0,0};
	uint8_t cnt = 0;

	BaroSensorInitialize();
	vTaskDelay(1000 / portTICK_RATE_MS);
	
	while (1) {

		// Update the pressure data
		PIOS_CPS122_ClaimDevice();
		PIOS_CPS122_StartADC();
		vTaskDelay(PIOS_CPS122_GetDelay() / portTICK_RATE_MS);
		read_adc_result = PIOS_CPS122_ReadADC();
		PIOS_CPS122_ReleaseDevice();

		// Compute the altitude from the pressure and temperature and send it out
		if (read_adc_result == 0 && dev->temperature_unscaled > -40.0f && dev->temperature_unscaled < 85.0f && dev->pressure_unscaled > 30000 && dev->pressure_unscaled < 120000) {
			BaroSensorData data;
			data.Temperature = dev->temperature_unscaled;
			
			float currentPressure = ((float) dev->pressure_unscaled) / 1000.0f;
			if(lastPressure == 0)
				lastPressure = currentPressure;
			data.Pressure = lastPressure * 0.75f + currentPressure * 0.25f;
			lastPressure = data.Pressure;

			pressureArray[cnt++] = data.Pressure;
			if(cnt == 2){
				cnt = 0;
				float pressure = (pressureArray[0] + pressureArray[1]) / 2.0f;
				data.Altitude = 44330.0f * (1.0f - powf(pressure / CPS122_P0, (1.0f / 5.255f)));
			}
			
			BaroSensorSet(&data);
		}
	}
}

#endif /* PIOS_INCLUDE_CPS122 */
