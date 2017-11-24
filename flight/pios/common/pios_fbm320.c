/**
 ******************************************************************************
 * @addtogroup PIOS PIOS Core hardware abstraction layer
 * @{
 * @addtogroup PIOS_FBM320 FBM320 Functions
 * @brief Hardware functions to deal with the altitude pressure sensor
 * @{
 *
 * @file       pios_fbm320.c
 * @author     elfx.
 * @author     elfx
 * @brief      FBM320 Pressure Sensor Routines
 * @see        * 
 *
 ******************************************************************************/

/* Project Includes */
#include "pios.h"
#include "openpilot.h"

#if defined(PIOS_INCLUDE_FBM320_I2C) || defined(PIOS_INCLUDE_FBM320_SPI)

#include <barosensor.h>
#include "pios_fbm320.h"

/* Private constants */
#define FBM320_TASK_PRIORITY	(tskIDLE_PRIORITY + 4)
#define FBM320_TASK_STACK_BYTES	128

/* Private Variables */
uint8_t fbm320_i2c_addr = 0x6D; // 0x6C:ADDR = 0      0x6D:ADDR = 1

/* Private methods */
static int32_t PIOS_FBM320_Read_I2C(uint8_t address, uint8_t * buffer, uint8_t len);
static uint8_t FBM320_Read_I2C(uint8_t address);
static int32_t FBM320_Write_I2C(uint8_t address, uint8_t data);

static uint8_t FBM320_Read_SPI(uint8_t address);
static int32_t FBM320_Write_SPI(uint8_t address, uint8_t data);

static int32_t PIOS_FBM320_GetDelay();
static uint8_t PIOS_FBM320_GetOsr();

static void Coefficient(uint8_t BusType);
static void Calculate(uint8_t BusType, int32_t UP, int32_t UT);
static float Rel_Altitude(long Press, long Ref_P);
static int32_t Abs_Altitude(int32_t Press);
static void PIOS_FBM320_Task(void *parameters);

static int32_t UP_S=0, UT_S=0, RP_S=0, RT_S=0, OffP_S=0;
static int32_t UP_I=0, UT_I=0, RP_I=0, RT_I=0, OffP_I=0;
static float H_S=0, H_I=0;

static uint16_t C0_S, C1_S, C2_S, C3_S, C6_S, C8_S, C9_S, C10_S, C11_S, C12_S; 
static uint32_t C4_S, C5_S, C7_S;
static uint16_t C0_I, C1_I, C2_I, C3_I, C6_I, C8_I, C9_I, C10_I, C11_I, C12_I; 
static uint32_t C4_I, C5_I, C7_I;
static uint8_t BusDetect=0;


/* Private types */
struct fbm320_dev {
	uint32_t i2c_id;
	uint32_t spi_id;
	uint32_t slave_num;
	const struct pios_fbm320_cfg *cfg;
	
#if defined(PIOS_INCLUDE_FREERTOS)
	xTaskHandle task;
	xSemaphoreHandle busy;
#endif

};

static struct fbm320_dev *dev;

/**
 * @brief Allocate a new device
 */
static struct fbm320_dev * PIOS_FBM320_alloc(void)
{
	struct fbm320_dev *fbm320_dev;

	fbm320_dev = (struct fbm320_dev *)pvPortMalloc(sizeof(*fbm320_dev));
	if (!fbm320_dev)
		return (NULL);

	memset(fbm320_dev, 0, sizeof(*fbm320_dev));

#if defined(PIOS_INCLUDE_FREERTOS)
	vSemaphoreCreateBinary(fbm320_dev->busy);
	PIOS_Assert(fbm320_dev->busy != NULL);
#else
	fbm320_dev->busy = false;
#endif

	return fbm320_dev;
}

/**
 * @brief Validate the handle to the i2c device
 * @returns 0 for valid device or <0 otherwise
 */
static int32_t PIOS_FBM320_Validate(struct fbm320_dev *dev)
{
	if (dev == NULL)
		return -1;
	if (dev->i2c_id == 0 && dev->spi_id == 0)
		return -3;
	return 0;
}

/**
 * Initialise the FBM320 sensor
 */
int32_t PIOS_FBM320_Init(const struct pios_fbm320_cfg *cfg, int32_t i2c_device, int32_t spi_device, int32_t slave_num)
{
	dev = PIOS_FBM320_alloc();
	if (dev == NULL)
		return -1;

	dev->i2c_id = i2c_device;
	dev->spi_id = spi_device;
	dev->slave_num = slave_num;
	dev->cfg = cfg;

	xTaskCreate(PIOS_FBM320_Task, "pios_FBM320", FBM320_TASK_STACK_BYTES, NULL, FBM320_TASK_PRIORITY, &dev->task);
	PIOS_Assert(dev->task != NULL);

	return 0;
}

/**
 * @brief Return the delay for the current osr
 */
static int32_t PIOS_FBM320_GetDelay()
{
	if (PIOS_FBM320_Validate(dev) != 0)
		return 10;

	switch(dev->cfg->oversampling) {
	case FBM320_OSR_1024:
		return 3;
	case FBM320_OSR_2048:
		return 4;
	case FBM320_OSR_4096:
		return 6;
	case FBM320_OSR_8192:
		return 10;
	default:
		break;
	}
	return 10;
}

static uint8_t PIOS_FBM320_GetOsr()
{
	switch(dev->cfg->oversampling) {
	case FBM320_OSR_1024:
		return 0x34;
	case FBM320_OSR_2048:
		return 0x74;
	case FBM320_OSR_4096:
		return 0xB4;
	case FBM320_OSR_8192:
		return 0xF4;
	default:
		break;
	}
	return 0xF4;
}

/**
 * Claim the FBM320 device semaphore.
 * \return 0 if no error
 * \return -1 if timeout before claiming semaphore
 */
static int32_t PIOS_FBM320_ClaimDevice(void)
{
	PIOS_Assert(PIOS_FBM320_Validate(dev) == 0);
	if (xSemaphoreTake(dev->busy, 0xffff) != pdTRUE)
		return -1;

	return 0;
}

/**
 * Release the FBM320 device semaphore.
 * \return 0 if no error
 */
static int32_t PIOS_FBM320_ReleaseDevice(void)
{
	PIOS_Assert(PIOS_FBM320_Validate(dev) == 0);
	xSemaphoreGive(dev->busy);
	
	return 0;
}

/**
* Reads one or more bytes into a buffer
* \param[in] the command indicating the address to read
* \param[out] buffer destination buffer
* \param[in] len number of bytes which should be read
* \return 0 if operation was successful
* \return -1 if dev is invalid
* \return -2 if error during I2C transfer
*/
static int32_t PIOS_FBM320_Read_I2C(uint8_t address, uint8_t *buffer, uint8_t len)
{
	if (PIOS_FBM320_Validate(dev) != 0)
		return -1;

	const struct pios_i2c_txn txn_list[] = {
		{
			.info = __func__,
			.addr = fbm320_i2c_addr,
			.rw = PIOS_I2C_TXN_WRITE,
			.len = 1,
			.buf = &address,
		},
		{
			.info = __func__,
			.addr = fbm320_i2c_addr,
			.rw = PIOS_I2C_TXN_READ,
			.len = len,
			.buf = buffer,
		 }
	};

	return PIOS_I2C_Transfer(dev->i2c_id, txn_list, NELEMENTS(txn_list));
}

static uint8_t FBM320_Read_I2C(uint8_t address)
{
	uint8_t data = 0;

	PIOS_FBM320_Read_I2C(address, &data, 1);

	return data;
}

static int32_t FBM320_Write_I2C(uint8_t address, uint8_t buffer)
{
	uint8_t data[] = {
		address,
		buffer,
	};
	
	const struct pios_i2c_txn txn_list[] = {
		{
			.info = __func__,
			.addr = fbm320_i2c_addr,
			.rw = PIOS_I2C_TXN_WRITE,
			.len = sizeof(data),
			.buf = data,
		}
		,
	};

	return PIOS_I2C_Transfer(dev->i2c_id, txn_list, NELEMENTS(txn_list));
}

static int32_t FBM320_ReadADC_I2C()
{
	int32_t val = 0;

	val = (FBM320_Read_I2C(0xF6) << 16)|(FBM320_Read_I2C(0xF7) << 8)|(FBM320_Read_I2C(0xF8) << 0);

	return val;
}

/**
 * @brief Claim the SPI bus for the baro communications and select this chip
 * @return 0 if successful, -1 for invalid device, -2 if unable to claim bus
 */
static int32_t PIOS_FBM320_ClaimBus(void)
{
	if (PIOS_FBM320_Validate(dev) != 0)
		return -1;

	if (PIOS_SPI_ClaimBus(dev->spi_id) != 0)
		return -2;

	PIOS_SPI_RC_PinSet(dev->spi_id, dev->slave_num, 0);

	return 0;
}

/**
 * @brief Release the SPI bus for the baro communications and end the transaction
 * @return 0 if successful
 */
static int32_t PIOS_FBM320_ReleaseBus(void)
{
	if (PIOS_FBM320_Validate(dev) != 0)
		return -1;

	PIOS_SPI_RC_PinSet(dev->spi_id, dev->slave_num, 1);

	return PIOS_SPI_ReleaseBus(dev->spi_id);
}



static uint8_t FBM320_Read_SPI(uint8_t address)
{
	uint8_t data = 0;
	
	if (PIOS_FBM320_Validate(dev) != 0)
		return -1;

	if (PIOS_FBM320_ClaimBus() != 0)
		return -2;

	// R/W W1 W0 A12 A11 .... A1 A0
	// Write instruction data
	if (PIOS_SPI_TransferByte(dev->spi_id, 0x80) < 0) {
		goto out;
	}
	if (PIOS_SPI_TransferByte(dev->spi_id, address) < 0) {
		goto out;
	}
	data = PIOS_SPI_TransferByte(dev->spi_id, 0);

out:
	PIOS_FBM320_ReleaseBus();

	return data;
}


static int32_t FBM320_Write_SPI(uint8_t address, uint8_t data)
{
	if (PIOS_FBM320_Validate(dev) != 0)
		return -1;

	if (PIOS_FBM320_ClaimBus() != 0)
		return -2;

	int32_t rc;
	// R/W W1 W0 A12 A11 .... A1 A0
	// Write instruction data
	if (PIOS_SPI_TransferByte(dev->spi_id, 0) < 0) {
		rc = -3;
		goto out;
	}
	if (PIOS_SPI_TransferByte(dev->spi_id, address) < 0) {
		rc = -3;
		goto out;
	}
	if (PIOS_SPI_TransferByte(dev->spi_id, data) < 0) {
		rc = -3;
		goto out;
	}

	rc = 0;

out:
	PIOS_FBM320_ReleaseBus();

	return rc;

}

static int32_t FBM320_ReadADC_SPI()
{
	int32_t val = 0;

	return val;
}


static void Coefficient(uint8_t BusType)	//Receive Calibrate Coefficient
{
	uint8_t i;
	uint16_t R[10];
	uint16_t C0=0, C1=0, C2=0, C3=0, C6=0, C8=0, C9=0, C10=0, C11=0, C12=0; 
	uint32_t C4=0, C5=0, C7=0;
	if((BusType & 0xF0) == 0x10){
		for(i=0; i<9; i++)
			R[i]=((uint16_t)FBM320_Read_SPI(0xAA + (i*2))<<8) | FBM320_Read_SPI(0xAB + (i*2));
		R[9]=((uint16_t)FBM320_Read_SPI(0xA4)<<8) | FBM320_Read_SPI(0xF1);
	}else if((BusType & 0x0F) == 0x01){
		for(i=0; i<9; i++)
			R[i]=((uint16_t)FBM320_Read_I2C(0xAA + (i*2))<<8) | FBM320_Read_I2C(0xAB + (i*2));
		R[9]=((uint16_t)FBM320_Read_I2C(0xA4)<<8) | FBM320_Read_I2C(0xF1);
	}	

	C0 = R[0] >> 4;
	C1 = ((R[1] & 0xFF00) >> 5) | (R[2] & 7);
	C2 = ((R[1] & 0xFF) << 1) | (R[4] & 1);
	C3 = R[2] >> 3;
	C4 = ((uint32_t)R[3] << 2) | (R[0] & 3);
	C5 = R[4] >> 1;
	C6 = R[5] >> 3;
	C7 = ((uint32_t)R[6] << 3) | (R[5] & 7);
	C8 = R[7] >> 3;
	C9 = R[8] >> 2;
	C10 = ((R[9] & 0xFF00) >> 6) | (R[8] & 3);
	C11 = R[9] & 0xFF;
	C12 = ((R[0] & 0x0C) << 1) | (R[7] & 7);

	if((BusType & 0xF0) == 0x10){
		C0_S = C0;
		C1_S = C1;
		C2_S = C2;
		C3_S = C3;
		C4_S = C4;
		C5_S = C5;
		C6_S = C6;
		C7_S = C7;
		C8_S = C8;
		C9_S = C9;
		C10_S = C10;
		C11_S = C11;
		C12_S = C12;
	}else if((BusType & 0x0F) == 0x01){
		C0_I = C0;
		C1_I = C1;
		C2_I = C2;
		C3_I = C3;
		C4_I = C4;
		C5_I = C5;
		C6_I = C6;
		C7_I = C7;
		C8_I = C8;
		C9_I = C9;
		C10_I = C10;
		C11_I = C11;
		C12_I = C12;
	}	
}

static void Calculate(uint8_t BusType, int32_t UP, int32_t UT)		//Calculate Real Pressure & Temperautre
{
	int8_t C12=0;
	int16_t C0=0, C2=0, C3=0, C6=0, C8=0, C9=0, C10=0, C11=0; 
	int32_t C1=0, C4=0, C5=0, C7=0;
	int32_t RP=0, RT=0;
	int32_t DT, DT2, X01, X02, X03, X11, X12, X13, X21, X22, X23, X24, X25, X26, X31, X32, CF, PP1, PP2, PP3, PP4;

	if((BusType & 0xF0) == 0x10)
	{
		C0 = C0_S;
		C1 = C1_S;
		C2 = C2_S;
		C3 = C3_S;
		C4 = C4_S;
		C5 = C5_S;
		C6 = C6_S;
		C7 = C7_S;
		C8 = C8_S;
		C9 = C9_S;
		C10 = C10_S;
		C11 = C11_S;
		C12 = C12_S;
	}
	else if((BusType & 0x0F) == 0x01)
	{
		C0 = C0_I;
		C1 = C1_I;
		C2 = C2_I;
		C3 = C3_I;
		C4 = C4_I;
		C5 = C5_I;
		C6 = C6_I;
		C7 = C7_I;
		C8 = C8_I;
		C9 = C9_I;
		C10 = C10_I;
		C11 = C11_I;
		C12 = C12_I;
	}	
	
	DT	=	((UT - 8388608) >> 4) + (C0 << 4);
	X01	=	(C1 + 4459) * DT >> 1;
	X02	=	((((C2 - 256) * DT) >> 14) * DT) >> 4;
	X03	=	(((((C3 * DT) >> 18) * DT) >> 18) * DT);
	RT	=	((2500 << 15) - X01 - X02 - X03) >> 15;
				
	DT2	=	(X01 + X02 + X03) >> 12;
				
	X11	=	((C5 - 4443) * DT2);
	X12	=	(((C6 * DT2) >> 16) * DT2) >> 2;
	X13	=	((X11 + X12) >> 10) + ((C4 + 120586) << 4);
				
	X21	=	((C8 + 7180) * DT2) >> 10;
	X22	=	(((C9 * DT2) >> 17) * DT2) >> 12;
	if(X22 >= X21)
		X23	=	X22 - X21;
	else
		X23	=	X21 - X22;
	X24	=	(X23 >> 11) * (C7 + 166426);
	X25	=	((X23 & 0x7FF) * (C7 + 166426)) >> 11;
	if((X22 - X21) < 0)
		X26	=	((0 - X24 - X25) >> 11) + C7 + 166426;
	else	
		X26	=	((X24 + X25) >> 11) + C7 + 166426;
				
	PP1	=	((UP - 8388608) - X13) >> 3;
	PP2	=	(X26 >> 11) * PP1;
	PP3	=	((X26 &(0x7FF)) * PP1) >> 11;
	PP4	=	(PP2 + PP3) >> 10;
				
	CF	=	(2097152 + C12 * DT2) >> 3;
	X31	=	(((CF * C10) >> 17) * PP4) >> 2;
	X32	=	(((((CF * C11) >> 15) * PP4) >> 18) * PP4);
	
	RP	=	((X31 + X32) >> 15) + PP4 + 99880;

	
	if((BusType & 0xF0) == 0x10)
	{
		RP_S = RP;
		RT_S = RT;
	}
	else if((BusType & 0x0F) == 0x01)
	{
		RP_I = RP;
		RT_I = RT;
	}
}

static float Rel_Altitude(long Press, long Ref_P)								//Calculate Relative Altitude
{
	return 44330.0f * (1.0f - powf(((float)Press / (float)Ref_P), (1.0f/5.255f)));
}

static __attribute__((unused)) int32_t Abs_Altitude(int32_t Press)
{
	int8_t P0;			
	int16_t hs1, dP0;			
	int32_t h0, hs0, HP1, HP2;			
					
	if(Press >= 103000)
	{	
		P0	=	103;
		h0	=	-138507;
		hs0	=	-21007;
		hs1	=	311;
	}	
	else if(Press >= 98000)
	{	
		P0	=	98;
		h0	=	280531;
		hs0	=	-21869;
		hs1	=	338;
	}	
	else if(Press >= 93000)
	{	
		P0	=	93;
		h0	=	717253;
		hs0	=	-22813;
		hs1	=	370;
	}	
				
	else if(Press >= 88000)
	{	
		P0	=	88;
		h0	=	1173421;
		hs0	=	-23854;
		hs1	=	407;
	}	
	else if(Press >= 83000)
	{	
		P0	=	83;
		h0	=	1651084;
		hs0	=	-25007;
		hs1	=	450;
	}	
	else if(Press >= 78000)
	{	
		P0	=	78;
		h0	=	2152645;
		hs0	=	-26292;
		hs1	=	501;
	}	
	else if(Press >= 73000)
	{	
		P0	=	73;
		h0	=	2680954;
		hs0	=	-27735;
		hs1	=	560;
	}	
	else if(Press >= 68000)
	{	
		P0	=	68;
		h0	=	3239426;
		hs0	=	-29366;
		hs1	=	632;
	}	
	else if(Press >= 63000)
	{	
		P0	=	63;
		h0	=	3832204;
		hs0	=	-31229;
		hs1	=	719;
	}	
	else if(Press >= 58000)
	{	
		P0	=	58;
		h0	=	4464387;
		hs0	=	-33377;
		hs1	=	826;
	}	
	else if(Press >= 53000)
	{	
		P0	=	53;
		h0	=	5142359;
		hs0	=	-35885;
		hs1	=	960;
	}		
	else if(Press >= 48000)
	{	
		P0	=	48;
		h0	=	5874268;
		hs0	=	-38855;
		hs1	=	1131;
	}	
	else if(Press >= 43000)
	{	
		P0	=	43;
		h0	=	6670762;
		hs0	=	-42434;
		hs1	=	1354;
	}	
	else if(Press >= 38000)
	{	
		P0	=	38;
		h0	=	7546157;
		hs0	=	-46841;
		hs1	=	1654;
	}	
	else if(Press >= 33000)
	{	
		P0	=	33;
		h0	=	8520395;
		hs0	=	-52412;
		hs1	=	2072;
	}	
	else
	{	
		P0	=	28;
		h0	=	9622536;
		hs0	=	-59704;
		hs1	=	2682;
	}
					
	dP0	=	Press - P0 * 1000;
				
	HP1	=	(hs0 * dP0) >> 2;
	HP2	=	(((hs1 * dP0) >> 10)* dP0) >> 4;			

	return	((h0 << 6) + HP1 + HP2) >> 6;
}


/**
* @brief Run self-test operation.
* \return 0 if self-test succeed, -1 if failed
*/
int32_t PIOS_FBM320_Test()
{
	if (PIOS_FBM320_Validate(dev) != 0)
		return -1;

	return 0;
}

static void PIOS_FBM320_Task(__attribute__((unused)) void *parameters)
{	
	bool updateOffset = false;
	int32_t UP_I_sum = 0;
	uint8_t up_count = 1;

	BaroSensorInitialize();
	vTaskDelay(1000 / portTICK_RATE_MS);
	
	// Read partid id
	PIOS_FBM320_ClaimDevice();
	if(dev->cfg->bus_type == FBM320_BUS_TYPE_I2C){
		uint8_t temp = 0;
		if((temp = FBM320_Read_I2C(0x6B)) == 0x42){
			BusDetect |= 0x01;
			Coefficient(BusDetect);
		}
	}else if(dev->cfg->bus_type == FBM320_BUS_TYPE_SPI){
		/*if(FBM320_Read_SPI(0x6B) == 0x42){
			BusDetect |= 0x10;
			Coefficient(BusDetect);
			printf2("spi ok!\r\n");
		}else{
			printf2("spi failed!\r\n");
		}*/
	}
	PIOS_FBM320_ReleaseDevice();

	while (1) {
		if((BusDetect & 0x0F) == 0x01){			//I2C Calibration Sequence
			PIOS_FBM320_ClaimDevice();
			FBM320_Write_I2C(0xF4, 0x2E); 		// temperature
			vTaskDelay(3 / portTICK_RATE_MS);
			UT_I = FBM320_ReadADC_I2C();
			FBM320_Write_I2C(0xF4, PIOS_FBM320_GetOsr());	// pressure
			vTaskDelay(PIOS_FBM320_GetDelay() / portTICK_RATE_MS);
			UP_I = FBM320_ReadADC_I2C();
			PIOS_FBM320_ReleaseDevice();
			
			Calculate(BusDetect, UP_I, UT_I);
			
			if(updateOffset == false){
				UP_I_sum += UP_I;
				OffP_I = UP_I_sum/up_count;
				up_count++;
				if(up_count == 30)
					updateOffset = true;
			}else{
				H_I = Rel_Altitude(RP_I, 101325);

				// Compute the altitude from the pressure and temperature and send it out
				if(RT_I > -4000 && RT_I < 8500 && RP_I > 1000 && RP_I < 120000){
					
					BaroSensorData data;
					data.Temperature = ((float) RT_I) / 100.0f;
					data.Pressure = ((float) RP_I) / 1000.0f;
					data.Altitude = H_I;
					
					BaroSensorSet(&data);
				}
			}
		}else if((BusDetect & 0xF0) == 0x10){
			PIOS_FBM320_ClaimDevice();
			FBM320_Write_SPI(0xF4, 0x2E);
			vTaskDelay(3 / portTICK_RATE_MS);
			UT_S = FBM320_ReadADC_SPI();
			FBM320_Write_SPI(0xF4, PIOS_FBM320_GetOsr());
			vTaskDelay(PIOS_FBM320_GetDelay() / portTICK_RATE_MS);
			UP_S = FBM320_ReadADC_SPI();
			PIOS_FBM320_ReleaseDevice();
			Calculate(BusDetect, UP_S, UT_S);
			if(updateOffset == false){
				OffP_S = UP_S;
				updateOffset = true;
			}else{
				H_S = Rel_Altitude(RP_S, OffP_S);
			}
		}else{
			return ;
		}
		//}
		//FBM320_Write_SPI(0x00, 0x81);
		//printf2("FBM320_Read_SPI=0x%02X\r\n", FBM320_Read_SPI(0x00));
		//PIOS_Thread_Sleep(100);
	}
}


#endif

/**
 * @}
 * @}
 */
