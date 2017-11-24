/**
 ******************************************************************************
 * @addtogroup PIOS PIOS Core hardware abstraction layer
 * @{
 * @addtogroup PIOS_FBM320 FBM320 Functions
 * @brief Hardware functions to deal with the altitude pressure sensor
 * @{
 *
 * @file       pios_fbm320.h  
 * @author     elfx 2016
 * @brief      FBM320 functions header.
 * @see        * 
 *
 *****************************************************************************/

#ifndef PIOS_FBM320_H
#define PIOS_FBM320_H


//! The valid oversampling rates
enum pios_fbm320_osr {
	FBM320_OSR_1024  = 0,
	FBM320_OSR_2048  = 2,
	FBM320_OSR_4096  = 4,
	FBM320_OSR_8192  = 6,
};

enum pios_fbm320_bus_type{
	FBM320_BUS_TYPE_SPI = 0,
	FBM320_BUS_TYPE_I2C = 1,
};

//! Configuration structure for the MS5611 driver
struct pios_fbm320_cfg {
	//! The oversampling setting for the baro, higher produces
	//! less frequenct cleaner data
	enum pios_fbm320_osr oversampling;
	enum pios_fbm320_bus_type bus_type;
};


int32_t PIOS_FBM320_Init(const struct pios_fbm320_cfg *cfg, int32_t i2c_device, int32_t spi_device, int32_t slave_num);

#endif /* PIOS_FBM320_H */

/** 
  * @}
  * @}
  */
