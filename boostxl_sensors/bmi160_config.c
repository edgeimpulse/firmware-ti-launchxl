/**
 * Copyright (C) 2018 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * @file    bmi160_read_sensor_data.c
 * @brief   Sample file to read BMI160 sensor data using COINES library
 *
 */

/*********************************************************************/
/* system header files */
/*********************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include "bmi160.h"

/* TI-Drivers Header files */
#include "ti_drivers_config.h"

/*********************************************************************/
/* local macro definitions */
/*! I2C interface communication, 1 - Enable; 0- Disable */
#define BMI160_INTERFACE_I2C  1

/*! SPI interface communication, 1 - Enable; 0- Disable */
#define BMI160_INTERFACE_SPI  0

#if (!((BMI160_INTERFACE_I2C == 1) && (BMI160_INTERFACE_SPI == 0)) && \
    (!((BMI160_INTERFACE_I2C == 0) && (BMI160_INTERFACE_SPI == 1))))
#error "Invalid value given for the macros BMI160_INTERFACE_I2C / BMI160_INTERFACE_SPI"
#endif


/***************************************************************/
/**\name	BUS READ AND WRITE FUNCTION POINTERS        */
/***************************************************************/
#define BMI160_I2C_ADDR1                   (0x68)
/**< I2C Address needs to be changed */
#define BMI160_I2C_ADDR2                    (0x69)
 /**< I2C Address needs to be changed */
#define BMI160_AUX_BMM150_I2C_ADDRESS       (0x13)
/**< I2C address of BMM150*/
#define BMI160_AUX_YAS532_I2C_ADDRESS       (0x2E)
/**< I2C address of YAS532*/
#define	BMI160_AUX_AKM09911_I2C_ADDR_1		(0x0C)
/**< I2C address of AKM09911*/
#define	BMI160_AUX_AKM09911_I2C_ADDR_2		(0x0D)
/**< I2C address of AKM09911*/
#define	BMI160_AUX_AKM09912_I2C_ADDR_1		(0x0C)
/**< I2C address of AKM09912*/
#define	BMI160_AUX_AKM09912_I2C_ADDR_2		(0x0D)
/**< I2C address of AKM09912*/
#define	BMI160_AUX_AKM09912_I2C_ADDR_3		(0x0E)
/**< I2C address of AKM09912*/
#define	BMI160_AUX_AKM09912_I2C_ADDR_4		(0x0F)
/**< I2C address of AKM09912*/
/*******************************************/
/**\name	CONSTANTS        */
/******************************************/
#define  BMI160_INIT_VALUE					(0)
#define  BMI160_ASSIGN_DATA                 (1)
#define  BMI160_GEN_READ_WRITE_DATA_LENGTH	(1)
#define  BMI160_MAXIMUM_TIMEOUT             (10)

#define	I2C_BUFFER_LEN 8

/*********************************************************************/
/* global variables */
/*********************************************************************/

/*! @brief This structure containing relevant bmi160 info */
struct bmi160_dev bmi160dev;

/*! @brief variable to hold the bmi160 accel data */
struct bmi160_sensor_data bmi160_accel;

/*! @brief variable to hold the bmi160 gyro data */
struct bmi160_sensor_data bmi160_gyro;

static I2C_Handle     i2cHandle    = NULL;

/*********************************************************************/
/* static function declarations */
/*********************************************************************/

/*!
 * @brief   This internal API is used to initialize the sensor driver interface
 */
static void init_bmi160_sensor_driver_interface(void);

/*********************************************************************/
/* functions */
/*********************************************************************/
extern void usleep(uint32_t usec);

#define CONV_TO_2G_FLOAT	(2.0 / 32768.0)

int8_t BMI160_I2C_bus_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t cnt);
int8_t BMI160_I2C_burst_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t cnt);
int8_t BMI160_I2C_bus_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t cnt);
static void BMI160_delay_msek(uint32_t msek);

/*!
 *  @brief This internal API is used to initializes the bmi160 sensor
 *  settings like power mode and OSRS settings.
 *
 *  @param[in] void
 *
 *  @return int -> 0 ok
 *
 */
int init_bmi160(I2C_Handle i2cHndl)
{
    int8_t rslt;

    i2cHandle = i2cHndl;

    init_bmi160_sensor_driver_interface();

    rslt = bmi160_init(&bmi160dev);

    if (rslt != BMI160_OK) {
		return rslt;
	}

	/* Wait for chip reset */
	uint8_t chip_id = 0;
	int try = 100;

	while ((try--) && (chip_id != BMI160_CHIP_ID))
	{
		/* Read chip_id */
		rslt = bmi160_get_regs(BMI160_CHIP_ID_ADDR, &chip_id, 1, &bmi160dev);
	}

	if((chip_id != BMI160_CHIP_ID)) {
		return -1;
	}

    /* Select the Output data rate, range of accelerometer sensor */
    bmi160dev.accel_cfg.odr = BMI160_ACCEL_ODR_1600HZ;
    bmi160dev.accel_cfg.range = BMI160_ACCEL_RANGE_2G;
    bmi160dev.accel_cfg.bw = BMI160_ACCEL_BW_NORMAL_AVG4;

    /* Select the power mode of accelerometer sensor */
    bmi160dev.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;

    /* Select the Output data rate, range of Gyroscope sensor */
    bmi160dev.gyro_cfg.odr = BMI160_GYRO_ODR_3200HZ;
    bmi160dev.gyro_cfg.range = BMI160_GYRO_RANGE_2000_DPS;
    bmi160dev.gyro_cfg.bw = BMI160_GYRO_BW_NORMAL_MODE;

    /* Select the power mode of Gyroscope sensor */
    bmi160dev.gyro_cfg.power = BMI160_GYRO_NORMAL_MODE;

    /* Set the sensor configuration */
    rslt = bmi160_set_sens_conf(&bmi160dev);

	return rslt;
}

/*!
 *  @brief This internal API is used to set the sensor driver interface to
 *  read/write the data.
 *
 *  @param[in] void
 *
 *  @return void
 *
 */
static void init_bmi160_sensor_driver_interface(void)
{
    /* I2C setup */

    /* link read/write/delay function of host system to appropriate
     * bmi160 function call prototypes */
    bmi160dev.write = BMI160_I2C_bus_write;
    bmi160dev.read = BMI160_I2C_bus_read;
    bmi160dev.delay_ms = BMI160_delay_msek;

    /* set correct i2c address */
    bmi160dev.id = BMI160_I2C_ADDR2;
    bmi160dev.intf = BMI160_I2C_INTF;
}

/**
 * @brief Read accelerometer data from sensor
 * 
 * @param acc x y z
 * @return int 
 */
int bmi160_getData(float *acc)
{
    int rslt = bmi160_get_sensor_data((BMI160_ACCEL_SEL | BMI160_GYRO_SEL), &bmi160_accel, &bmi160_gyro, &bmi160dev);

    acc[0] = ((float) bmi160_accel.x) * CONV_TO_2G_FLOAT;
    acc[1] = ((float) bmi160_accel.y) * CONV_TO_2G_FLOAT;
    acc[2] = ((float) bmi160_accel.z) * CONV_TO_2G_FLOAT;    

    return rslt;
}

 /*	\Brief: The function is used as I2C bus read
 *	\Return : Status of the I2C read
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register,
 *            will data is going to be read
 *	\param reg_data : This data read from the sensor,
 *            which is hold in an array
 *	\param cnt : The no of byte of data to be read
 */
int8_t BMI160_I2C_bus_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t cnt)
{
	int32_t ierror = BMI160_INIT_VALUE;
	I2C_Transaction i2cTransaction;
	/* Please take the below function as your reference
	* for read the data using I2C communication
	* add your I2C rad function here.
	* "IERROR = I2C_WRITE_READ_STRING(
	*  DEV_ADDR, ARRAY, ARRAY, C_BMI160_ONE_U8X, CNT)"
	* iError is an return value of SPI write function
	* Please select your valid return value
	* In the driver SUCCESS defined as BMI160_INIT_VALUE
	* and FAILURE defined as -C_BMI160_ONE_U8X
	*/
	i2cTransaction.writeBuf = &reg_addr;
	i2cTransaction.writeCount = 1;
	i2cTransaction.readBuf = reg_data;
	i2cTransaction.readCount = (size_t)cnt;
	i2cTransaction.slaveAddress = dev_addr;
	
	if (!I2C_transfer(i2cHandle,&i2cTransaction))
	{
		ierror = -1;
	}
	
	return (int8_t)ierror;
}

/*	\Brief: The function is used as I2C bus read
*	\Return : Status of the I2C read
*	\param dev_addr : The device address of the sensor
*	\param reg_addr : Address of the first register,
*            will data is going to be read
*	\param reg_data : This data read from the sensor,
*            which is hold in an array
*	\param cnt : The no of byte of data to be read
*/
int8_t BMI160_I2C_burst_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t cnt)
{
	int32_t ierror = BMI160_INIT_VALUE;
	I2C_Transaction i2cTransaction;
	/* Please take the below function as your reference
	* for read the data using I2C communication
	* add your I2C rad function here.
	* "IERROR = I2C_WRITE_READ_STRING(
	*  DEV_ADDR, ARRAY, ARRAY, C_BMI160_ONE_U8X, CNT)"
	* iError is an return value of SPI write function
	* Please select your valid return value
	* In the driver SUCCESS defined as BMI160_INIT_VALUE
	* and FAILURE defined as -C_BMI160_ONE_U8X
	*/
	i2cTransaction.writeBuf = &reg_addr;
	i2cTransaction.writeCount = 1;
	i2cTransaction.readBuf = reg_data;
	i2cTransaction.readCount = (size_t)cnt;
	i2cTransaction.slaveAddress = dev_addr;
	
	if (!I2C_transfer(i2cHandle,&i2cTransaction))
	{
		ierror = -1;
	}
	
	return (int8_t)ierror;
}

 /*	\Brief: The function is used as I2C bus write
 *	\Return : Status of the I2C write
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register,
 *              will data is going to be written
 *	\param reg_data : It is a value hold in the array,
 *		will be used for write the value into the register
 *	\param cnt : The no of byte of data to be write
 */
int8_t BMI160_I2C_bus_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t cnt)
{
	int32_t ierror = BMI160_INIT_VALUE;
	uint8_t array[I2C_BUFFER_LEN];
	uint16_t stringpos = BMI160_INIT_VALUE;
	I2C_Transaction i2cTransaction;
	
	array[BMI160_INIT_VALUE] = reg_addr;
	for (stringpos = BMI160_INIT_VALUE; stringpos < cnt; stringpos++)
		array[stringpos + BMI160_GEN_READ_WRITE_DATA_LENGTH] = *(reg_data +
				stringpos);
	
	i2cTransaction.writeBuf = array;
	i2cTransaction.writeCount = (size_t)cnt + 1;
	i2cTransaction.readCount = 0;
	i2cTransaction.slaveAddress = dev_addr;
	
	/* If transaction success */
	if (!I2C_transfer(i2cHandle, &i2cTransaction)) {
		ierror = -1;
	}
	
	return (int8_t)ierror;
	
	/*
	* Please take the below function as your reference for
	* write the data using I2C communication
	* "IERROR = I2C_WRITE_STRING(DEV_ADDR, ARRAY, CNT+C_BMI160_ONE_U8X)"
	* add your I2C write function here
	* iError is an return value of I2C read function
	* Please select your valid return value
	* In the driver SUCCESS defined as BMI160_INIT_VALUE
	* and FAILURE defined as -C_BMI160_ONE_U8X
	* Note :
	* This is a full duplex operation,
	* The first read data is discarded, for that extra write operation
	* have to be initiated. For that cnt+C_BMI160_ONE_U8X operation done
	* in the I2C write string function
	* For more information please refer data sheet SPI communication:
	*/
}

/*	Brief : The delay routine
 *	\param : delay in ms
*/
static void BMI160_delay_msek(uint32_t msek)
{
	/*Here you can write your own delay routine*/
	usleep(msek * 1000);
}
