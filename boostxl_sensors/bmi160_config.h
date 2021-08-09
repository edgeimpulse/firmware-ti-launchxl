#ifndef BMI160_CONFIG_H
#define BMI160_CONFIG_H

#include <ti/drivers/GPIO.h>
#include <ti/drivers/I2C.h>

/* Prototypes -------------------------------------------------------------- */
int init_bmi160(I2C_Handle i2cHndl);
#endif