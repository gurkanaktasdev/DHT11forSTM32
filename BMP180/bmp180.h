/*
 * bmp180.h
 *
 *  Created on: Dec 1, 2025
 *      Author: aktas
 */

#ifndef INC_BMP180_H_
#define INC_BMP180_H_

#include "stm32f4xx_hal.h"

// oss değerleri

#define BMP180_LOWPOWER      0
#define BMP180_STANDARD      1
#define BMP180_HIGHRES       2
#define BMP180_ULTRAHIGHRES  3

void BMP180_Init(I2C_HandleTypeDef *hi2c1);
float BMP180_GetTemperature(void);
long BMP180_GetPressure(uint8_t oss);
double BMP180_GetAltitude(uint8_t oss);
float low_pass_filter_alt(double alt);


#endif /* INC_BMP180_H_ */
