/*
 * DHT11.c
 *
 *  Created on: Dec 1, 2025
 *      Author: aktas
 */

#include "bmp180.h"
#include <math.h>

// Global Private Variables
static I2C_HandleTypeDef *bmp_i2c;

// Device Addresses (Write: 0xEE, Read: 0xEF)
#define BMP180_devAdress  0xEE

#define BMP180_Pa_SeaLevel     101325.0f

// Calibration Data Registers
short AC1, AC2, AC3;
unsigned short AC4, AC5, AC6;
short B1, B2;
short MB, MC, MD;

// Calculation Variables
long B5 = 0;
long UT = 0;
long UP = 0;

// Filter Variables
float alpha = 0.1;
float filtered_alt = 0;

static void BMP180_Read_CalibrationData(void);
static long BMP180_Get_UT(void);
static long BMP180_Get_UP(uint8_t oss);


// --- PUBLIC FUNCTIONS ---

void BMP180_Init(I2C_HandleTypeDef *hi2c1)
{
    bmp_i2c = hi2c1;

    HAL_Delay(100);
    BMP180_Read_CalibrationData();
}

float BMP180_GetTemperature(void)
{
    long X1, X2;

    UT = BMP180_Get_UT();

    X1 = ((UT - (long)AC6) * (long)AC5) >> 15;
    if (X1 + MD == 0) return 0;
    X2 = ((long)MC << 11) / (X1 + MD);
    B5 = X1 + X2;
    float temp = (B5 + 8) >> 4;
    return temp / 10.0f;
}

long BMP180_GetPressure(uint8_t oss)
{
    long B6, X1, X2, X3, B3, p;
    unsigned long B4, B7;

    if (B5 == 0) {
        BMP180_GetTemperature();
    }

    UP = BMP180_Get_UP(oss);

    B6 = B5 - 4000;

    X1 = (B2 * ((B6 * B6) >> 12)) >> 11;
    X2 = (AC2 * B6) >> 11;
    X3 = X1 + X2;
    B3 = (((((long)AC1) * 4 + X3) << oss) + 2) >> 2;

    X1 = (AC3 * B6) >> 13;
    X2 = (B1 * ((B6 * B6) >> 12)) >> 16;
    X3 = ((X1 + X2) + 2) >> 2;
    B4 = (AC4 * (unsigned long)(X3 + 32768)) >> 15;
    B7 = ((unsigned long)UP - B3) * (50000 >> oss);

    if (B7 < 0x80000000)
    {
        p = (B7 << 1) / B4;
    }
    else
    {
        p = (B7 / B4) << 1;
    }

    X1 = (p >> 8) * (p >> 8);
    X1 = (X1 * 3038) >> 16;
    X2 = (-7357 * p) >> 16;

    p = p + ((X1 + X2 + 3791) >> 4);

    return p;
}

double BMP180_GetAltitude(uint8_t oss)
{
    long p = BMP180_GetPressure(oss);

    double ratio = (double)p / BMP180_Pa_SeaLevel;
    double alt = 44330.0 * (1.0 - pow(ratio, 0.1903)); // 1/5.255 = ~0.1903

    return alt;
}

float low_pass_filter_alt(double alt)
{
    if(filtered_alt == 0) filtered_alt = alt;

    filtered_alt = (alpha * alt) + ((1.0 - alpha) * filtered_alt);
    return filtered_alt;
}

// --- PRIVATE FUNCTIONS ---

static void BMP180_Read_CalibrationData(void)
{
    uint8_t clbrt_datas[22] = {0};

    HAL_I2C_Mem_Read(bmp_i2c, BMP180_devAdress, 0xAA, 1, clbrt_datas, 22, 1000);

    AC1 = ((short)clbrt_datas[0] << 8) | clbrt_datas[1];
    AC2 = ((short)clbrt_datas[2] << 8) | clbrt_datas[3];
    AC3 = ((short)clbrt_datas[4] << 8) | clbrt_datas[5];
    AC4 = ((unsigned short)clbrt_datas[6] << 8) | clbrt_datas[7];
    AC5 = ((unsigned short)clbrt_datas[8] << 8) | clbrt_datas[9];
    AC6 = ((unsigned short)clbrt_datas[10] << 8) | clbrt_datas[11];
    B1  = ((short)clbrt_datas[12] << 8) | clbrt_datas[13];
    B2  = ((short)clbrt_datas[14] << 8) | clbrt_datas[15];
    MB  = ((short)clbrt_datas[16] << 8) | clbrt_datas[17];
    MC  = ((short)clbrt_datas[18] << 8) | clbrt_datas[19];
    MD  = ((short)clbrt_datas[20] << 8) | clbrt_datas[21];
}

static long BMP180_Get_UT(void)
{
    uint8_t data = 0x2E;
    uint8_t buffer[2] = {0};

    HAL_I2C_Mem_Write(bmp_i2c, BMP180_devAdress, 0xF4, 1, &data, 1, 100);
    HAL_Delay(5); // Wait at least 4.5ms
    HAL_I2C_Mem_Read(bmp_i2c, BMP180_devAdress, 0xF6, 1, buffer, 2, 100);

    return ((buffer[0] << 8) | buffer[1]);
}

static long BMP180_Get_UP(uint8_t oss)
{
    uint8_t data = 0x34 | (oss << 6);
    uint8_t buffer[3] = {0};

    HAL_I2C_Mem_Write(bmp_i2c, BMP180_devAdress, 0xF4, 1, &data, 1, 100);

    switch(oss)
    {
        case 0: HAL_Delay(5); break;
        case 1: HAL_Delay(8); break;
        case 2: HAL_Delay(14); break;
        case 3: HAL_Delay(26); break;
    }

    HAL_I2C_Mem_Read(bmp_i2c, BMP180_devAdress, 0xF6, 1, buffer, 3, 100);

    return ((buffer[0] << 16) | (buffer[1] << 8) | buffer[2]) >> (8 - oss);
}
