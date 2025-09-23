/*
 * DHT11.h
 *
 *  Created on: Sep 22, 2025
 *      Author: aktas
 */

#ifndef INC_DHT11_H_
#define INC_DHT11_H_

#include "stm32f1xx_hal.h"


/*
 *	@brief : DHT11 den okuduğumuz verileri toplu bir şekilde tutabilmek için.
 *	@param humidity_int    : DHT11 den gelen ilk 8 bitlik paketin integer değeri.(Tam Kısmı)
 *	@param humidity_dec    : DHT11 den gelen 2. 8 bitlik paketin decimal değeri.(Genelde 0 olur)
 *	@param temperature_int : DHT11 den gelen 3. 8 bitlik paketin integer değeri.(Tam Kısmı)
 *	@param temperature_dec : DHT11 den gelen 4. 8 bitlik paketin decimal değeri.(Ondalık kısım)
 *	@param checksum        : DHT11 den gelen son 8 bitlik paketin değeri. Kontrol bitlerinin değerini tutar.
 *	@param databits[40]    : DHT11 den alıncak toplam 40 bitlik verinin tutulduğu ksıısm.
 */
typedef struct
{
	uint8_t humidity_int;
	uint8_t humidity_dec;
	uint8_t temperature_int;
	uint8_t temperature_dec;
	uint8_t checksum;
	uint8_t databits[40];

}DHT11_Data;

/*
 * @brief : DHT11 sensörü için ön hazırlık işlemleri
 * @example -> DHT11_Init(&htim1, GPIOA, 5)
 */
void DHT11_Init(TIM_HandleTypeDef *htim,GPIO_TypeDef *port, uint32_t pin);

/*
 * @brief : DHT11 sensöründen veri okuma işlemi için kullnaılır. 1 dönerse okuma başarılı 0 dönerse başarısız.
 */
uint8_t DHT11_readData(DHT11_Data *dhtdata);

#endif /* INC_DHT11_H_ */
