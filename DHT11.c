/*
 * DHT11.c
 *
 *  Created on: Sep 23, 2025
 *      Author: aktas
 */

#include "DHT11.h"

static TIM_HandleTypeDef *dht_timer;
static GPIO_TypeDef *dht_port;
static uint32_t dht_pin;

/*
 *	@brief : İlgili dosyada timer ve GPIO konfigürasyonları için yapılandırılmıştır.
 *	@param  htim -> TIM_HandleTypeDef türünden bir timer ın pointırı .
				  bu timerın birimi Mikrosaniye(us) türünden olmalıdır. Aksi taktirde istenilen hedefe ulaşılamaz.
 *	@param port -> GPIO_TypeDef türünden  buraya girilen parametreler(GPIOx türünden , GPIOA , GPIOB ..)
 *	@param pin  -> GPIO_PIN_7 türünden bir değer .
 */
void DHT11_Init(TIM_HandleTypeDef *htim,GPIO_TypeDef *port, uint32_t pin)
{
	dht_timer = htim;
	dht_port = port;
	dht_pin = pin;
}

/*
 *	@brief : Bu fonksiyonun mikrosaniye(us) biriminde çalışmak amacı ile yapılandırılmıştır.
 */
static void DHT11_Delay_us(uint32_t time)
{
	__HAL_TIM_SET_COUNTER(dht_timer,0);
	while(time >__HAL_TIM_GET_COUNTER(dht_timer));
}

/*
 *	@brief : DHT_Init() ile alınan GPIO parametrelerine göre ilgili pini Input moduna çevirir.
 */
static void setPinInput(void)
{
	GPIO_InitTypeDef DHT11Data= {0};
	DHT11Data.Pin = dht_pin;
	DHT11Data.Mode = GPIO_MODE_INPUT;
	DHT11Data.Pull = GPIO_NOPULL;
	DHT11Data.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(dht_port, &DHT11Data);
}

/*
 *	@brief : DHT_Init() ile alınan GPIO parametrelerine göre ilgili pini Output moduna çevirir.
 */
static void setPinOutput(void)
{
	GPIO_InitTypeDef DHT11Data= {0};
	HAL_GPIO_WritePin(dht_port, dht_pin, GPIO_PIN_SET);

	DHT11Data.Pin = dht_pin;
	DHT11Data.Mode = GPIO_MODE_OUTPUT_PP;
	DHT11Data.Pull = GPIO_NOPULL;
	DHT11Data.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(dht_port, &DHT11Data);
}

/*
 *	@brief : DHT11 sensörüne veri almak istediğini anlatmak üzere yapılandırılmıştır.
 */
static uint8_t StartSignalDHT11(void)
{
	uint32_t time = 0;
	setPinOutput();
	HAL_GPIO_WritePin(dht_port, dht_pin, GPIO_PIN_RESET);
	DHT11_Delay_us(18000);
	setPinInput();

	__HAL_TIM_SET_COUNTER(dht_timer,0);
	while(HAL_GPIO_ReadPin(dht_port, dht_pin) == GPIO_PIN_SET)
		{if(__HAL_TIM_GET_COUNTER(dht_timer) > 200) break; }
	time = __HAL_TIM_GET_COUNTER(dht_timer);
	if(time < 20) return 1;
	return 0;
}

/*
 *	@brief: DHT11 in veri yollamaya hazır olduğunun anlaşıldığı kısımdır.
		   (DataSheete göre 80us low ve 80us high olduğunda veri aktarılacaktır.)
		   Buradaki time ölçeğinin geniş olması kullnaılan sensörün klon olması durumundan kaynaklanır.
 */
static uint8_t IsReadyDHT11(void)
{
	uint32_t time = 0;
	if(StartSignalDHT11())
	{
		__HAL_TIM_SET_COUNTER(dht_timer,0);
		while(HAL_GPIO_ReadPin(dht_port, dht_pin) == GPIO_PIN_RESET)
		{if(__HAL_TIM_GET_COUNTER(dht_timer) > 120) break; }
		time = __HAL_TIM_GET_COUNTER(dht_timer);
		if(time < 90 && time > 70)						// datasheet e göre 80us LOW olmalı
		{
			__HAL_TIM_SET_COUNTER(dht_timer,0);
			while(HAL_GPIO_ReadPin(dht_port, dht_pin) == GPIO_PIN_SET)
			{if(__HAL_TIM_GET_COUNTER(dht_timer) > 120) break; }
			time = __HAL_TIM_GET_COUNTER(dht_timer);
			if(time < 94 && time > 68){return 1;}		// datasheet e göre 80us HIGH olmalı
		}
	}
	return 0;
}

/*
 	 brief: DataSheete göre DHT11 sensörü tarafından 50us low bekledikten sonra
 	 0 için 26-28us ve 1 için 70us HIGH da olması gerkmektedir.
 	 Bunun kontrolünün yapıldığı aşama.
*/
static uint8_t scaleDataDHT11(void)
{
	uint32_t time = 0;
	__HAL_TIM_SET_COUNTER(dht_timer,0);
	while(HAL_GPIO_ReadPin(dht_port, dht_pin) == GPIO_PIN_RESET)
	{if(__HAL_TIM_GET_COUNTER(dht_timer) > 100) break; }
	time = __HAL_TIM_GET_COUNTER(dht_timer);
	if(time > 35 && time < 65)                         // DataSheete Göre 50us Low kalması gerkmektedir.
	{
		__HAL_TIM_SET_COUNTER(dht_timer,0);
		while(HAL_GPIO_ReadPin(dht_port, dht_pin) == GPIO_PIN_SET)
		{if(__HAL_TIM_GET_COUNTER(dht_timer) > 100) break; }
		time = __HAL_TIM_GET_COUNTER(dht_timer);
		if(time > 14 && time < 40){return 0;}     // DataSheete Göre 26-28us  0
		if(time >55 && time < 86){return 1;}      // DataSheete Göre 70us     1
	}

	return 3;  // data alınamadı
}

/*
	@brief: Data nın okunduğu ve doğruluğunun checksum ile kontrol edildiği son adımdır.
 */
uint8_t DHT11_readData(DHT11_Data *dhtdata)
{
	HAL_TIM_Base_Start(dht_timer);
	HAL_Delay(800);   				 // Timerı etkili bir şekilde kullanabilmek için .

	if(IsReadyDHT11())
	{
		for(uint8_t i = 0; i < 40; i++)
		{
			dhtdata->databits[i] = scaleDataDHT11();
		}
		HAL_TIM_Base_Stop(dht_timer);
		// Atama işlemleri bittikten sonra Checksum bitleri ile karşılaştırma yapacağız.
		// RhData verileri için for (RH integral )
		for(uint8_t rh = 0; rh < 8; rh++)
		{ dhtdata->humidity_int |= (dhtdata->databits[rh] << (8-rh-1)); }
		// RhData verileri için for (RH decimal )
		for(uint8_t rh = 8; rh < 16; rh++)
		{ dhtdata->humidity_dec |= (dhtdata->databits[rh] << (16-rh-1)); }
		// Sıcaklık Derece (T integral) TData verileri için for
		for(uint8_t t = 16; t < 24; t++)
		{ dhtdata->temperature_int |= (dhtdata->databits[t] << (24-t-1)); }
		// Sıcaklık Derece (T decimal) TData verileri için
		for(uint8_t t = 24; t < 32; t++)
		{ dhtdata->temperature_dec |= (dhtdata->databits[t] << (32-t-1)); }
		for(uint8_t c = 32; c < 40; c++)
		{ dhtdata->checksum |= (dhtdata->databits[c] << (40-c -1)); }

		if((dhtdata->humidity_dec
			+ dhtdata->humidity_int
		    + dhtdata->temperature_dec
			+ dhtdata->temperature_int) == dhtdata->checksum) return 1;
		//control işlmeleri

	}
	HAL_TIM_Base_Stop(dht_timer);
	return 0;
}
/*
 	 Datanın doğru bir şekilde alınıp alınmadığını belirlemek adımı:
 	 uint8_t bool;
 	 bool = DHT11_readData(&dhtdata)

 	 -> bool değeri "1" ise data doğru bir şekilde alınmıştır.
 	 -> bool değeri "0" ise data yanlış bir şekilde alınmış yahut hiç data alınamamıştır..

 */
