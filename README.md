

# DHT11 Sensörü için Kütüphane
## CubeMx Yapılandırması
- 1 adet Timer Seçilmeli.
    - Bu timer için Timer_clock = 1 Mhz olmalı(us biriminde çalıştığımız için)
- 1 adet GPIO OUTPUT pini
   
#### DHT11.h dosyasındaki `#include "stm32f1xx_hal.h"` bu satırı eğer STM32F1xx kullanmıyorsanız ilgili dosya ile değiştiriniz.
---
>[DHT11Datasheet](https://www.alldatasheet.com/datasheet-pdf/view/2193416/OSEPP/DHT11.html)
---
Sensör **STM32F103C8T6** ile test edilmiştir.
