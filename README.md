# TeknofestBackupFlightSoftware
Teknofest Roket Yarışması yedek uçuş bilgisayarı kodudur. Yedek uçuş bilgisayarı (Arduino Uno), ana uçuş bilgisayarı (STM32F103C8T6), Neo-6M ve GY-BMP280 kullanılmıştır.
## Bağlantı Şeması
UNO Ground -> GY-BMP280 Ground  
UNO 5V     -> GY-BMP280 Vcc  
UNO A4     -> GY-BMP280 SDA  
UNO A5     -> GY-BMP280 SCL  
UNO PIN8   -> STM32F103C8T6 PB10  
UNO PIN9   -> STM32F103C8T6 PB11  
UNO PIN2   -> NEO 6M TX  
UNO PIN3   -> NEO 6M RX  
UNO PIN4   -> MG995_0 PWM  
UNO PIN5   -> MG995_1 PWM  
UNO PIN6   -> MG995_2 PWM  
UNO PIN7   -> MG995_3 PWM  
