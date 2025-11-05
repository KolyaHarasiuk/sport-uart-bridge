#include <Arduino.h>
#include <HardwareSerial.h>
#include "driver/uart.h"  // Для uart_set_line_inverse, uart_set_mode

// Піни для S-Port (half-duplex, інвертований)
#define SPORT_PIN 10          // Один пін для TX+RX
#define SPORT_BAUD 420000

// Піни для медіаконвертора (full-duplex, нормальний)
#define MEDIA_RX 16
#define MEDIA_TX 17
#define MEDIA_BAUD 400000

// UART порти
HardwareSerial SportSerial(1);    // UART1 для S-Port
HardwareSerial MediaSerial(2);    // UART2 для медіаконвертора

// Буфери для даних
uint8_t buffer[256];

void setup() {
  // Ініціалізація Serial для дебагу (опціонально)
  Serial.begin(115200);
  delay(1000);
  Serial.println("S-Port <-> UART Bridge starting...");

  // Налаштування S-Port (UART1) - half-duplex, інвертований
  // Використовуємо один пін для TX і RX
  SportSerial.begin(SPORT_BAUD, SERIAL_8N1, SPORT_PIN, SPORT_PIN);
  
  // Встановлюємо інверсію сигналу для S-Port
  // Потрібно використати низькорівневий API ESP-IDF
  uart_set_line_inverse(UART_NUM_1, UART_SIGNAL_TXD_INV | UART_SIGNAL_RXD_INV);
  
  // Встановлюємо half-duplex режим (опціонально, але краще)
  uart_set_mode(UART_NUM_1, UART_MODE_RS485_HALF_DUPLEX);
  
  Serial.println("S-Port configured: GPIO 10, 420000 baud, inverted, half-duplex");

  // Налаштування медіаконвертора (UART2) - full-duplex, нормальний
  MediaSerial.begin(MEDIA_BAUD, SERIAL_8N1, MEDIA_RX, MEDIA_TX);
  
  Serial.println("Media UART configured: RX=16, TX=17, 420000 baud");
  Serial.println("Bridge ready!");
}

void loop() {
  // S-Port -> Media (апаратура -> медіаконвертор)
  if (SportSerial.available()) {
    int len = SportSerial.available();
    if (len > sizeof(buffer)) len = sizeof(buffer);
    
    int bytesRead = SportSerial.readBytes(buffer, len);
    if (bytesRead > 0) {
      MediaSerial.write(buffer, bytesRead);
      
      // Дебаг (можна закоментувати для продакшн)
      // Serial.printf("S->M: %d bytes\n", bytesRead);
    }
  }

  // Media -> S-Port (медіаконвертор -> апаратура або ELRS)
  if (MediaSerial.available()) {
    int len = MediaSerial.available();
    if (len > sizeof(buffer)) len = sizeof(buffer);
    
    int bytesRead = MediaSerial.readBytes(buffer, len);
    if (bytesRead > 0) {
      SportSerial.write(buffer, bytesRead);
      
      // Дебаг (можна закоментувати для продакшн)
      // Serial.printf("M->S: %d bytes\n", bytesRead);
    }
  }

  // Невеликий delay щоб не молотити CPU
  // Можна прибрати якщо потрібна максимальна швидкість
  // delayMicroseconds(100);
}