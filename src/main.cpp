#include <Arduino.h>
#include <HardwareSerial.h>
#include "driver/uart.h"  // Для uart_set_line_inverse, uart_set_mode

// Піни для S-Port (half-duplex, інвертований)
#define SPORT_PIN 10          // Один пін для TX+RX

// Піни для медіаконвертора (full-duplex, нормальний)
#define MEDIA_RX 16
#define MEDIA_TX 17

// Підтримувані швидкості
#define BAUD_HIGH 400000
#define BAUD_LOW 115200

// UART порти
HardwareSerial SportSerial(1);    // UART1 для S-Port
HardwareSerial MediaSerial(2);    // UART2 для медіаконвертора

// Буфери для даних
uint8_t buffer[256];

// Поточна швидкість
uint32_t currentBaud = 0;

// Налаштування S-Port з певною швидкістю
void configureSPort(uint32_t baud) {
  SportSerial.end();  // Закрити попередній порт
  delay(100);
  
  SportSerial.begin(baud, SERIAL_8N1, SPORT_PIN, SPORT_PIN);
  uart_set_line_inverse(UART_NUM_1, UART_SIGNAL_TXD_INV | UART_SIGNAL_RXD_INV);
  uart_set_mode(UART_NUM_1, UART_MODE_RS485_HALF_DUPLEX);
  
  currentBaud = baud;
  Serial.printf("S-Port configured: %d baud\n", baud);
}

// Автоматичне визначення швидкості S-Port
uint32_t detectBaudRate() {
  Serial.println("Starting auto-baud detection...");
  
  // Спочатку пробуємо високу швидкість (400k)
  Serial.println("Trying 400000 baud...");
  configureSPort(BAUD_HIGH);
  
  unsigned long startTime = millis();
  int bytesReceived = 0;
  
  // Чекаємо дані протягом 2 секунд
  while (millis() - startTime < 2000) {
    if (SportSerial.available()) {
      bytesReceived += SportSerial.available();
      SportSerial.read();  // Просто очищаємо буфер
    }
    delay(10);
  }
  
  Serial.printf("Received %d bytes at 400000 baud\n", bytesReceived);
  
  // Якщо отримали дані - використовуємо цю швидкість
  if (bytesReceived > 10) {
    Serial.println("✓ Detected 400000 baud");
    configureSPort(BAUD_HIGH);  // Реконфігуруємо для очистки буферів
    return BAUD_HIGH;
  }
  
  // Якщо немає даних - пробуємо низьку швидкість (115200)
  Serial.println("No data at 400k, trying 115200 baud...");
  configureSPort(BAUD_LOW);
  
  startTime = millis();
  bytesReceived = 0;
  
  // Чекаємо дані протягом 2 секунд
  while (millis() - startTime < 2000) {
    if (SportSerial.available()) {
      bytesReceived += SportSerial.available();
      SportSerial.read();
    }
    delay(10);
  }
  
  Serial.printf("Received %d bytes at 115200 baud\n", bytesReceived);
  
  if (bytesReceived > 10) {
    Serial.println("✓ Detected 115200 baud");
    configureSPort(BAUD_LOW);
    return BAUD_LOW;
  }
  
  // Якщо нічого не знайдено - використовуємо 400k за замовчуванням
  Serial.println("⚠ No data detected, defaulting to 400000 baud");
  configureSPort(BAUD_HIGH);
  return BAUD_HIGH;
}

void setup() {
  // Ініціалізація Serial для дебагу (опціонально)
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n\n=================================");
  Serial.println("S-Port <-> UART Bridge starting...");
  Serial.println("=================================\n");

  // Автоматичне визначення швидкості S-Port
  currentBaud = detectBaudRate();

  // Налаштування медіаконвертора з тією ж швидкістю
  MediaSerial.begin(currentBaud, SERIAL_8N1, MEDIA_RX, MEDIA_TX);
  
  Serial.printf("\n✓ Media UART configured: RX=%d, TX=%d, %d baud\n", MEDIA_RX, MEDIA_TX, currentBaud);
  Serial.println("✓ Bridge ready!\n");
  Serial.println("=================================\n");
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