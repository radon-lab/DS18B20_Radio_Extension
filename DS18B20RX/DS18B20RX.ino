/*
  Arduino IDE 1.8.13 версия прошивки RX 3.3.2 релиз от 02.11.21

  Автор Radon-lab.
*/
#include <util/delay.h>

#define BIT_SET(value, bit) ((value) |= (0x01 << (bit)))
#define BIT_CLEAR(value, bit) ((value) &= ~(0x01 << (bit)))

#define DDR_REG DDRB
#define PIN_REG PINB
#define PORT_REG PORTB

#define SEARCH_ROM 0xF0 //поиск адреса
#define MATCH_ROM 0x55  //отправка адреса
#define READ_ROM 0x33   //запрос адреса
#define SKIP_ROM 0xCC   //пропуск адреса

#define READ_DATA 0xBE  //отправка массива памяти

//пин светодиода PB2
#define LED_BIT   2 // PB2

#define LED_ON    (BIT_SET(PORT_REG, LED_BIT))
#define LED_OFF   (BIT_CLEAR(PORT_REG, LED_BIT))
#define LED_OUT   (BIT_SET(DDR_REG, LED_BIT))

#define LED_INIT  LED_OFF; LED_OUT

//пин шины oneWire PB1
#define WIRE_BIT   1 // PB1

#define WIRE_SET  (BIT_SET(PORT_REG, WIRE_BIT))
#define WIRE_CLR  (BIT_CLEAR(PORT_REG, WIRE_BIT))
#define WIRE_CHK  (PIN_REG & (0x01 << WIRE_BIT))
#define WIRE_LO   (BIT_SET(DDR_REG, WIRE_BIT))
#define WIRE_HI   (BIT_CLEAR(DDR_REG, WIRE_BIT))

#define WIRE_INIT  WIRE_CLR; WIRE_HI

//пин приемника PB0
#define RX_DATA_BIT   0 // PB0

#define RX_DATA_HI    (BIT_SET(PORT_REG, RX_DATA_BIT))
#define RX_DATA_LO    (BIT_CLEAR(PORT_REG, RX_DATA_BIT))
#define RX_DATA_INP   (BIT_CLEAR(DDR_REG, RX_DATA_BIT))

#define RX_DATA_INIT  RX_DATA_LO; RX_DATA_INP

uint16_t timeOutReceiveWaint; //счетчик тиков
uint8_t wireDataBuf[9]; //буфер шины oneWire
uint8_t wireAddrBuf[8]; //буфер адреса шины oneWire
const uint8_t wireDataError[] = {0xD0, 0x07, 0x4B, 0x46, 0x7F, 0xFF, 0x05, 0x10, 0x46}; //значение 125

int main(void) {
  cli(); //запрещаем прерывания глобально

  WIRE_INIT; //инициализация датчика температуры
  LED_INIT; //инициализация светодиода
  RX_DATA_INIT; //инициализация приемника данных

  PRR = (0x01 << PRADC); //выключаем АЦП

  TCCR0A = 0; //отключаем OC0A/OC0B
  TCCR0B = (0x01 << CS00 | 0x01 << CS01); //пределитель 64 | нормальный режим
  TIMSK0 = 0; //отключаем прерывания Таймера0

  PCMSK |= (0x01 << PCINT0); //настроили маску прерываний для PB0 радиоприемник
  MCUCR |= (0x01 << ISC01); //настроили маску прерываний для PB1 шина oneWire

  for (uint8_t i = 0; i < 9; i++) wireDataBuf[i] = wireDataError[i]; //записывае ошибку отсутствия сигнала от передатчика
  for (uint8_t i = 0; i < 8; i++) wireAddrBuf[i] = EEPROM_read(i); //читаем адрес из памяти

  wdtEnable(); //включаем WDT
  //--------------------------------------------------------------------------------------
  for (;;) {
    switch (GIFR & 0x60) {
      case 0x40: readOneWire(); break; //сигнал протокола oneWire
      case 0x60: //спорная ситуация - отдаем приоретет передатчику
      case 0x20: readDataRX(); LED_OFF; break; //сигнал передатчика
      default: //нет сигнала с датчика температуры
        if (WDTCR &  (0x01 << WDTIF)) { //если флаг переполнения WDT установлен
          WDTCR |= (0x01 << WDTIF); //сбрасываем флаг
          if (++timeOutReceiveWaint > 520) { //если максимальное время ожидания превышено
            timeOutReceiveWaint = 0; //сбрасываем таймер
            for (uint8_t i = 0; i < 9; i++) wireDataBuf[i] = wireDataError[i]; //записывае ошибку отсутствия сигнала от передатчика
          }
        }
        break;
    }
  }
  return 0;
}
//-------------------------------------Включение WDT---------------------------------------------
void wdtEnable(void) //включение WDT
{
  MCUSR &= ~(0x01 << WDRF); //сбрасываем флаг сброса по watchdog
  WDTCR |= (0x01 << WDCE) | (0x01 << WDE); //разрешаем внесение изменений
  WDTCR = (0x01 << WDTIE) | (0x01 << WDP3) | (0x01 << WDP0); //устанавливаем собаку на 8сек
}
//------------------------------Эмуляция шины 1wire---------------------------------------
void readOneWire(void) //эмуляция шины 1wire
{
  TCNT0 = 0; //сбросили таймер
  TIFR0 |= (0x01 << TOV0); //сбросили флаг прерывания таймера
  GIFR |= (0x01 << INTF0); //сбросили флаг прерывания пина PB1

  while (!WIRE_CHK) if (TIFR0 & (0x01 << TOV0)) return; //ждем окончания сигнала сброса
  if (TCNT0 < 64) return; //если сигнал сброса слишком короткий

  TCNT0 = 0; //сбросили таймер

  _delay_us(2); //ждем
  WIRE_LO; //установили низкий уровень
  _delay_us(120); //ждем
  GIFR |= (0x01 << INTF0); //сбросили флаг прерывания пина PB1
  WIRE_HI; //установили высокий уровень
  _delay_us(2); //ждем

  TCNT0 = 0; //сбросили таймер
  TIFR0 |= (0x01 << TOV0); //сбросили флаг прерывания таймера

  switch (oneWireRead()) { //читаем байт сетевого протокола
    case READ_ROM: //комманда отправить адрес
      for (uint8_t i = 0; i < sizeof(wireAddrBuf); i++) if (oneWireWrite(wireAddrBuf[i])) return; //отправляем адрес на шину 1wire
      return; //выходим
    case MATCH_ROM: //комманда сравнить адрес
      for (uint8_t i = 0; i < sizeof(wireAddrBuf); i++) if (oneWireRead() != wireAddrBuf[i]) return; //читаем адрес шины 1wire
      break; //выходим
    //      case SEARCH_ROM: //комманда поиска адреса
    //        for (uint8_t i = 0; i < 64; i++) {
    //          boolean addrBit = wireAddrBuf[i >> 3] & (0x01 << (i % 8)); //находим нужный бит адреса
    //          oneWireWriteBit(addrBit); //отправляем прямой бит
    //          oneWireWriteBit(!addrBit); //отправляем инверсный бит
    //          if (oneWireReadBit() != addrBit) return; //отправка на шину 1wire
    //        }
    //        return; //выходим
    case SKIP_ROM: break; //пропуск адресации
  }

  switch (oneWireRead()) { //читаем байт команды
    case READ_DATA: //комманда отправить температуру
      for (uint8_t i = 0; i < sizeof(wireDataBuf); i++) if (oneWireWrite(wireDataBuf[i])) return; //отправка на шину 1wire
      break;
  }
}
//-----------------------------------Отправка на шину 1wire----------------------------------------
boolean oneWireWrite(uint8_t data) //отправка на шину 1wire
{
  for (uint8_t i = 0; i < 8;) { //отправляем 8 бит
    if (TIFR0 & (0x01 << TOV0)) return 1; //ждем флага прерывания
    if (GIFR & (0x01 << INTF0)) { //если был спад
      TCNT0 = 0; //сбросили таймер
      if ((data >> i) & 0x01) WIRE_HI; //передаем 1
      else WIRE_LO; //передаем 0
      while (TCNT0 < 5); //ждем
      WIRE_HI; //освобождаем линию
      GIFR |= (0x01 << INTF0); //сбросили флаг прерывания пина PB1
      i++; //сместили бит передачи
    }
  }
  return 0;
}
//--------------------------------------Чтение шины 1wire------------------------------------------
uint8_t oneWireRead(void) //чтение шины 1wire
{
  uint8_t data = 0; //временный буфер приема
  for (uint8_t i = 0; i < 8;) { //читаем 8 бит
    if (TIFR0 & (0x01 << TOV0)) return 0; //ждем флага прерывания
    if (GIFR & (0x01 << INTF0)) { //если был спад
      TCNT0 = 0; //сбросили таймер
      GIFR |= (0x01 << INTF0); //сбросили флаг прерывания пина PB1
      while (!WIRE_CHK) if (TIFR0 & (0x01 << TOV0)) return 0; //ждем флага прерывания
      if (TCNT0 < 5) data |= 0x01 << i; //установли единицу
      i++; //сместили бит чтения
    }
  }
  return data; //возвращаем прочитаный байт
}
//-----------------------------------Отправка бита на шину 1wire----------------------------------------
void oneWireWriteBit(uint8_t data) //отправка бита на шину 1wire
{
  while (!(GIFR & (0x01 << INTF0))) if (TIFR0 & (0x01 << TOV0)) return; //ждем флага прерывания
  TCNT0 = 0; //сбросили таймер
  if (data) WIRE_HI; //передаем 1
  else WIRE_LO; //передаем 0
  while (TCNT0 < 5); //ждем
  WIRE_HI; //освобождаем линию
  GIFR |= (0x01 << INTF0); //сбросили флаг прерывания пина PB1
}
//--------------------------------------Чтение бита шины 1wire------------------------------------------
uint8_t oneWireReadBit(void) //чтение бита шины 1wire
{
  while (!(GIFR & (0x01 << INTF0))) if (TIFR0 & (0x01 << TOV0)) return 0; //ждем флага прерывания
  TCNT0 = 0; //сбросили таймер
  GIFR |= (0x01 << INTF0); //сбросили флаг прерывания пина PB1
  while (!WIRE_CHK) if (TIFR0 & (0x01 << TOV0)) return 0; //ждем флага прерывания
  if (TCNT0 < 5) return 1; //возвращаем единицу

  return 0; //возвращаем ноль
}
//--------------------------------------Чтение сигнал приемника------------------------------------------
void readDataRX(void) //чтение сигнала приемника
{
  boolean dataMode = 0; //режим приема данных
  uint8_t bitPulse = 0; //длинна ипульса
  uint8_t bitNum = 0; //номер бита
  uint8_t byteNum = 0; //номер байта

  uint8_t data[] = {0, 0, 0, 0, 0, 0, 0, 0, 0}; //опустошаем буфер передатчика

  TCNT0 = 0; //сбросили таймер
  TIFR0 |= (0x01 << TOV0); //сбросили флаг прерывания таймера
  GIFR |= (0x01 << PCIF); //сбросили флаг прерывания пина

  while (1) {
    while (!(GIFR & (0x01 << PCIF))) if (TIFR0 & (0x01 << TOV0)) return; //ждем флага прерывания
    bitPulse = TCNT0; //запомнили длинну импульса
    TCNT0 = 0; //сбросили таймер
    GIFR |= (0x01 << PCIF); //сбросили флаг прерывания пина

    switch (dataMode) {
      case 0:
        if (bitPulse > 132) { //если получили старт бит
          LED_ON; //включили светодиод
          dataMode = 1; //переходим в режим чтения
        }
        else if (bitPulse < 100) return; //иначе выходим
        break;
      case 1:
        if (bitPulse > 100) { //если был стоп бит
          if (!checkCRC(data, byteNum)) { //если контрольная сумма совпала
            for (uint8_t i = 0; i < byteNum; i++) {
              switch (byteNum) { //в зависимости от количества принятых байт
                case 8: wireAddrBuf[i] = data[i]; EEPROM_write(i, data[i]); break; //обновляем массив адреса шины
                case 9: wireDataBuf[i] = data[i]; break; //обновляем массив шины
              }
            }
          }
          timeOutReceiveWaint = 0; //сбрасываем таймер приема
          return; //выходим если конец пакета
        }
        else if (bitPulse > 56) data[byteNum] |= 0x01 << bitNum; //иначе утанавливаем единицу в буфер

        if (++bitNum > 7) { //если приняли 8 бит
          bitNum = 0; //сбрасываем счетчик бит
          byteNum++; //прибавляем счетчик байт
        }
        break;
    }
  }
}
//--------------------------------------Контроль CRC с датчика температуры------------------------------------------
uint8_t checkCRC(uint8_t* data, uint8_t size) //контроль CRC с датчика температуры
{
  uint8_t crcData = 0; //буфер проверки crc
  for (uint8_t i = 0; i < size; i++) { //проверяем все байты
    uint8_t counter; //счетчик циклов
    uint8_t buffer; //буфер
    asm volatile (
      "EOR %[crc_out], %[data_in] \n\t"
      "LDI %[counter], 8          \n\t"
      "LDI %[buffer], 0x8C        \n\t"
      "_loop_start_%=:            \n\t"
      "LSR %[crc_out]             \n\t"
      "BRCC _loop_end_%=          \n\t"
      "EOR %[crc_out], %[buffer]  \n\t"
      "_loop_end_%=:              \n\t"
      "DEC %[counter]             \n\t"
      "BRNE _loop_start_%="
      : [crc_out]"=r" (crcData), [counter]"=d" (counter), [buffer]"=d" (buffer)
      : [crc_in]"0" (crcData), [data_in]"r" (data[i])
    );
  }
  return crcData;
}
//--------------------------------------Запись EEPROM------------------------------------------
void EEPROM_write(uint8_t addr, uint8_t data) //запись EEPROM
{
  while (EECR & (0x01 << EEPE)); //ждём завершения записи

  EECR |= (0x01 << EEPM1); //включаем режим стирание-запись
  EEARL = addr; //устанавливаем адрес
  EEDR = data; //загружаем данные

  EECR |= (1 << EEMPE); //запускаем запись
  EECR |= (1 << EEPE);
}
//--------------------------------------Чтение EEPROM------------------------------------------
uint8_t EEPROM_read(uint8_t addr) //чтение EEPROM
{
  while (EECR & (0x01 << EEPE)); //ждём завершения записи

  EEARL = addr; //устанавливаем адрес
  EECR |= (0x01 << EERE); //запускаем чтение

  return EEDR; //возвращаем данные
}
