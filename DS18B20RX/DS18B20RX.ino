/*
  Arduino IDE 1.8.13 версия прошивки RX 3.5.2 релиз от 04.02.22
  Частота мк приемника 9.6MHz microCore 2.1.0

  Автор Radon-lab.
*/
#include <util/delay.h>
#include <avr/pgmspace.h>

#define MAX_TIME 60 //максимальное время одного сеанса связи(мин)
#define SLOW_MODE 1 //если наблюдаются перебои в передачи данных, установите 1

#define ADDR_CELL 16 //ячейка адреса передатчика

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

//пин кнопки программирования адреса PB3
#define ADDR_SET_BIT   3 // PB3

#define ADDR_SET_HI    (BIT_SET(PORT_REG, ADDR_SET_BIT))
#define ADDR_SET_LO    (BIT_CLEAR(PORT_REG, ADDR_SET_BIT))
#define ADDR_SET_CHK   (PIN_REG & (0x01 << ADDR_SET_BIT))
#define ADDR_SET_INP   (BIT_CLEAR(DDR_REG, ADDR_SET_BIT))

#define ADDR_SET_INIT  ADDR_SET_HI; ADDR_SET_INP

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
#define RX_DATA_CHK   (PIN_REG & (0x01 << RX_DATA_BIT))
#define RX_DATA_INP   (BIT_CLEAR(DDR_REG, RX_DATA_BIT))

#define RX_DATA_INIT  RX_DATA_LO; RX_DATA_INP

#define TICK_PER_WDT ((MAX_TIME * 60) / 8) //перевод минут в тики WDT

uint16_t timeOutReceiveWaint; //счетчик тиков
uint8_t receiveTime; //счетчик импульса приёма
uint8_t receiveBits; //регистр принятых бит
uint8_t receiveData[10]; //буфер приёмника

uint8_t wireMemory[9]; //память шины oneWire
const uint8_t wireReceiveError[] PROGMEM = {0xD0, 0x07, 0x4B, 0x46, 0x7F, 0xFF, 0x05, 0x10, 0x46}; //значение 125

int main(void) {
  cli(); //запрещаем прерывания глобально

  WIRE_INIT; //инициализация датчика температуры
  LED_INIT; //инициализация светодиода
  RX_DATA_INIT; //инициализация приемника данных
  ADDR_SET_INIT; //инициализация кнопки программирования адреса

  TCCR0A = 0; //отключаем OC0A/OC0B
#if SLOW_MODE
  TCCR0B = (0x01 << CS02); //пределитель 256 | нормальный режим
#else
  TCCR0B = (0x01 << CS00 | 0x01 << CS01); //пределитель 64 | нормальный режим
#endif
  TIMSK0 = 0; //отключаем прерывания Таймера0

  PCMSK |= (0x01 << PCINT0); //настроили маску прерываний для PB0 радиоприемник
  MCUCR |= (0x01 << ISC01); //настроили маску прерываний для PB1 шина oneWire

  if (!ADDR_SET_CHK) { //если зажата кнопка программирования адреса
    LED_ON; //включили светодиод
    EEPROM_write(ADDR_CELL, 0); //удалили адрес передатчика
  }

  for (uint8_t i = 0; i < 9; i++) wireMemory[i] = pgm_read_byte(&wireReceiveError[i]); //записывае ошибку отсутствия сигнала от передатчика

  wdtEnable(); //включаем WDT
  //--------------------------------------------------------------------------------------
  for (;;) {
    if (GIFR & (0x01 << INTF0)) { //если увидели спад на шине 1wire
#if SLOW_MODE
      TCCR0B = (0x01 << CS00 | 0x01 << CS01); //пределитель 64 | нормальный режим
#endif
      TCNT0 = 0; //сбросили таймер
      readOneWire(); //сигнал протокола oneWire
#if SLOW_MODE
      TCCR0B = (0x01 << CS02); //пределитель 256 | нормальный режим
#endif
      TCNT0 = 0; //сбросили таймер
      TIFR0 |= (0x01 << TOV0); //сбросили флаг прерывания таймера
    }

    if (GIFR & (0x01 << PCIF)) { //если увидели сигнал передатчика
      GIFR |= (0x01 << PCIF); //сбросили флаг прерывания пина
      if (TIFR0 & (0x01 << TOV0)) { //если был флаг переполнения таймера
        TIFR0 |= (0x01 << TOV0); //сбросили флаг прерывания таймера
        receiveTime = 255; //переполнение
      }
      else receiveTime = TCNT0; //запомнили время
      TCNT0 = 0; //сбросили таймер

      if (!RX_DATA_CHK && receiveTime >= 95) { //если низкий уровень и длинна импульса больше минимальной
        receiveBits <<= 0x01; //сместили биты маски приема
        if (receiveTime < 132) receiveBits |= 0x01; //установли бит маски приема
        else if (receiveTime >= 132 && receiveTime < 170 && receiveBits == 0xFE) { //если получили старт бит
          receiveBits = 0; //сбросили буфер раскачки
          readDataRX(); //читаем пакет данных
          LED_OFF; //выключили светодиод
        }
      }
    }

    if (WDTCR &  (0x01 << WDTIF)) { //если флаг переполнения WDT установлен
      WDTCR |= (0x01 << WDTIF); //сбрасываем флаг
      if (++timeOutReceiveWaint > TICK_PER_WDT) { //если максимальное время ожидания превышено
        timeOutReceiveWaint = 0; //сбрасываем таймер
        for (uint8_t i = 0; i < 9; i++) wireMemory[i] = pgm_read_byte(&wireReceiveError[i]); //записывае ошибку отсутствия сигнала от передатчика
      }
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
  TIFR0 |= (0x01 << TOV0); //сбросили флаг прерывания таймера
  GIFR |= (0x01 << INTF0); //сбросили флаг прерывания пина PB1

  while (!WIRE_CHK) if (TIFR0 & (0x01 << TOV0)) return; //ждем окончания сигнала сброса
  if (TCNT0 < 64) return; //если сигнал сброса слишком короткий

  _delay_us(2); //ждем
  WIRE_LO; //установили низкий уровень
  _delay_us(120); //ждем
  GIFR |= (0x01 << INTF0); //сбросили флаг прерывания пина PB1
  WIRE_HI; //установили высокий уровень
  _delay_us(2); //ждем

  TCNT0 = 0; //сбросили таймер
  TIFR0 |= (0x01 << TOV0); //сбросили флаг прерывания таймера

  switch (oneWireRead(8)) { //читаем байт сетевого протокола
    case READ_ROM: //комманда отправить адрес
      for (uint8_t i = 0; i < 8; i++) if (oneWireWrite(EEPROM_read(i), 8)) return; //отправляем адрес на шину 1wire
      return; //выходим
    case MATCH_ROM: //комманда сравнить адрес
      for (uint8_t i = 0; i < 8; i++) if (oneWireRead(8) != EEPROM_read(i)) return; //читаем адрес шины 1wire
      break; //продолжаем
    case SEARCH_ROM: //комманда поиска адреса
      for (uint8_t _byte = 0; _byte < 8; _byte++) { //перебираем байты адреса
        uint8_t _addr_byte = EEPROM_read(_byte); //находим нужный байт адреса
        for (uint8_t _bit = 0; _bit < 8; _bit++) { //перебираем биты адреса
          oneWireWrite((_addr_byte & 0x01), 1); //отправляем прямой бит
          oneWireWrite(!(_addr_byte & 0x01), 1); //отправляем инверсный бит
          if ((boolean)oneWireRead(1) != (_addr_byte & 0x01)) return; //отправка на шину 1wire
          _addr_byte >>= 0x01; //сместили байт
        }
      }
      return; //выходим
    case SKIP_ROM: break; //пропуск адресации
    default: return; //если команда неизвестна то выходим
  }

  switch (oneWireRead(8)) { //читаем байт команды
    case READ_DATA: //комманда отправить температуру
      for (uint8_t i = 0; i < sizeof(wireMemory); i++) if (oneWireWrite(wireMemory[i], 8)) return; //отправка на шину 1wire
      break;
  }
}
//-----------------------------------Отправка на шину 1wire----------------------------------------
boolean oneWireWrite(uint8_t data, uint8_t size) //отправка на шину 1wire
{
  for (uint8_t i = 0; i < size;) { //отправляем 8 бит
    if (TIFR0 & (0x01 << TOV0)) return 1; //ждем флага прерывания
    if (GIFR & (0x01 << INTF0)) { //если был спад
      TCNT0 = 0; //сбросили таймер
      if (data & 0x01) WIRE_HI; //передаем 1
      else WIRE_LO; //передаем 0
      while (TCNT0 < 5); //ждем
      WIRE_HI; //освобождаем линию
      GIFR |= (0x01 << INTF0); //сбросили флаг прерывания пина PB1
      data >>= 0x01; //сместили байт
      i++; //сместили бит передачи
    }
  }
  return 0;
}
//--------------------------------------Чтение шины 1wire------------------------------------------
uint8_t oneWireRead(uint8_t size) //чтение шины 1wire
{
  uint8_t data = 0; //временный буфер приема
  for (uint8_t i = 0; i < size;) { //читаем 8 бит
    if (TIFR0 & (0x01 << TOV0)) return 0; //ждем флага прерывания
    if (GIFR & (0x01 << INTF0)) { //если был спад
      TCNT0 = 0; //сбросили таймер
      GIFR |= (0x01 << INTF0); //сбросили флаг прерывания пина PB1
      data >>= 0x01; //сместили байт
      while (!WIRE_CHK) if (TIFR0 & (0x01 << TOV0)) return 0; //ждем флага прерывания
      if (TCNT0 < 5) data |= 0x80; //установли единицу
      i++; //сместили бит чтения
    }
  }
  return data; //возвращаем прочитаный байт
}
//--------------------------------------Чтение сигнал приемника------------------------------------------
void readDataRX(void) //чтение сигнала приемника
{
  boolean _addr = 0; //флаг адреса
  for (uint8_t _byte = 0; _byte < 10;) { //счетчик принятых байт
    receiveData[_byte] = 0; //очищаем байт буфера приёма
    for (uint8_t _bit = 0; _bit < 8;) { //счетчик принятых бит
      while (!(GIFR & (0x01 << PCIF))) if (TIFR0 & (0x01 << TOV0)) return; //ждем флага прерывания
      receiveTime = TCNT0; //запомнили длинну импульса
      TCNT0 = 0; //сбросили таймер
      GIFR |= (0x01 << PCIF); //сбросили флаг прерывания пина

      if (!RX_DATA_CHK && receiveTime >= 25) { //если обнаружили спад и длинна импульса больше минимальной
        _bit++; //добавили бит
        receiveData[_byte] >>= 0x01; //сместили байт
        if (receiveTime >= 56 && receiveTime < 95) receiveData[_byte] |= 0x80; //утанавливаем единицу в буфер
        else if (receiveTime >= 95) { //иначе если был стоп бит
          if (!checkCRC(receiveData, _byte)) { //если контрольная сумма совпала
            for (uint8_t i = 0; i < _byte; i++) { //переписываем временный буфер в основной
              switch (_byte) { //в зависимости от количества принятых байт
                case 8: EEPROM_write(i, receiveData[i]); break; //обновляем массив адреса шины
                case 9: wireMemory[i] = receiveData[i]; break; //обновляем массив шины
              }
            }
          }
          timeOutReceiveWaint = 0; //сбрасываем таймер приема
          return; //выходим если конец пакета
        }
      }
    }
    if (!_addr) { //если адрес не прочитан
      if (receiveData[0] != EEPROM_read(ADDR_CELL)) { //если адрес не совпал
        if (!EEPROM_read(ADDR_CELL)) EEPROM_write(ADDR_CELL, receiveData[0]); //если ячейка сброшена то записываем новый адрес
        else return; // выходим
      }
      LED_ON; //включили светодиод
      _addr = 1; //установили флаг прочитанного адреса
    }
    else _byte++; //иначе прибавили байт буфера
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
  return crcData; //возвращаем результат
}
//--------------------------------------Запись EEPROM------------------------------------------
void EEPROM_write(uint8_t addr, uint8_t data) //запись EEPROM
{
  if (EEPROM_read(addr) != data) {
    while (EECR & (0x01 << EEPE)); //ждём завершения записи

    EECR |= (0x01 << EEPM1); //включаем режим стирание-запись
    EEARL = addr; //устанавливаем адрес
    EEDR = data; //загружаем данные

    EECR |= (1 << EEMPE); //запускаем запись
    EECR |= (1 << EEPE);
  }
}
//--------------------------------------Чтение EEPROM------------------------------------------
uint8_t EEPROM_read(uint8_t addr) //чтение EEPROM
{
  while (EECR & (0x01 << EEPE)); //ждём завершения записи

  EEARL = addr; //устанавливаем адрес
  EECR |= (0x01 << EERE); //запускаем чтение

  return EEDR; //возвращаем данные
}
