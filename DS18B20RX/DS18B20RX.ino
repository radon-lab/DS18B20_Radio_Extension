/*
  Arduino IDE 1.8.13 версия прошивки RX 4.1.1 релиз от 10.03.23
  Частота мк приемника 9.6MHz microCore 2.2.0

  Автор Radon-lab.
*/
#include <util/delay.h>
#include <avr/pgmspace.h>

#define MAX_TIME 30 //максимальное время одного сеанса связи(1..60)(мин)
#define SLOW_MODE 1 //режим передачи данных(0 - быстрый | 1 - медленный)
#define OSCCAL_SET 0 //установка коррекции частоты(1..127)(0 - без коррекции)

#define WIRE_TIMEOUT 5 //тайм-аут ожидания команды шины 1wire(2..15)(1ед. ~1.7мс)
#define ADDR_CELL 15 //ячейка адреса передатчика(8..15)

#define BIT_SET(value, bit) ((value) |= (0x01 << (bit)))
#define BIT_CLEAR(value, bit) ((value) &= ~(0x01 << (bit)))

#define DDR_REG DDRB
#define PIN_REG PINB
#define PORT_REG PORTB

//пин кнопки программирования адреса PB3
#define ADDR_SET_BIT   3 // PB3
#define ADDR_SET_CHK   (PIN_REG & (0x01 << ADDR_SET_BIT))

//пин светодиода PB2
#define LED_BIT   2 // PB2
#define LED_ON    (BIT_SET(PORT_REG, LED_BIT))
#define LED_OFF   (BIT_CLEAR(PORT_REG, LED_BIT))

//пин шины oneWire PB1
#define WIRE_BIT   1 // PB1
#define WIRE_CHK  (PIN_REG & (0x01 << WIRE_BIT))
#define WIRE_LO   (BIT_SET(DDR_REG, WIRE_BIT))
#define WIRE_HI   (BIT_CLEAR(DDR_REG, WIRE_BIT))

//пин приемника PB0
#define RX_DATA_BIT   0 // PB0
#define RX_DATA_CHK   (PIN_REG & (0x01 << RX_DATA_BIT))

#define SEARCH_ROM 0xF0 //поиск адреса
#define MATCH_ROM 0x55  //отправка адреса
#define READ_ROM 0x33   //запрос адреса
#define SKIP_ROM 0xCC   //пропуск адреса

#define READ_DATA 0xBE  //отправка массива памяти

#define START_BIT_TIME 197 //время старт бита
#define STOP_BIT_TIME 141 //время стоп бита
#define PITCH_TIME 141 //время бита раскачки
#define LOW_BIT_TIME 85 //время бита 0
#define HIGH_BIT_TIME 30 //время бита 1

#define TICK_PER_WDT ((MAX_TIME * 60) / 8) //перевод минут в тики WDT

enum {
  WIRE_READ,
  WIRE_WRITE
};

uint16_t timeOutReceiveWaint; //счетчик тиков
uint8_t receiveTime; //счетчик импульса приёма
uint8_t receiveBits; //регистр принятых бит
uint8_t receiveAddr; //адрес передатчика
uint8_t receiveData[10]; //буфер приёмника

uint8_t wireMemory[9]; //память шины oneWire
const uint8_t wireReceiveError[] PROGMEM = {0xD0, 0x07, 0x4B, 0x46, 0x7F, 0xFF, 0x05, 0x10, 0x46}; //значение 125

int main(void) {
  cli(); //запрещаем прерывания глобально

#if OSCCAL_SET
  OSCCAL = OSCCAL_SET;
#endif

  PORT_REG = (0x01 << ADDR_SET_BIT); //включили подтяжку кнопки адреса
  DDR_REG = (0x01 << LED_BIT); //установили светодиод как выход

  TCCR0A = 0; //отключаем OC0A/OC0B
#if SLOW_MODE
  TCCR0B = (0x01 << CS02); //пределитель 256 | нормальный режим
#else
  TCCR0B = (0x01 << CS00 | 0x01 << CS01); //пределитель 64 | нормальный режим
#endif
  TIMSK0 = 0; //отключаем прерывания Таймера0

  PCMSK |= (0x01 << PCINT0); //настроили маску прерываний для PB0 радиоприемник
  MCUCR |= (0x01 << ISC01); //настроили маску прерываний для PB1 шина oneWire

  if (ADDR_SET_CHK) receiveAddr = EEPROM_read(ADDR_CELL); //прочитали адрес передатчика
  if (!receiveAddr) LED_ON; //если нет адреса то включаем индикацию

  receiveError(); //ошибка передатчика
  wdtEnable(); //включаем WDT
  //--------------------------------------------------------------------------------------
  for (;;) {
    if ((GIFR & (0x01 << INTF0)) || OCR0A) { //если увидели спад на шине 1wire
      oneWireCheck(); //сигнал протокола oneWire
      if (OCR0B) { //если был запрос на сигнал присутствия
        OCR0A = 1; //записали сигнал сброса
        OCR0B = 0; //сбросили запрос на сигнал присутствия
        _delay_us(15); //ждем
        WIRE_LO; //установили низкий уровень
        _delay_us(120); //ждем
        WIRE_HI; //установили высокий уровень
        _delay_us(2); //ждем
        GIFR = (0x01 << INTF0); //сбросили флаг прерывания пина PB1
      }
      else {
        OCR0A = 0; //сбросили сигнал сброса
#if SLOW_MODE
        TCCR0B = (0x01 << CS02); //пределитель 256 | нормальный режим
#endif
        TCNT0 = 0; //сбросили таймер
        TIFR0 = (0x01 << TOV0); //сбросили флаг прерывания таймера
      }
    }
    else { //иначе обработка радиоканала
      if (GIFR & (0x01 << PCIF)) { //если увидели сигнал передатчика
        if (TIFR0 & (0x01 << TOV0)) { //если был флаг переполнения таймера
          TIFR0 = (0x01 << TOV0); //сбросили флаг прерывания таймера
          receiveTime = 255; //переполнение
        }
        else receiveTime = TCNT0; //запомнили время

        TCNT0 = 0; //сбросили таймер
        GIFR = (0x01 << PCIF); //сбросили флаг прерывания пина

        if (!RX_DATA_CHK && receiveTime >= HIGH_BIT_TIME) { //если низкий уровень и длинна импульса больше минимальной
          receiveBits <<= 0x01; //сместили биты маски приема
          if (receiveTime >= START_BIT_TIME) { //если получили старт бит
            if (receiveBits == 0xFE) { //если получили все биты раскачки
              receiveCheck(); //читаем пакет данных
              if (receiveAddr) LED_OFF; //выключили светодиод
            }
            receiveBits = 0; //сбросили буфер раскачки
          }
          else if (receiveTime >= PITCH_TIME) receiveBits |= 0x01; //установли бит маски приема
        }
      }

      if (WDTCR & (0x01 << WDTIF)) { //если флаг переполнения WDT установлен
        WDTCR |= (0x01 << WDTIF); //сбрасываем флаг
        if (++timeOutReceiveWaint > TICK_PER_WDT) { //если максимальное время ожидания превышено
          timeOutReceiveWaint = 0; //сбрасываем таймер
          receiveError(); //ошибка передатчика
        }
      }
    }
  }
  return 0;
}
//------------------------------Ошибка передатчика----------------------------------------
void receiveError(void) //ошибка передатчика
{
  for (uint8_t i = 0; i < 9; i++) wireMemory[i] = pgm_read_byte(&wireReceiveError[i]); //записывае ошибку отсутствия сигнала от передатчика
}
//--------------------------------Включение WDT-------------------------------------------
void wdtEnable(void) //включение WDT
{
  MCUSR &= ~(0x01 << WDRF); //сбрасываем флаг сброса по watchdog
  WDTCR |= (0x01 << WDCE) | (0x01 << WDE); //разрешаем внесение изменений
  WDTCR = (0x01 << WDTIE) | (0x01 << WDP3) | (0x01 << WDP0); //устанавливаем собаку на 8сек
}
//------------------------------Эмуляция шины 1wire---------------------------------------
void oneWireCheck(void) //эмуляция шины 1wire
{
#if SLOW_MODE
  TCCR0B = (0x01 << CS00 | 0x01 << CS01); //пределитель 64 | нормальный режим
#endif
  while (1) { //ожидание команды
    switch (oneWireExchange(WIRE_READ, 0, 8)) { //читаем байт сетевого протокола
      case READ_ROM: //комманда отправить адрес
        for (uint8_t i = 0; i < 8; i++) if (oneWireExchange(WIRE_WRITE, EEPROM_read(i), 8)) return; //отправляем адрес на шину 1wire
        return; //выходим
      case MATCH_ROM: //комманда сравнить адрес
        for (uint8_t i = 0; i < 8; i++) if (oneWireExchange(WIRE_READ, 0, 8) != EEPROM_read(i)) return; //читаем адрес шины 1wire
        break; //продолжаем
      case SEARCH_ROM: //комманда поиска адреса
        for (uint8_t _byte = 0; _byte < 8; _byte++) { //перебираем байты адреса
          uint8_t _addr_byte = EEPROM_read(_byte); //находим нужный байт адреса
          for (uint8_t _bit = 0; _bit < 8; _bit++) { //перебираем биты адреса
            if (oneWireExchange(WIRE_WRITE, (_addr_byte & 0x01), 1)) return; //отправляем прямой бит
            if (oneWireExchange(WIRE_WRITE, !(_addr_byte & 0x01), 1)) return; //отправляем инверсный бит
            if ((boolean)oneWireExchange(WIRE_READ, 0, 1) != (_addr_byte & 0x01)) return; //отправка на шину 1wire
            _addr_byte >>= 0x01; //сместили байт
          }
        }
        return; //выходим
      case SKIP_ROM: break; //пропуск адресации
      case READ_DATA: //комманда отправить температуру
        for (uint8_t i = 0; i < sizeof(wireMemory); i++) if (oneWireExchange(WIRE_WRITE, wireMemory[i], 8)) return; //отправка на шину 1wire
        break;
      default: return; //если команда неизвестна то выходим
    }
  }
}
//----------------------------Обмен данными на шине 1wire------------------------------------------
uint8_t oneWireExchange(boolean mode, uint8_t data, uint8_t size) //обмен данными на шине 1wire
{
  uint8_t timer = WIRE_TIMEOUT; //буфер таймера

  if (OCR0B) return 0x01; //выходим если был повторный сигнал сброса
  while (size) { //читаем 8 бит
    if (GIFR & (0x01 << INTF0)) { //если был спад
      TCNT0 = 0; //сбросили таймер
      TIFR0 = (0x01 << TOV0); //сбросили флаг прерывания таймера
      timer = WIRE_TIMEOUT; //установили таймер

      if (mode == WIRE_WRITE) { //если режим записи
        if (!(data & 0x01)) WIRE_LO; //передаем 0
        _delay_us(30); //ждем
        WIRE_HI; //освобождаем линию
      }

      data >>= 1; //сместили байт
      GIFR = (0x01 << INTF0); //сбросили флаг прерывания пина PB1

      while (!WIRE_CHK) if (TIFR0 & (0x01 << TOV0)) return 0x01; //ждем окончания сигнала сброса
      if (TCNT0 >= 64) { //если сигнал сброса
        OCR0B = 1; //записали повторный сигнал сброса
        return 0x01; //выходим
      }
      else if (!OCR0A) return 0x01; //выходим если не было сигнал сброса

      if (mode == WIRE_READ) if (TCNT0 < 5) data |= 0x80; //установли единицу
      size--; //сместили бит чтенияан
    }
    if (TIFR0 & (0x01 << TOV0)) { //ждем флага прерывания
      TIFR0 = (0x01 << TOV0); //сбросили флаг прерывания таймера
      if (!(--timer)) return 0x01; //выходим
    }
  }
  return data; //возвращаем прочитаный байт
}
//--------------------------------------Чтение сигнал приемника------------------------------------------
void receiveCheck(void) //чтение сигнала приемника
{
  uint8_t _addr = 0; //флаг адреса
  for (uint8_t _byte = 0; _byte < 10;) { //счетчик принятых байт
    receiveData[_byte] = 0; //очищаем байт буфера приёма
    for (uint8_t _bit = 0; _bit < 8;) { //счетчик принятых бит
      while (!(GIFR & (0x01 << PCIF)) && !(TIFR0 & (0x01 << TOV0))); //ждем флага прерывания
      if (TIFR0 & (0x01 << TOV0)) { //если был флаг переполнения таймера
        TIFR0 = (0x01 << TOV0); //сбросили флаг прерывания таймера
        receiveTime = 255; //переполнение
      }
      else receiveTime = TCNT0; //запомнили время
      TCNT0 = 0; //сбросили таймер
      GIFR = (0x01 << PCIF); //сбросили флаг прерывания пина

      if (!RX_DATA_CHK && receiveTime >= HIGH_BIT_TIME) { //если обнаружили спад и длинна импульса больше минимальной
        _bit++; //добавили бит
        receiveData[_byte] >>= 0x01; //сместили байт
        if (receiveTime < LOW_BIT_TIME) receiveData[_byte] |= 0x80; //утанавливаем единицу в буфер
        else if (receiveTime >= STOP_BIT_TIME) { //иначе если был стоп бит
          if (!checkCRC(receiveData, _byte)) { //если контрольная сумма совпала
            for (uint8_t i = 0; i < _byte; i++) { //переписываем временный буфер в основной
              switch (_byte) { //в зависимости от количества принятых байт
                case 8: EEPROM_write(i, receiveData[i]); break; //обновляем массив адреса шины
                case 9: wireMemory[i] = receiveData[i]; break; //обновляем массив шины
              }
            }
          }
          if (_addr) timeOutReceiveWaint = 0; //сбрасываем таймер приема
          if (_addr == 2) EEPROM_write(ADDR_CELL, receiveAddr); //записали новый адрес передатчика
          return; //выходим если конец пакета
        }
      }
    }
    if (!_addr) { //если адрес не прочитан
      if (receiveData[0] != receiveAddr) { //если адрес не совпал
        if (receiveAddr) return; //если ячейка заполнена то выходим
        receiveAddr = receiveData[0]; //запомнили новый адрес передатчика
        _addr = 2; //установили флаг нового адреса
      }
      else _addr = 1; //установили флаг прочитанного адреса
      LED_ON; //включили светодиод
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
  while (EECR & (0x01 << EEPE)); //ждём завершения записи

  EECR = 0x00; //включаем режим стирание-запись
  EEARL = addr; //устанавливаем адрес
  EEDR = data; //загружаем данные

  EECR |= (0x01 << EEMPE); //запускаем запись
  EECR |= (0x01 << EEPE);
}
//--------------------------------------Чтение EEPROM------------------------------------------
uint8_t EEPROM_read(uint8_t addr) //чтение EEPROM
{
  while (EECR & (0x01 << EEPE)); //ждём завершения записи

  EEARL = addr; //устанавливаем адрес
  EECR |= (0x01 << EERE); //запускаем чтение

  return EEDR; //возвращаем данные
}
