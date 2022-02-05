/*
  Arduino IDE 1.8.13 версия прошивки TX 3.5.2 релиз от 04.02.22
  Частота мк передатчика 4.8MHz microCore 1.0.5

  Установка перемычек(0 - разомкнуто, 1 - замкнуто)
  1x - PB3(1) PB4(1)
  2x - PB3(0) PB4(1)
  4x - PB3(1) PB4(0)
  8x - PB3(0) PB4(0)

  Автор Radon-lab.
*/
#include <util/delay.h>
#include <avr/pgmspace.h>

#define ADDR 0x33   //адрес периёмника
#define MAX_TIME 60 //максимальный период одного сеанса связи(мин)
#define SLOW_MODE 1 //если наблюдаются перебои в передачи данных, установите 1

#define BIT_SET(value, bit) ((value) |= (0x01 << (bit)))
#define BIT_CLEAR(value, bit) ((value) &= ~(0x01 << (bit)))

#define DDR_REG DDRB
#define PIN_REG PINB
#define PORT_REG PORTB

//пин сенсора температуры PB2
#define WIRE_BIT   2 // PB2

#define WIRE_SET  (BIT_SET(PORT_REG, WIRE_BIT))
#define WIRE_CLR  (BIT_CLEAR(PORT_REG, WIRE_BIT))
#define WIRE_CHK  (PIN_REG & (0x01 << WIRE_BIT))
#define WIRE_LO   (BIT_SET(DDR_REG, WIRE_BIT))
#define WIRE_HI   (BIT_CLEAR(DDR_REG, WIRE_BIT))

#define WIRE_INIT  WIRE_CLR; WIRE_HI

//пин питания передатчика PB0
#define TX_POWER_BIT   0 // PB0

#define TX_POWER_ON    (BIT_SET(PORT_REG, TX_POWER_BIT))
#define TX_POWER_OFF   (BIT_CLEAR(PORT_REG, TX_POWER_BIT))
#define TX_POWER_OUT   (BIT_SET(DDR_REG, TX_POWER_BIT))

#define TX_POWER_INIT  TX_POWER_OFF; TX_POWER_OUT

//пин передатчика PB1
#define TX_DATA_BIT   1 // PB1

#define TX_DATA_HI    (BIT_SET(PORT_REG, TX_DATA_BIT))
#define TX_DATA_LO    (BIT_CLEAR(PORT_REG, TX_DATA_BIT))
#define TX_DATA_INV   (PORT_REG ^= (0x01 << TX_DATA_BIT))
#define TX_DATA_OUT   (BIT_SET(DDR_REG, TX_DATA_BIT))

#define TX_DATA_INIT  TX_DATA_LO; TX_DATA_OUT

#if SLOW_MODE
#define PITC_LENGTH 24
#define PITC_TIME 3000

#define START_BIT_TIME 4000
#define STOP_BIT_TIME 3000

#define HIGH_BIT_TIME 2000
#define LOW_BIT_TIME 1000
#else
#define PITC_LENGTH 48
#define PITC_TIME 750

#define START_BIT_TIME 1000
#define STOP_BIT_TIME 750

#define HIGH_BIT_TIME 500
#define LOW_BIT_TIME 250
#endif

#define TICK_PER_WDT ((MAX_TIME * 60) / 64) //рассчет минимального количества тиков

uint16_t timeOutTransceivWaint; //счетчик тиков начала передачи
uint16_t timeStartTransceiv; //время начала передачи
const uint8_t tempSensError[] PROGMEM = {0xB0, 0xFA, 0x4B, 0x46, 0x7F, 0xFF, 0x05, 0x10, 0xDB}; //значение -85

int main(void) {
  cli(); //запрещаем прерывания глобально

  WIRE_INIT; //инициализация датчика температуры
  TX_POWER_INIT; //инициализация питания передатчика
  TX_DATA_INIT; //инициализация передатчика

  PRR = (0x01 << PRADC); //выключаем АЦП

  DDRB &= ~(0x01 << PB3 | 0x01 << PB4); //устанавливаем PB3 и PB4 как входы
  PORTB |= (0x01 << PB3 | 0x01 << PB4); //устанавливаем подтяжку для PB3 и PB4

  timeStartTransceiv = timeOutTransceivWaint = ((uint16_t)TICK_PER_WDT << ((PINB >> 3) & 0x03)); //устанавливаем начальное значение таймера

  PORTB &= ~(0x01 << PB3 | 0x01 << PB4); //отключаем подтяжку для PB3 и PB4

  requestTemp(); //запрос на преобразование температуры
  _delay_ms(1500); //ждем

  sendAddrDS(); //отправка адреса датчика
  wdtEnable(); //включаем WDT

  sei(); //разрешаем прерывания глобально
  //--------------------------------------------------------------------------------------
  for (;;) {
    if (++timeOutTransceivWaint > timeStartTransceiv) sendDataDS(); //отправляем температуру
    else if (timeOutTransceivWaint == (timeStartTransceiv - 1)) requestTemp(); //запрос на преобразование температуры
    if (timeOutTransceivWaint > timeStartTransceiv) timeOutTransceivWaint = 0; //сбрасываем счетчик
    sleep(); //спим
  }
  return 0;
}
EMPTY_INTERRUPT(WDT_vect); //прерывание WDT
//-------------------------------------Сон---------------------------------------------
void sleep(void)
{
  MCUCR |= (0x01 << SM1) | (0x01 << SE); //устанавливаем режим сна powerdown
  BODCR = (0x01 << BODS) | (0x01 << BODSE); //разрешаем вносить изменения
  BODCR = (0x01 << BODS); //отключаем БОД
  asm ("sleep"); //с этого момента спим.
}
//-------------------------------------Включение WDT---------------------------------------------
void wdtEnable(void) //включение WDT
{
  MCUSR &= ~(0x01 << WDRF); //сбрасываем флаг сброса по watchdog
  WDTCR |= (0x01 << WDCE) | (0x01 << WDE); //разрешаем внесение изменений
  WDTCR = (0x01 << WDTIE) | (0x01 << WDP3) | (0x01 << WDP0); //устанавливаем режим прерываний каждые 8сек
}
//-----------------------------------Сигнал сброса шины--------------------------------------------
boolean oneWireReset(void)
{
  WIRE_LO; //устанавливаем низкий уровень
  _delay_us(520); //ждем
  WIRE_HI; //устанавливаем высокий уровень
  _delay_us(2); //ждем
  for (uint8_t c = 80; c; c--) { //ожидание установки низкого уровня
    if (!WIRE_CHK) { //если видим низкий уровень
      for (uint8_t i = 200; !WIRE_CHK && i; i--) _delay_us(1); //ждем
      return 0; //возвращаем статус ок
    }
    _delay_us(1); //ждем
  }
  return 1; //возвращаем ошибку
}
//----------------------------------Отправка данных в шину-----------------------------------------
void oneWireWrite(uint8_t data)
{
  for (uint8_t i = 0; i < 8; i++) { //отправляем 8 бит на шину
    if ((data >> i) & 0x01) { //если единица
      WIRE_LO; //устанавливаем низкий уровень
      _delay_us(5); //ждем
      WIRE_HI; //устанавливаем высокий уровень
      _delay_us(60); //ждем
    }
    else { //иначе ноль
      WIRE_LO; //устанавливаем низкий уровень
      _delay_us(60); //ждем
      WIRE_HI; //устанавливаем высокий уровень
      _delay_us(5); //ждем
    }
  }
}
//-----------------------------------------Чтение шины--------------------------------------------
uint8_t oneWireRead(void)
{
  uint8_t data = 0; //временный буфер
  for (uint8_t i = 0; i < 8; i++) { //читаем 8 бит с шины
    WIRE_LO; //устанавливаем низкий уровень
    _delay_us(2); //ждем
    WIRE_HI; //устанавливаем высокий уровень
    _delay_us(8); //ждем
    if (WIRE_CHK) data |= (0x01 << i); //читаем состояние шины
    _delay_us(60); //ждем
  }
  return data; //возвращаем принятый байт
}
//---------------------------------------Запрос температуры-----------------------------------------
void requestTemp(void) //запрос температуры
{
  if (oneWireReset()) return; //посылаем сигнал сброса
  oneWireWrite(0xCC); //пропуск адресации
  oneWireWrite(0x44); //запрос на преобразование температуры
}
//------------------------------------Отправка адреса датчика----------------------------------------
void sendAddrDS(void) //отправка адреса датчика
{
  uint8_t dataRaw[9]; //временный буфер

  dataRaw[0] = ADDR; //установили адрес
  if (oneWireReset()) return; //посылаем сигнал сброса
  oneWireWrite(0x33); //запрос на отправку адреса
  for (uint8_t i = 1; i < 9; i++) dataRaw[i] = oneWireRead(); //читаем 8 байт адреса
  sendDataTX(dataRaw, sizeof(dataRaw)); //оправляем 8 байт адреса
}
//--------------------------------------Отправка температуры------------------------------------------
void sendDataDS(void) //отправка температуры
{
  uint8_t dataRaw[10]; //временный буфер

  dataRaw[0] = ADDR; //установили адрес
  if (oneWireReset()) { //посылаем сигнал сброса
    for (uint8_t i = 1; i < 10; i++) dataRaw[i] = pgm_read_byte(&tempSensError[i - 1]); //значение отсутсвуещего датчика
  }
  else {
    oneWireWrite(0xCC); //пропуск адресации
    oneWireWrite(0xBE); //запрос на отправку температуры
    for (uint8_t i = 1; i < 10; i++) dataRaw[i] = oneWireRead(); //читаем 9 байт памяти
  }

  for (uint8_t i = 0; i < 3; i++) { //отправляем 3 пакета 
    _delay_ms(50); //ждем
    sendDataTX(dataRaw, sizeof(dataRaw)); //оправляем 9 байт памяти + адрес
  }
}
//--------------------------------------Отправка данных------------------------------------------
void sendDataTX(uint8_t* data, uint8_t size)
{
  TX_POWER_ON; //включили питание передатчика

  for (uint8_t i = 0; i < PITC_LENGTH; i++) { //предварительная модуляция сигнала
    _delay_us(PITC_TIME); //ждем
    TX_DATA_INV; //ивертируем состояние
  }

  TX_DATA_LO; //устанавливаем низкий уровень
  _delay_us(START_BIT_TIME); //ждем
  TX_DATA_HI; //устанавливаем высокий уровень
  _delay_us(START_BIT_TIME); //ждем
  TX_DATA_LO; //устанавливаем низкий уровень - старт бит
  _delay_us(LOW_BIT_TIME); //ждем
  TX_DATA_HI; //устанавливаем высокий уровень

  for (uint8_t _byte = 0; _byte < size; _byte++) { //передаем пакет
    for (uint8_t _bit = 0; _bit < 8; _bit++) { //передаем байт
      if (data[_byte] & 0x01) { //передаем 1
        _delay_us(HIGH_BIT_TIME); //ждем
        TX_DATA_LO; //устанавливаем низкий уровень
        _delay_us(LOW_BIT_TIME); //ждем
      }
      else { //передаем 0
        _delay_us(LOW_BIT_TIME); //ждем
        TX_DATA_LO; //устанавливаем низкий уровень
        _delay_us(HIGH_BIT_TIME); //ждем
      }
      data[_byte] >>= 0x01; //сместили байт
      TX_DATA_HI; //устанавливаем высокий уровень
    }
  }

  _delay_us(STOP_BIT_TIME); //ждем
  TX_DATA_LO; //устанавливаем низкий уровень - стоп бит

  TX_POWER_OFF; //выключаем питание передатчика
}
