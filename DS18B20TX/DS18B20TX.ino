/*
  Arduino IDE 1.8.13 версия прошивки TX 3.3.2 релиз от 04.11.21

  Автор Radon-lab.
*/
#include <util/delay.h>

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

uint16_t timeOutTransceivWaint; //счетчик тиков начала передачи
uint16_t timeStartTransceiv; //время начала передачи
const uint8_t tempSensError[] = {0xB0, 0xFA, 0x4B, 0x46, 0x7F, 0xFF, 0x05, 0x10, 0xDB}; //значение -85

int main(void) {
  cli(); //запрещаем прерывания глобально

  WIRE_INIT; //инициализация датчика температуры
  TX_POWER_INIT; //инициализация питания передатчика
  TX_DATA_INIT; //инициализация передатчика

  PRR = (0x01 << PRADC); //выключаем АЦП

  DDRB &= ~(0x01 << PB3 | 0x01 << PB4); //устанавливаем PB3 и PB4 входы
  PORTB |= (0x01 << PB3 | 0x01 << PB4); //устанавливаем подтяжку для PB3 и PB4

  timeStartTransceiv = timeOutTransceivWaint = ((uint16_t)0x40 << ((PINB >> 3) & 0x03)); //устанавливаем начальное значение таймера

  _delay_ms(1500); //ждем

  sendAddrDS(); //отправка адреса датчика
  requestTemp(); //запрос на преобразование температуры
  wdtEnable(); //включаем WDT

  sei(); //разрешаем прерывания глобально
  //--------------------------------------------------------------------------------------
  for (;;) {
    sleep(); //спим
    if (++timeOutTransceivWaint >= timeStartTransceiv) { //если пришло время подать сигнал
      timeOutTransceivWaint = 0; //сбрасываем счетчик
      sendDataDS(); //отправляем температуру
    }
    else if (timeOutTransceivWaint == (timeStartTransceiv - 1)) requestTemp(); //запрос на преобразование температуры
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
  uint8_t dataRaw[8]; //временный буфер

  if (oneWireReset()) return; //посылаем сигнал сброса
  oneWireWrite(0x33); //запрос на отправку адреса
  for (uint8_t i = 0; i < 8; i++) dataRaw[i] = oneWireRead(); //читаем 8 байт адреса
  sendDataTX(dataRaw, sizeof(dataRaw)); //оправляем 8 байт адреса
}
//--------------------------------------Отправка температуры------------------------------------------
void sendDataDS(void) //отправка температуры
{
  uint8_t dataRaw[9]; //временный буфер

  if (oneWireReset()) { //посылаем сигнал сброса
    for (uint8_t i = 0; i < 9; i++) dataRaw[i] = tempSensError[i]; //значение отсутсвуещего датчика
  }
  else {
    oneWireWrite(0xCC); //пропуск адресации
    oneWireWrite(0xBE); //запрос на отправку температуры
    for (uint8_t i = 0; i < 9; i++) dataRaw[i] = oneWireRead(); //читаем 9 байт памяти
  }

  sendDataTX(dataRaw, sizeof(dataRaw)); //оправляем 9 байт памяти
}
//--------------------------------------Отправка данных------------------------------------------
void sendDataTX(uint8_t* data, uint8_t size)
{
  TX_POWER_ON; //включили питание передатчика

  for (uint8_t i = 0; i < 48; i++) { //предварительная модуляция сигнала
    _delay_us(750); //ждем
    TX_DATA_INV; //ивертируем состояние
  }

  TX_DATA_LO; //устанавливаем низкий уровень
  _delay_us(1000); //ждем
  TX_DATA_HI; //передаем старт бит

  for (uint8_t b = 0; b < size; b++) { //передаем пакет
    for (uint8_t i = 0; i < 8; i++) { //передаем байт
      if ((data[b] >> i) & 0x01) _delay_us(500); //передаем 1
      else _delay_us(250); //передаем 0
      TX_DATA_INV; //ивертируем состояние
    }
  }

  _delay_us(750); //ждем
  TX_DATA_INV; //ивертируем состояние - стоп бит

  TX_POWER_OFF; //выключаем питание передатчика
}
