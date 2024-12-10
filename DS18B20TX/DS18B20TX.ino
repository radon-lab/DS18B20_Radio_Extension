/*
  Arduino IDE 1.8.13 версия прошивки TX 4.1.2 релиз от 30.08.24
  Частота мк передатчика 4.8MHz microCore 2.2.0

  Установка перемычек:
  JMP1 - 1мин(GND)..30мин(VCC)
  JMP2 - 0xAA(GND)..0xCC(VCC)

  Автор Radon-lab.
*/
#include <util/delay.h>
#include <avr/pgmspace.h>

#define SLOW_MODE 1 //режим передачи данных(0 - быстрый | 1 - медленный)
#define OSCCAL_SET 0 //установка коррекции частоты(1..127)(0 - без коррекции)
#define RESOLUTION_SET 3 //установка разрешения датчика температуры(0 - 9бит | 1 - 10бит | 2 - 11бит | 3 - 12бит)

#define TX_CHANNEL_A 0xAA //адрес канала A(0x01..0xFF)
#define TX_CHANNEL_B 0xBB //адрес канала B(0x01..0xFF)
#define TX_CHANNEL_C 0xEE //адрес канала C(0x01..0xFF)
#define TX_CHANNEL_D 0xCC //адрес канала D(0x01..0xFF)

#define TX_TIME_A 1  //интервал передачи A(1..60)(мин)
#define TX_TIME_B 5  //интервал передачи B(1..60)(мин)
#define TX_TIME_C 10 //интервал передачи C(1..60)(мин)
#define TX_TIME_D 30 //интервал передачи D(1..60)(мин)

#define BIT_SET(value, bit) ((value) |= (0x01 << (bit)))
#define BIT_CLEAR(value, bit) ((value) &= ~(0x01 << (bit)))

#define DDR_REG DDRB
#define PIN_REG PINB
#define PORT_REG PORTB

//пин сенсора температуры PB2
#define WIRE_BIT   2 //PB2

#define WIRE_SET  (BIT_SET(PORT_REG, WIRE_BIT))
#define WIRE_CLR  (BIT_CLEAR(PORT_REG, WIRE_BIT))
#define WIRE_CHK  (PIN_REG & (0x01 << WIRE_BIT))
#define WIRE_LO   (BIT_SET(DDR_REG, WIRE_BIT))
#define WIRE_HI   (BIT_CLEAR(DDR_REG, WIRE_BIT))

#define WIRE_INIT  WIRE_CLR; WIRE_HI

//пин питания передатчика PB0
#define TX_POWER_BIT   0 //PB0

#define TX_POWER_ON    (BIT_SET(PORT_REG, TX_POWER_BIT))
#define TX_POWER_OFF   (BIT_CLEAR(PORT_REG, TX_POWER_BIT))
#define TX_POWER_OUT   (BIT_SET(DDR_REG, TX_POWER_BIT))

#define TX_POWER_INIT  TX_POWER_OFF; TX_POWER_OUT

//пин передатчика PB1
#define TX_DATA_BIT   1 //PB1

#define TX_DATA_HI    (BIT_SET(PORT_REG, TX_DATA_BIT))
#define TX_DATA_LO    (BIT_CLEAR(PORT_REG, TX_DATA_BIT))
#define TX_DATA_INV   (PORT_REG ^= (0x01 << TX_DATA_BIT))
#define TX_DATA_OUT   (BIT_SET(DDR_REG, TX_DATA_BIT))

#define TX_DATA_INIT  TX_DATA_LO; TX_DATA_OUT

#if SLOW_MODE
#define PITC_LENGTH 16
#define PITC_TIME 4500

#define START_BIT_TIME 6000
#define STOP_BIT_TIME 4500

#define LOW_BIT_TIME 3000
#define HIGH_BIT_TIME 1500
#else
#define PITC_LENGTH 32
#define PITC_TIME 1125

#define START_BIT_TIME 1500
#define STOP_BIT_TIME 1125

#define LOW_BIT_TIME 750
#define HIGH_BIT_TIME 375
#endif

#define CONVERT_TIME(x) ((x * 60) / 8) //рассчет количества тиков

const uint8_t tempSensError[] PROGMEM = {0xB0, 0xFA, 0x4B, 0x46, 0x7F, 0xFF, 0x05, 0x10, 0xDB}; //значение ошибки -85

const uint16_t transceivTime[] PROGMEM = {CONVERT_TIME(TX_TIME_A), CONVERT_TIME(TX_TIME_B), CONVERT_TIME(TX_TIME_C), CONVERT_TIME(TX_TIME_D)}; //массив времени передачи
const uint8_t transceivAddr[] PROGMEM = {TX_CHANNEL_A, TX_CHANNEL_B, TX_CHANNEL_C, TX_CHANNEL_D}; //массив адресов датчика
const uint8_t transceivCorrect[] PROGMEM = {2, 3, 4, 6}; //массив коррекции времени передачи

uint16_t timeStartTransceiv; //счетчик времени первой передачи
uint8_t timeEndTransceiv; //счетчик времени последней передачи

uint16_t _current_time;
uint8_t _current_addr;
uint8_t _current_correct;

int main(void) {
  cli(); //запрещаем прерывания глобально

#if OSCCAL_SET
  OSCCAL = OSCCAL_SET;
#endif

  WIRE_INIT; //инициализация датчика температуры
  TX_POWER_INIT; //инициализация питания передатчика
  TX_DATA_INIT; //инициализация передатчика

  DDRB &= ~(0x01 << PB3 | 0x01 << PB4); //устанавливаем PB3 и PB4 как входы
  PORTB |= (0x01 << PB3 | 0x01 << PB4); //устанавливаем подтяжку для PB3 и PB4

  ADCSRA = (0x01 << ADEN) | (0x01 << ADPS0) | (0x01 << ADPS1) | (0x01 << ADPS2); //настройка АЦП

  ADMUX = 3; //настройка мультиплексатора АЦП на PB3
  _delay_ms(15); //ждем
  ADCSRA |= (0x01 << ADSC); //запускаем преобразование
  while (ADCSRA & (0x01 << ADSC)); //ждем окончания преобразования
  _current_addr = pgm_read_byte(&transceivAddr[ADCH]); //установили адрес передатчика
  _current_correct = pgm_read_byte(&transceivCorrect[ADCH]); //установили коррекцию времени передачи

  ADMUX = 2; //настройка мультиплексатора АЦП на PB4
  _delay_ms(15); //ждем
  ADCSRA |= (0x01 << ADSC); //запускаем преобразование
  while (ADCSRA & (0x01 << ADSC)); //ждем окончания преобразования
  _current_time = timeStartTransceiv = pgm_read_word(&transceivTime[ADCH]); //устанавливаем начальное и максимальное значение таймера

  PORTB &= ~(0x01 << PB3 | 0x01 << PB4); //отключаем подтяжку для PB3 и PB4
  
  ADCSRA = 0x00; //выключаем АЦП
  PRR = (0x01 << PRADC); //выключаем питание АЦП
  
#if RESOLUTION_SET < 3
  setResolution(); //установка разрешения датчика
#endif
  requestTemp(); //запрос на преобразование температуры
  _delay_ms(1000); //ждем

  sendAddrDS(); //отправка адреса датчика
  _delay_ms(500); //ждем

  wdtEnable_1s(); //включение WDT 1сек

  sei(); //разрешаем прерывания глобально
  //--------------------------------------------------------------------------------------
  for (;;) {
    if (!timeEndTransceiv) { //если время коррекции окончено
      if (++timeStartTransceiv >= _current_time) { //если пришло время отправить температуру
        sendDataDS(); //отправляем температуру
        timeEndTransceiv = _current_correct; //установили время коррекции
      }
      else if (timeStartTransceiv == (_current_time - 1)) { //если пришло время запросиить температуру
        requestTemp(); //запрос на преобразование температуры
        wdtEnable_1s(); //включение WDT 1сек
        timeEndTransceiv = 8 - _current_correct; //установили время коррекции
      }
      if (timeStartTransceiv > _current_time) { //если конец сеанса связи
        wdtEnable_8s(); //включение WDT 8сек
        timeEndTransceiv = 0; //сбросили время коррекции
        timeStartTransceiv = 0; //сбрасываем счетчик
      }
    }
    else timeEndTransceiv--; //убавляен время коррекции
    sleep(); //спим
  }
  return 0;
}
EMPTY_INTERRUPT(WDT_vect); //прерывание WDT
//--------------------------------------------Сон--------------------------------------------------
void sleep(void)
{
  MCUCR |= (0x01 << SM1) | (0x01 << SE); //устанавливаем режим сна powerdown
  BODCR = (0x01 << BODS) | (0x01 << BODSE); //разрешаем вносить изменения
  BODCR = (0x01 << BODS); //отключаем БОД
  asm ("sleep"); //вошли в режим сна
}
//-----------------------------------Включение WDT 1сек--------------------------------------------
void wdtEnable_1s(void) //включение WDT 1сек
{
  asm ("wdr"); //сбросили таймер watchdog
  MCUSR &= ~(0x01 << WDRF); //сбрасываем флаг сброса по watchdog
  WDTCR |= (0x01 << WDCE) | (0x01 << WDE); //разрешаем внесение изменений
  WDTCR = (0x01 << WDTIE) | (0x01 << WDP2) | (0x01 << WDP1); //устанавливаем режим прерываний каждую 1сек
}
//-----------------------------------Включение WDT 8сек--------------------------------------------
void wdtEnable_8s(void) //включение WDT 8сек
{
  asm ("wdr"); //сбросили таймер watchdog
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
//-----------------------------------Установка разрешения датчика-----------------------------------
void setResolution(void)
{
  if (oneWireReset()) return; //посылаем сигнал сброса
  oneWireWrite(0xCC); //пропуск адресации
  oneWireWrite(0x4E); //запись в память
  oneWireWrite(0xFF); //устанавливаем разрешение
  oneWireWrite(0x00);
  oneWireWrite(0x1F | RESOLUTION_SET);
}
//------------------------------------Отправка адреса датчика----------------------------------------
void sendAddrDS(void) //отправка адреса датчика
{
  uint8_t dataRaw[9]; //временный буфер

  dataRaw[0] = _current_addr; //установили адрес
  if (oneWireReset()) return; //посылаем сигнал сброса
  oneWireWrite(0x33); //запрос на отправку адреса
  for (uint8_t i = 1; i < 9; i++) dataRaw[i] = oneWireRead(); //читаем 8 байт адреса
  sendDataTX(dataRaw, sizeof(dataRaw)); //оправляем 8 байт адреса
}
//--------------------------------------Отправка температуры------------------------------------------
void sendDataDS(void) //отправка температуры
{
  uint8_t dataRaw[10]; //временный буфер

  dataRaw[0] = _current_addr; //установили адрес
  if (oneWireReset()) { //посылаем сигнал сброса
    for (uint8_t i = 1; i < 10; i++) dataRaw[i] = pgm_read_byte(&tempSensError[i - 1]); //значение отсутсвуещего датчика
  }
  else {
    oneWireWrite(0xCC); //пропуск адресации
    oneWireWrite(0xBE); //запрос на отправку температуры
    for (uint8_t i = 1; i < 10; i++) dataRaw[i] = oneWireRead(); //читаем 9 байт памяти
  }

  sendDataTX(dataRaw, sizeof(dataRaw)); //оправляем 9 байт памяти + адрес
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
