# <p align="center">Радиоудленнитель DS18B20 на Attiny13a.</p> 

- Что реализовано:
  - Полноценная шина 1wire с возможностью адресации(команды - Search ROM, Read ROM, Match ROM, Skip ROM).
  - Копирование памяти и адреса датчика(адрес хранится в EEPROM приемника).
  - Защита от помех(передача бит по высокому уровню + проверка сигнала раскачки).
  - Система ошибок(125 - передатчик давено не выходил на связь, -85 - нет сигнала от датчика температуры).
  - Возможность установки перемычками пределителя интервалов передачи(1x, 2x, 4x, 8x).
  - Возможность перед компиляцией установить свой максимальный интервал передачи(по умолчанию 60мин).
  - Дублирование передачи(отправка повторного пакета через 8 секунд после основной передачи).
  - Запрос температуры(за 8 секунд до основной передачи).
  
- Синхронизация адреса датчика с приёмником:
  - Выключить передатчик и приёмник.
  - Отключить приемник от шины 1wire.
  - Включить приёмник.
  - Включить передатчик.
  - Через 1,5сек светодиоды приёмника и передатчика должны загорется - адрес прописан.
  
- Установка перемычек(0 - разомкнуто, 1 - замкнуто):
  - 1x - PB3(1) PB4(1).
  - 2x - PB3(0) PB4(1).
  - 4x - PB3(1) PB4(0).
  - 8x - PB3(0) PB4(0).