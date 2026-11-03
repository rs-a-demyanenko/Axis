<h1 align="center">Axis</h1>

<p align="center">Управление осью станка с обратной связью по энкодеру</p>

<p align="center">
  <img src="https://img.shields.io/badge/version-0.1.0-blue" alt="Версия 0.1.0">
  <img src="https://img.shields.io/badge/Arduino-compatible-00979D" alt="Arduino">
  <img src="https://img.shields.io/badge/ESP32-supported-333333" alt="ESP32">
  <img src="https://img.shields.io/badge/C%2B%2B-11-00599C" alt="C++11">
</p>

<p align="center">
  <a href="docs/GUIDE.md">Руководство</a> ·
  <a href="docs/GUIDE.md#api">API</a> ·
  <a href="examples">Примеры</a> ·
  <a href="tests/VALIDATION.md">Проверки</a>
</p>

**Axis** — библиотека Arduino для линейных и вращательных осей с коллекторным двигателем. Задайте точку и скорость — библиотека построит профиль движения, замкнёт контуры позиции и скорости и проверит ограничения. Управление драйвером основано на [GyverMotor2](https://github.com/GyverLibs/GyverMotor2).

## Возможности

- **Движение к точке** — абсолютное и относительное, с разгоном и торможением.
- **Обратная связь** — линейный или вращательный энкодер, декодирование A/B ×4 либо внешний счётчик.
- **Регулирование** — каскад позиции и скорости, PID и удержание точки.
- **Привязка Home** — поиск датчика, медленный повторный подход и парковка.
- **Ограничения** — MIN/MAX, программные границы и контролируемый отход от концевика.
- **Вращательные оси** — накопленная координата, угол и выбор направления поворота.

## Установка

**Arduino IDE:** установите **GyverMotor2** через менеджер библиотек, затем добавьте Axis через ZIP или поместите каталог в `Documents/Arduino/libraries/Axis`.

<details>
<summary><b>PlatformIO</b></summary>

Добавьте в `platformio.ini`:

```ini
lib_deps =
    https://github.com/rs-a-demyanenko/Axis.git
```

</details>

## Быстрый старт

Откройте [пример для Arduino Uno/Nano](examples/Linear_AVR/Linear_AVR.ino) или [ESP32](examples/Linear_ESP32/Linear_ESP32.ino). В них уже есть подключение энкодера, концевиков и команды через Serial. Настройте пины, масштаб энкодера и параметры своей механики.

Основные вызовы для настроенного объекта `machineAxis`:

```cpp
machineAxis.tick();           // регулярно в loop()

machineAxis.home();           // запустить привязку по команде оператора

// После успешного homing — команды по одной, по завершении предыдущей:
machineAxis.moveTo(50, 10);    // к координате 50 мм со скоростью до 10 мм/с
machineAxis.moveBy(-5, 2);     // на 5 мм назад со скоростью до 2 мм/с

machineAxis.stop();           // отменить движение и плавно остановиться
```

`home()` и движения выполняются неблокирующе. Проверяйте результат принятия команды, а завершение — через `isBusy()`, `isHomed()` и `getCompletion()`. Абсолютное движение разрешено после успешной привязки.

## Примеры

| Пример | Что показывает |
|---|---|
| [Linear_AVR](examples/Linear_AVR/Linear_AVR.ino) | Линейная ось, Uno/Nano, рабочий ноль и управление через Serial |
| [Linear_ESP32](examples/Linear_ESP32/Linear_ESP32.ino) | Линейная ось и атомарное чтение энкодера на ESP32 |
| [Rotary_ESP32](examples/Rotary_ESP32/Rotary_ESP32.ino) | Углы, выбор направления и многооборотное движение |
| [ExternalFeedback](examples/ExternalFeedback/ExternalFeedback.ino) | Адаптер внешнего счётчика и диагностики |

## Документация

[Полное руководство](docs/GUIDE.md) — настройки, масштаб энкодера, драйверы, homing, API и настройка регуляторов. [Отчёт о проверках](tests/VALIDATION.md) — сценарии и результаты сборок.

Код рассчитан на Arduino с C++11; проверены сборки для **Uno, ESP32 и ESP32-S3**. На реальном оборудовании библиотека пока не испытывалась — коэффициенты регуляторов и пределы движения нужно настроить под свою механику.
