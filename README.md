<h1 align="center">Axis</h1>

<p align="center">Управление осью станка: коллекторные и шаговые двигатели</p>

<p align="center">
  <img src="https://img.shields.io/badge/version-0.1.0-blue" alt="Версия 0.1.0">
  <img src="https://img.shields.io/badge/Arduino-compatible-00979D" alt="Arduino">
  <img src="https://img.shields.io/badge/ESP32-supported-333333" alt="ESP32">
  <img src="https://img.shields.io/badge/C%2B%2B-11-00599C" alt="C++11">
</p>

<p align="center">
  <a href="docs/GUIDE.md">Руководство</a> ·
  <a href="docs/GUIDE.md#api">API</a> ·
  <a href="docs/SYNCHRONIZATION.md">Синхронизация</a> ·
  <a href="examples">Примеры</a> ·
  <a href="tests/VALIDATION.md">Проверки</a>
</p>

**Axis** — библиотека Arduino для линейных и вращательных осей с коллекторным или шаговым двигателем. Задайте точку и скорость — библиотека построит профиль движения и проверит ограничения. Коллекторные двигатели управляются через [GyverMotor2](https://github.com/GyverLibs/GyverMotor2) с энкодером; шаговые — через [StepperCore из GyverStepper](https://github.com/GyverLibs/GyverStepper/blob/main/src/StepperCore.h), с энкодером или подсчётом выданных шагов.

## Возможности

- **S-профиль движения** — абсолютные и относительные команды с ограничением скорости, ускорения и рывка.
- **Обратная связь** — линейный или вращательный энкодер, декодирование A/B ×4 либо внешний счётчик.
- **Шаговые двигатели** — STEP/DIR, четыре фазы, полушаг и виртуальный драйвер. Масштабы шагов и энкодера задаются отдельно.
- **Регулирование** — каскад позиции и скорости, 2-DOF PID с весами задания и удержание точки.
- **Привязка Home** — поиск датчика, медленный повторный подход и парковка.
- **Ограничения** — MIN/MAX, программные границы и контролируемый отход от концевика.
- **Вращательные оси** — накопленная координата, угол и выбор направления поворота.
- **Синхронизация** — общий профиль нескольких осей, электронная передача и подача по энкодеру шпинделя.

## Установка

**Arduino IDE:** установите **GyverMotor2** и **GyverStepper** через менеджер библиотек, затем добавьте Axis через ZIP или поместите каталог в `Documents/Arduino/libraries/Axis`.

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

Плавность и реакция на задание настраиваются перед `begin()`:

```cpp
config.maxJerk = 300;                  // мм/с³; для вращательной оси — °/с³
config.velocitySetpointWeight = 0.8;   // вес задания в P-составляющей
config.velocityDerivativeWeight = 0;  // D-составляющая по измерению
```

S-образный разгон создаёт профиль с ограничением рывка. Для просмотра кривых без двигателя откройте [SCurvePlotter](examples/SCurvePlotter/SCurvePlotter.ino) и Serial Plotter на 115200 бод. [Подробнее о профиле и регуляторе](docs/GUIDE.md#s-профиль-и-ограничение-рывка).

## Шаговые двигатели

```cpp
#include <AxisStepper.h>
AxisStepper<STEPPER2WIRE> x(5, 6, 7);  // STEP, DIR, EN

// В setup(), до begin():
// x.setStepsPerUnit(640);            // 200 шагов * 16 микрошагов / 5 мм
// x.attachEncoder(readEncoder);     // Необязательно; без него считаются шаги
```

Команды `moveTo()`, `moveBy()`, `home()`, `stop()` и `syncPort()` общие с DC-осями. Без энкодера координата рассчитывается только по реально выданным импульсам; физические пропуски шагов не обнаруживаются. Частота вызовов `tick()` должна существенно превышать `maxSpeed * stepsPerUnit`. Откройте [пример без энкодера](examples/Stepper_OpenLoop/Stepper_OpenLoop.ino), [пример с энкодером](examples/Stepper_Encoder/Stepper_Encoder.ino) и [руководство по шаговым двигателям](docs/STEPPERS.md).

## Положение оси

| Координата | В мм (для вращательной оси — в градусах) | В отсчётах энкодера |
|---|---|---|
| Абсолютная, от машинного нуля Home | `getAbsolutePosition()` | `getAbsolutePositionCounts()` |
| Относительная, от заданного нуля | `getRelativePosition()` | `getRelativePositionCounts()` |

`setRelativeOrigin(20)` задаёт относительный ноль в абсолютной точке +20 мм; `setRelativeOrigin(-20)` — в −20 мм. Вызов `setRelativePosition()` обнуляет относительную координату в текущей точке. Подробнее — [координаты и примеры](docs/GUIDE.md#чтение-абсолютного-и-относительного-положения).

## Примеры

| Пример | Что показывает |
|---|---|
| [Linear_AVR](examples/Linear_AVR/Linear_AVR.ino) | Линейная ось, Uno/Nano, рабочий ноль и управление через Serial |
| [Stepper_OpenLoop](examples/Stepper_OpenLoop/Stepper_OpenLoop.ino) | Шаговый двигатель, положение по выданным импульсам |
| [Stepper_Encoder](examples/Stepper_Encoder/Stepper_Encoder.ino) | Шаговый двигатель с обратной связью по энкодеру |
| [Linear_ESP32](examples/Linear_ESP32/Linear_ESP32.ino) | Линейная ось и атомарное чтение энкодера на ESP32 |
| [Rotary_ESP32](examples/Rotary_ESP32/Rotary_ESP32.ino) | Углы, выбор направления и многооборотное движение |
| [ExternalFeedback](examples/ExternalFeedback/ExternalFeedback.ino) | Адаптер внешнего счётчика и диагностики |
| [SCurvePlotter](examples/SCurvePlotter/SCurvePlotter.ino) | Графики позиции, скорости, ускорения и рывка без оборудования |

<details>
<summary><b>Примеры синхронизации</b></summary>

| Пример | Задача |
|---|---|
| [SynchronizedXY_ESP32](examples/SynchronizedXY_ESP32/SynchronizedXY_ESP32.ino) / [Mega](examples/SynchronizedXY_Mega/SynchronizedXY_Mega.ino) | X:Y = 1:2, общий S-профиль и общая остановка |
| [RotaryGear_ESP32](examples/RotaryGear_ESP32/RotaryGear_ESP32.ino) | Z = 1 мм за оборот управляемой A |
| [SpindleSync_ESP32](examples/SpindleSync_ESP32/SpindleSync_ESP32.ino) | Внешний энкодер шпинделя: точная связь или плавная подача |

</details>

## Документация

[Полное руководство](docs/GUIDE.md) — настройки, масштаб энкодера, драйверы, homing, API и настройка регуляторов. [Отчёт о проверках](tests/VALIDATION.md) — сценарии и результаты сборок.

[Синхронизация осей и шпинделя](docs/SYNCHRONIZATION.md) — коэффициенты передачи, согласованные ограничения и допустимое отставание плавной подачи.

Код рассчитан на Arduino с C++11; проверены сборки для **Uno, Nano ATmega328P, Mega 2560, ESP32 и ESP32-S3**. На реальном оборудовании библиотека пока не испытывалась — коэффициенты регуляторов и пределы движения нужно настроить под свою механику.
