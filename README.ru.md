# ladrc-no-std

`ladrc-no-std` - Rust-крейт без зависимостей для LADRC-регуляторов в embedded
циклах управления с `no_std`.

LADRC - это linear active disturbance rejection control: линейное активное
подавление возмущений. Регулятор использует простую модель объекта, оценивает
неизвестную часть объекта как одно суммарное возмущение и компенсирует его в
реальном времени.

Имя пакета: `ladrc-no-std`. Имя для `use` в Rust: `ladrc_no_std`.
Английская документация находится в [README.md](README.md).

## Для Чего Этот Крейт

Используйте этот крейт, когда нужно удерживать измеряемую величину около
заданного значения, но точной модели объекта нет или она неудобна для прошивки.

Типичные embedded-задачи:

- управление скоростью мотора при меняющейся нагрузке;
- управление положением мотора, осью, подвесом или актуатором;
- управление клапаном, давлением, потоком или температурой;
- любой SISO-контур, где PID работает, но требует частой перенастройки.

Крейт не читает датчики и не управляет железом напрямую. Это делает ваше
приложение. Крейт только считает следующую команду управления:

```text
задание -> регулятор -> команда -> исполнитель/объект -> измерение
             ^                                      |
             +--------------------------------------+
```

## Основные Термины

- `reference` - задание: куда хотим прийти.
- `measurement` - текущее измерение с датчика в тех же единицах, что и
  `reference`.
- `control` - команда, которую возвращает регулятор.
- `sample_period` - номинальный период между обновлениями, в секундах.
- `b0` - примерный коэффициент от `control` к старшей моделируемой производной
  выхода. Знак должен быть правильным.
- `disturbance` - оценка неизвестной динамики объекта и внешних возмущений.

## Какой Регулятор Выбрать

Используйте `LadrcFirstOrder`, если команда в основном влияет на скорость
изменения измеряемой величины:

```text
y' = f + b0 * u
```

Подходит для температуры, давления, потока, скорости мотора.

Используйте `LadrcSecondOrder`, если команда в основном влияет на ускорение, а
измеряемая величина похожа на положение:

```text
y'' = f + b0 * u
```

Подходит для положения мотора, угла сустава, подвеса, линейного актуатора.

## Подключение

Для локального использования:

```toml
[dependencies]
ladrc-no-std = { path = "../ladrc-no-std" }
```

Если крейт опубликован на crates.io:

```toml
[dependencies]
ladrc-no-std = "0.1"
```

## Минимальный Пример Второго Порядка

```rust,ignore
use ladrc_no_std::{LadrcSecondOrder, LadrcSecondOrderConfig, OutputLimit};

let sample_period = 0.001; // номинальный цикл 1 кГц
let b0 = 1.0;              // первая оценка, знак должен быть правильным

let config = LadrcSecondOrderConfig::from_bandwidth(
    sample_period,
    b0,
    12.0, // полоса регулятора: скорость реакции
    60.0, // полоса наблюдателя: скорость оценки возмущения
)
.with_output_limit(OutputLimit::new(-1.0, 1.0));

let mut controller = LadrcSecondOrder::new(config)?;

loop {
    let reference = target_position();
    let measurement = read_position_sensor();

    let out = controller.update(reference, measurement);
    set_motor_command(out.control);
}
```

Обычный `update` предназначен для фиксированного периода. Если период цикла
плавает, используйте `update_at_millis` или `update_with_period`.

## Пример Первого Порядка

```rust,ignore
use ladrc_no_std::{LadrcFirstOrder, LadrcFirstOrderConfig, OutputLimit};

let config = LadrcFirstOrderConfig::from_bandwidth(
    0.01, // номинальный цикл 100 Гц
    0.8,  // примерное изменение скорости выхода на единицу команды
    2.0,  // полоса регулятора
    10.0, // полоса наблюдателя
)
.with_output_limit(OutputLimit::new(0.0, 1.0)); // мощность нагревателя: 0..100%

let mut controller = LadrcFirstOrder::new(config)?;

let out = controller.update(target_temperature(), measured_temperature());
set_heater_power(out.control);
```

## Фиксированный Или Плавающий Период Цикла

В `no_std` нет универсальных часов, поэтому крейт не может сам прочитать время
из микроконтроллера. Приложение должно либо вызывать регулятор из фиксированного
таймерного цикла, либо передавать информацию о времени в регулятор.

Используйте обычный `update`, когда цикл периодический:

```rust,ignore
let out = controller.update(reference, measurement);
```

Используйте `update_with_period`, если приложение уже посчитало прошедшее
время:

```rust,ignore
let actual_dt = seconds_since_previous_poll();
let out = controller.update_with_period(actual_dt, reference, measurement)?;
```

Используйте `update_at`, если есть монотонный timestamp в секундах:

```rust,ignore
let now = monotonic_seconds();
let out = controller.update_at(now, reference, measurement)?;
```

Используйте `update_at_millis`, если HAL возвращает время в целых
миллисекундах:

```rust,ignore
let now_ms = esp_hal::time::Instant::now()
    .duration_since_epoch()
    .as_millis();

let out = controller.update_at_millis(now_ms, reference, measurement)?;
```

Перед стартом цикла с плавающим периодом инициализируйте timestamp:

```rust,ignore
controller.reset_at_millis(monotonic_millis(), current_measurement);
```

`update_at_millis` сначала вычитает timestamp как `u64`, и только короткий
прошедший интервал переводит в секунды. Так не теряется миллисекундная точность
при большом uptime.

## Как Работает LADRC

Для объекта второго порядка LADRC предполагает только простую форму:

```text
y'' = f + b0 * u
```

Здесь `y` - измеряемый выход, `u` - команда управления. Неизвестный член `f`
содержит реальную динамику объекта и возмущения. Расширенный наблюдатель
оценивает:

- `position` - оценку выхода `y`;
- `velocity` - оценку производной выхода `y'`;
- `disturbance` - оценку `f`.

Регулятор считает обратную связь по состоянию и вычитает оцененное возмущение:

```text
feedback = kp * (reference - position) + kd * (reference_rate - velocity)
control  = (feedback - disturbance) / b0
```

## Настройка

`from_bandwidth(sample_period, b0, controller_bandwidth, observer_bandwidth)` -
самый простой интерфейс настройки.

Начинайте так:

1. Установите `sample_period` равным номинальному периоду цикла в секундах.
2. Установите `output_limit` равным реальному диапазону исполнительного
   механизма.
3. Выберите `b0` с правильным знаком.
4. Начните с небольшой `controller_bandwidth`.
5. Поставьте `observer_bandwidth` примерно в `3..5` раз выше
   `controller_bandwidth`.
6. Увеличивайте `controller_bandwidth`, пока реакция не станет достаточно
   быстрой.
7. Увеличивайте `observer_bandwidth`, если подавление возмущений слишком
   медленное.
8. Уменьшайте `observer_bandwidth`, если шум датчика заметно попадает в
   команду управления.

Метод настройки по полосе задает коэффициенты так:

- первый порядок: `kp = wc`, `beta1 = 2 * wo`, `beta2 = wo^2`;
- второй порядок: `kp = wc^2`, `kd = 2 * wc`,
  `beta1 = 3 * wo`, `beta2 = 3 * wo^2`, `beta3 = wo^3`.

## Чеклист Отладки

- Команда сразу упирается в ограничение: уменьшите `controller_bandwidth`,
  проверьте `b0` или расширьте `OutputLimit`, если исполнитель это позволяет.
- Команда имеет неправильный знак: скорее всего, знак `b0` неверный.
- Шум датчика дергает исполнитель: уменьшите `observer_bandwidth`.
- Контур стабилен, но слишком медленный: увеличьте `controller_bandwidth`.
- После изменения нагрузки возмущение компенсируется медленно: увеличьте
  `observer_bandwidth`.
- Период цикла не фиксирован: используйте `update_at_millis`, `update_at` или
  `update_with_period`.
- Есть удар при старте: вызовите `reset(...)` или `reset_at_millis(...)` перед
  включением автоматического управления.

## Обзор API

- `Float` - псевдоним для `f32`;
- `OutputLimit` - ограничитель управляющего сигнала;
- `ConfigError` - ошибки проверки параметров;
- `LadrcFirstOrderConfig`, `LadrcFirstOrder`;
- `LadrcSecondOrderConfig`, `LadrcSecondOrder`;
- `Ladrc` - псевдоним для `LadrcSecondOrder`;
- методы времени: `update`, `update_with_period`, `update_at`,
  `update_at_millis`, `reset_at`, `reset_at_millis`.

## Запуск Примеров

```powershell
cargo run --example first_order_temperature
cargo run --example second_order_position
cargo run --example variable_period_position
```

## Тесты И Проверки

```powershell
cargo test
cargo fmt --check
cargo clippy --all-targets -- -D warnings
cargo doc --no-deps
```

## Источники

- Gernot Herbst, "A Simulative Study on Active Disturbance Rejection Control
  (ADRC) as a Control Tool for Practitioners":
  <https://arxiv.org/abs/1908.04596>
- Gernot Herbst, "Transfer Function Analysis and Implementation of Active
  Disturbance Rejection Control": <https://arxiv.org/abs/2011.01044>
