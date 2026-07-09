# ladrc-no-std

`ladrc-no-std` is a dependency-free `no_std` Rust crate with LADRC controllers
for embedded control loops.

LADRC means linear active disturbance rejection control. It keeps the plant
model simple, estimates the unknown part of the plant as one total disturbance,
and compensates that disturbance online.

The package name is `ladrc-no-std`; the Rust import name is `ladrc_no_std`.
Russian documentation is available in [README.ru.md](README.ru.md).

## What This Crate Is For

Use this crate when you need a small controller that keeps a measured value near
a target value while the plant model is incomplete.

Typical embedded examples:

- motor speed control when load torque changes;
- motor position, axis angle, gimbal, or actuator position control;
- valve, pressure, flow, or temperature control;
- any single-input single-output loop where PID works but needs too much
  retuning when load or plant parameters change.

The crate does not read sensors and does not write actuators. Your application
does that. The crate only computes the next control command:

```text
reference -> controller -> control -> actuator/plant -> measurement
                ^                                  |
                +----------------------------------+
```

## Main Terms

- `reference` - the target value you want.
- `measurement` - the current sensor value in the same units as `reference`.
- `control` - the command returned by the controller.
- `sample_period` - nominal time between control updates, in seconds.
- `b0` - approximate gain from `control` to the highest modeled derivative of
  the output. The sign must be correct.
- `disturbance` - the observer's estimate of unknown plant dynamics plus
  external disturbance.

## Which Controller To Use

Use `LadrcFirstOrder` when the command mostly affects the speed of change of
the measured value:

```text
y' = f + b0 * u
```

Good fits: temperature, pressure, flow, motor speed.

Use `LadrcSecondOrder` when the command mostly affects acceleration and the
measured value is position-like:

```text
y'' = f + b0 * u
```

Good fits: motor position, robot joint angle, gimbal angle, linear actuator
position.

## Install

For local use:

```toml
[dependencies]
ladrc-no-std = { path = "../ladrc-no-std" }
```

If published to crates.io:

```toml
[dependencies]
ladrc-no-std = "0.1"
```

## Minimal Second-Order Example

```rust,ignore
use ladrc_no_std::{LadrcSecondOrder, LadrcSecondOrderConfig, OutputLimit};

let sample_period = 0.001; // 1 kHz nominal control loop
let b0 = 1.0;              // first estimate, must have the correct sign

let config = LadrcSecondOrderConfig::from_bandwidth(
    sample_period,
    b0,
    12.0, // controller bandwidth: response speed
    60.0, // observer bandwidth: disturbance estimation speed
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

Plain `update` is for fixed-period loops. For a jittery polling loop, use
`update_at_millis` or `update_with_period`.

## First-Order Example

```rust,ignore
use ladrc_no_std::{LadrcFirstOrder, LadrcFirstOrderConfig, OutputLimit};

let config = LadrcFirstOrderConfig::from_bandwidth(
    0.01, // 100 Hz nominal loop
    0.8,  // approximate output-rate change per unit command
    2.0,  // controller bandwidth
    10.0, // observer bandwidth
)
.with_output_limit(OutputLimit::new(0.0, 1.0)); // heater power: 0..100%

let mut controller = LadrcFirstOrder::new(config)?;

let out = controller.update(target_temperature(), measured_temperature());
set_heater_power(out.control);
```

## Fixed Or Variable Loop Timing

There is no universal clock in `no_std`, so the crate cannot read time from the
microcontroller by itself. Your application must either run the controller from
a fixed timer interrupt/task or pass time information into the controller.

Use plain `update` when the loop is periodic:

```rust,ignore
let out = controller.update(reference, measurement);
```

Use `update_with_period` when your application already computed elapsed time:

```rust,ignore
let actual_dt = seconds_since_previous_poll();
let out = controller.update_with_period(actual_dt, reference, measurement)?;
```

Use `update_at` when you have a monotonic timestamp in seconds:

```rust,ignore
let now = monotonic_seconds();
let out = controller.update_at(now, reference, measurement)?;
```

Use `update_at_millis` when your HAL gives time as integer milliseconds:

```rust,ignore
let now_ms = esp_hal::time::Instant::now()
    .duration_since_epoch()
    .as_millis();

let out = controller.update_at_millis(now_ms, reference, measurement)?;
```

Before starting a variable-period loop, initialize the timestamp:

```rust,ignore
controller.reset_at_millis(monotonic_millis(), current_measurement);
```

`update_at_millis` subtracts timestamps as `u64` first and converts only the
small elapsed interval to seconds. That avoids losing millisecond precision
after long uptime.

## How LADRC Works

For a second-order plant, LADRC assumes only this simple shape:

```text
y'' = f + b0 * u
```

Here `y` is the measured output and `u` is the control command. The unknown
term `f` contains real plant dynamics and disturbances. The extended state
observer estimates:

- `position` - estimated output `y`;
- `velocity` - estimated output derivative `y'`;
- `disturbance` - estimated `f`.

The controller computes a state-feedback command, then subtracts the estimated
disturbance:

```text
feedback = kp * (reference - position) + kd * (reference_rate - velocity)
control  = (feedback - disturbance) / b0
```

## Tuning

`from_bandwidth(sample_period, b0, controller_bandwidth, observer_bandwidth)`
is the easiest tuning interface.

Start here:

1. Set `sample_period` to the nominal control-loop period in seconds.
2. Set `output_limit` to the real actuator range.
3. Pick `b0` with the correct sign.
4. Start with a low `controller_bandwidth`.
5. Set `observer_bandwidth` about `3..5` times higher than
   `controller_bandwidth`.
6. Increase `controller_bandwidth` until response is fast enough.
7. Increase `observer_bandwidth` if disturbance rejection is too slow.
8. Decrease `observer_bandwidth` if measurement noise appears in the actuator
   command.

The bandwidth helper sets gains as follows:

- first order: `kp = wc`, `beta1 = 2 * wo`, `beta2 = wo^2`;
- second order: `kp = wc^2`, `kd = 2 * wc`,
  `beta1 = 3 * wo`, `beta2 = 3 * wo^2`, `beta3 = wo^3`.

## Debugging Checklist

- Output immediately saturates: lower `controller_bandwidth`, check `b0`, or
  widen `OutputLimit` if the actuator really supports it.
- Output has the wrong sign: `b0` sign is likely wrong.
- Sensor noise strongly shakes the actuator: lower `observer_bandwidth`.
- Response is stable but too slow: raise `controller_bandwidth`.
- Disturbance rejection is slow after load changes: raise `observer_bandwidth`.
- Loop period is not fixed: use `update_at_millis`, `update_at`, or
  `update_with_period`.
- Large startup kick: call `reset(...)` or `reset_at_millis(...)` before
  enabling automatic control.

## API Map

- `Float` - alias for `f32`;
- `OutputLimit` - control output clamp;
- `ConfigError` - validation errors;
- `LadrcFirstOrderConfig`, `LadrcFirstOrder`;
- `LadrcSecondOrderConfig`, `LadrcSecondOrder`;
- `Ladrc` - alias for `LadrcSecondOrder`;
- timing methods: `update`, `update_with_period`, `update_at`,
  `update_at_millis`, `reset_at`, `reset_at_millis`.

## Run Examples

```powershell
cargo run --example first_order_temperature
cargo run --example second_order_position
cargo run --example variable_period_position
```

## Tests And Checks

```powershell
cargo test
cargo fmt --check
cargo clippy --all-targets -- -D warnings
cargo doc --no-deps
```

## References

- Gernot Herbst, "A Simulative Study on Active Disturbance Rejection Control
  (ADRC) as a Control Tool for Practitioners":
  <https://arxiv.org/abs/1908.04596>
- Gernot Herbst, "Transfer Function Analysis and Implementation of Active
  Disturbance Rejection Control": <https://arxiv.org/abs/2011.01044>
