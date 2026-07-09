#![no_std]
#![doc = include_str!("../README.md")]

/// Floating-point type used by the crate.
///
/// `f32` keeps the implementation compact for microcontrollers and avoids
/// pulling in generic numeric traits.
pub type Float = f32;

const MIN_ABS_B0: Float = 1.0e-9;

/// Configuration validation error.
///
/// Constructors validate parameters before a controller is created. This avoids
/// silently running a loop with a zero sample time, zero plant gain, inverted
/// output limit, or non-finite coefficient.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConfigError {
    /// A value must be finite.
    NonFinite,
    /// The sample period must be positive.
    NonPositiveSamplePeriod,
    /// A bandwidth, gain, tracking speed, or delta must be positive.
    NonPositiveParameter,
    /// The plant input gain estimate `b0` is zero or too close to zero.
    ZeroPlantGain,
    /// The lower output limit is greater than the upper output limit.
    InvalidOutputLimit,
}

/// Optional control output clamp.
///
/// Use this to match the command range of the real actuator, for example
/// `0.0..1.0` for normalized heater power or `-1.0..1.0` for a bidirectional
/// motor command. The controller still reports the unsaturated value in its
/// update output, which is useful during tuning.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct OutputLimit {
    /// Minimum allowed controller output.
    pub min: Float,
    /// Maximum allowed controller output.
    pub max: Float,
}

impl OutputLimit {
    /// Creates a new output clamp.
    #[inline]
    pub const fn new(min: Float, max: Float) -> Self {
        Self { min, max }
    }

    /// Applies the clamp to `value`.
    #[inline]
    pub fn apply(self, value: Float) -> Float {
        if value < self.min {
            self.min
        } else if value > self.max {
            self.max
        } else {
            value
        }
    }

    #[inline]
    fn validate(self) -> Result<(), ConfigError> {
        if !finite(self.min) || !finite(self.max) {
            return Err(ConfigError::NonFinite);
        }

        if self.min > self.max {
            return Err(ConfigError::InvalidOutputLimit);
        }

        Ok(())
    }
}

#[inline]
fn apply_limit(value: Float, limit: Option<OutputLimit>) -> Float {
    match limit {
        Some(limit) => limit.apply(value),
        None => value,
    }
}

#[inline]
fn validate_limit(limit: Option<OutputLimit>) -> Result<(), ConfigError> {
    match limit {
        Some(limit) => limit.validate(),
        None => Ok(()),
    }
}

#[inline]
fn finite(value: Float) -> bool {
    value.is_finite()
}

#[inline]
fn validate_sample_period(sample_period: Float) -> Result<(), ConfigError> {
    if !finite(sample_period) {
        return Err(ConfigError::NonFinite);
    }

    if sample_period <= 0.0 {
        return Err(ConfigError::NonPositiveSamplePeriod);
    }

    Ok(())
}

#[inline]
fn validate_time(time_seconds: Float) -> Result<(), ConfigError> {
    if !finite(time_seconds) {
        return Err(ConfigError::NonFinite);
    }

    Ok(())
}

#[inline]
fn elapsed_sample_period(
    now_seconds: Float,
    last_update_at: &mut Option<Float>,
    fallback_sample_period: Float,
) -> Result<Float, ConfigError> {
    validate_time(now_seconds)?;

    let sample_period = match *last_update_at {
        Some(previous) => {
            let elapsed = now_seconds - previous;
            validate_sample_period(elapsed)?;
            elapsed
        }
        None => fallback_sample_period,
    };

    *last_update_at = Some(now_seconds);
    Ok(sample_period)
}

#[inline]
fn elapsed_sample_period_millis(
    now_millis: u64,
    last_update_at_millis: &mut Option<u64>,
    fallback_sample_period: Float,
) -> Result<Float, ConfigError> {
    let sample_period = match *last_update_at_millis {
        Some(previous) => {
            let elapsed_millis = now_millis
                .checked_sub(previous)
                .ok_or(ConfigError::NonPositiveSamplePeriod)?;

            if elapsed_millis == 0 {
                return Err(ConfigError::NonPositiveSamplePeriod);
            }

            elapsed_millis as Float * 0.001
        }
        None => fallback_sample_period,
    };

    *last_update_at_millis = Some(now_millis);
    Ok(sample_period)
}

#[inline]
fn validate_positive(value: Float) -> Result<(), ConfigError> {
    if !finite(value) {
        return Err(ConfigError::NonFinite);
    }

    if value <= 0.0 {
        return Err(ConfigError::NonPositiveParameter);
    }

    Ok(())
}

#[inline]
fn validate_b0(b0: Float) -> Result<(), ConfigError> {
    if !finite(b0) {
        return Err(ConfigError::NonFinite);
    }

    if b0.abs() <= MIN_ABS_B0 {
        return Err(ConfigError::ZeroPlantGain);
    }

    Ok(())
}

pub mod ladrc {
    //! Linear active disturbance rejection controllers.
    //!
    //! LADRC keeps the model small and estimates the unknown part of the plant
    //! as one total disturbance. The crate provides first-order and
    //! second-order controllers with a bandwidth-based configuration helper.
    //!
    //! Use [`LadrcFirstOrder`] for plants that are well represented by:
    //!
    //! ```text
    //! y' = f + b0 * u
    //! ```
    //!
    //! Use [`LadrcSecondOrder`] for plants that are well represented by:
    //!
    //! ```text
    //! y'' = f + b0 * u
    //! ```
    //!
    //! In both cases `f` is not modeled explicitly. The extended state observer
    //! estimates it as `disturbance`, and the control law subtracts that
    //! estimate before dividing by `b0`.
    //!
    //! Typical update order in an application:
    //!
    //! ```text
    //! read sensor -> controller.update(reference, measurement) -> write actuator
    //! ```

    use super::{
        apply_limit, elapsed_sample_period, elapsed_sample_period_millis, validate_b0,
        validate_limit, validate_positive, validate_sample_period, validate_time, ConfigError,
        Float, OutputLimit,
    };

    /// Estimated first-order LADRC state.
    #[derive(Debug, Clone, Copy, Default, PartialEq)]
    pub struct FirstOrderEstimate {
        /// Estimated process output.
        pub output: Float,
        /// Estimated total disturbance.
        pub disturbance: Float,
    }

    /// One first-order LADRC update result.
    #[derive(Debug, Clone, Copy, PartialEq)]
    pub struct FirstOrderOutput {
        /// Saturated control signal.
        pub control: Float,
        /// Control before output saturation.
        pub unsaturated_control: Float,
        /// Pure feedback term before disturbance compensation.
        pub feedback: Float,
        /// Estimated state after the observer update.
        pub estimate: FirstOrderEstimate,
    }

    /// First-order LADRC configuration.
    ///
    /// This configuration is for processes where the control command mostly
    /// changes the first derivative of the measured output:
    ///
    /// ```text
    /// y' = f + b0 * u
    /// ```
    ///
    /// Examples include temperature, pressure, flow, and motor speed loops.
    #[derive(Debug, Clone, Copy, PartialEq)]
    pub struct LadrcFirstOrderConfig {
        /// Nominal sampling period in seconds.
        ///
        /// Plain `update` uses this value directly. `update_at` uses it only
        /// for the first call when no previous timestamp is known yet.
        pub sample_period: Float,
        /// Estimated plant input gain from command to output rate.
        ///
        /// The value may be approximate, but the sign must match the real
        /// plant. A wrong sign usually makes the closed loop diverge.
        pub b0: Float,
        /// Controller feedback gain.
        pub kp: Float,
        /// First observer gain for output estimation.
        pub observer_beta1: Float,
        /// Second observer gain for total-disturbance estimation.
        pub observer_beta2: Float,
        /// Optional output clamp.
        pub output_limit: Option<OutputLimit>,
    }

    impl LadrcFirstOrderConfig {
        /// Creates a configuration from controller and observer bandwidths.
        ///
        /// The gain placement is `kp = wc`, `beta1 = 2 * wo`,
        /// `beta2 = wo^2`.
        ///
        /// `controller_bandwidth` controls the closed-loop response speed.
        /// `observer_bandwidth` controls how fast the observer estimates the
        /// total disturbance. A common first value is `observer_bandwidth`
        /// around three to five times `controller_bandwidth`.
        #[inline]
        pub fn from_bandwidth(
            sample_period: Float,
            b0: Float,
            controller_bandwidth: Float,
            observer_bandwidth: Float,
        ) -> Self {
            Self {
                sample_period,
                b0,
                kp: controller_bandwidth,
                observer_beta1: 2.0 * observer_bandwidth,
                observer_beta2: observer_bandwidth * observer_bandwidth,
                output_limit: None,
            }
        }

        /// Returns the same configuration with an output clamp.
        #[inline]
        pub const fn with_output_limit(mut self, limit: OutputLimit) -> Self {
            self.output_limit = Some(limit);
            self
        }

        /// Validates all parameters.
        pub fn validate(self) -> Result<(), ConfigError> {
            validate_sample_period(self.sample_period)?;
            validate_b0(self.b0)?;
            validate_positive(self.kp)?;
            validate_positive(self.observer_beta1)?;
            validate_positive(self.observer_beta2)?;
            validate_limit(self.output_limit)
        }
    }

    /// First-order linear active disturbance rejection controller.
    ///
    /// Call [`LadrcFirstOrder::update`] once per fixed sample period. The
    /// method reads no hardware; it only consumes the reference and measured
    /// output and returns the next actuator command.
    #[derive(Debug, Clone, Copy, PartialEq)]
    pub struct LadrcFirstOrder {
        config: LadrcFirstOrderConfig,
        z1: Float,
        z2: Float,
        last_control: Float,
        last_update_at: Option<Float>,
        last_update_at_millis: Option<u64>,
    }

    impl LadrcFirstOrder {
        /// Creates a controller and validates the configuration.
        #[inline]
        pub fn new(config: LadrcFirstOrderConfig) -> Result<Self, ConfigError> {
            config.validate()?;
            Ok(Self {
                config,
                z1: 0.0,
                z2: 0.0,
                last_control: 0.0,
                last_update_at: None,
                last_update_at_millis: None,
            })
        }

        /// Returns the current configuration.
        #[inline]
        pub const fn config(&self) -> LadrcFirstOrderConfig {
            self.config
        }

        /// Returns the current observer estimate.
        #[inline]
        pub const fn estimate(&self) -> FirstOrderEstimate {
            FirstOrderEstimate {
                output: self.z1,
                disturbance: self.z2,
            }
        }

        /// Returns the control signal used by the previous observer update.
        #[inline]
        pub const fn last_control(&self) -> Float {
            self.last_control
        }

        /// Returns the timestamp stored by the last [`LadrcFirstOrder::update_at`]
        /// call.
        #[inline]
        pub const fn last_update_at(&self) -> Option<Float> {
            self.last_update_at
        }

        /// Returns the millisecond timestamp stored by the last
        /// [`LadrcFirstOrder::update_at_millis`] call.
        #[inline]
        pub const fn last_update_at_millis(&self) -> Option<u64> {
            self.last_update_at_millis
        }

        /// Resets the observer to a measured output and clears disturbance and
        /// control memory.
        #[inline]
        pub fn reset(&mut self, measurement: Float) {
            self.reset_with(measurement, 0.0, 0.0);
        }

        /// Resets the observer and initializes the timestamp used by
        /// [`LadrcFirstOrder::update_at`].
        ///
        /// Use this before enabling a variable-period loop. It prevents the
        /// first `update_at` call from falling back to the nominal
        /// `config.sample_period`.
        #[inline]
        pub fn reset_at(
            &mut self,
            now_seconds: Float,
            measurement: Float,
        ) -> Result<(), ConfigError> {
            validate_time(now_seconds)?;
            self.reset(measurement);
            self.last_update_at = Some(now_seconds);
            self.last_update_at_millis = None;
            Ok(())
        }

        /// Resets the observer and initializes the millisecond timestamp used
        /// by [`LadrcFirstOrder::update_at_millis`].
        ///
        /// This is the preferred timestamp API for HAL clocks that return
        /// integer milliseconds, such as `esp-hal`'s
        /// `Instant::now().duration_since_epoch().as_millis()`.
        #[inline]
        pub fn reset_at_millis(&mut self, now_millis: u64, measurement: Float) {
            self.reset(measurement);
            self.last_update_at_millis = Some(now_millis);
            self.last_update_at = None;
        }

        /// Resets all controller state.
        #[inline]
        pub fn reset_with(
            &mut self,
            estimated_output: Float,
            estimated_disturbance: Float,
            last_control: Float,
        ) {
            self.z1 = estimated_output;
            self.z2 = estimated_disturbance;
            self.last_control = last_control;
            self.last_update_at = None;
            self.last_update_at_millis = None;
        }

        /// Runs one LADRC sample.
        ///
        /// The extended state observer uses the previous saturated control
        /// signal, then the new control signal is computed and stored for the
        /// next call.
        ///
        /// `reference` and `measurement` must use the same units. The returned
        /// `control` uses the actuator units implied by `b0`.
        pub fn update(&mut self, reference: Float, measurement: Float) -> FirstOrderOutput {
            self.update_unchecked(self.config.sample_period, reference, measurement)
        }

        /// Runs one LADRC sample with an explicit period in seconds.
        ///
        /// Use this when the control loop period is not perfectly constant and
        /// the application already computed the elapsed time since the previous
        /// sample.
        pub fn update_with_period(
            &mut self,
            sample_period: Float,
            reference: Float,
            measurement: Float,
        ) -> Result<FirstOrderOutput, ConfigError> {
            validate_sample_period(sample_period)?;
            Ok(self.update_unchecked(sample_period, reference, measurement))
        }

        /// Runs one LADRC sample at a monotonic timestamp in seconds.
        ///
        /// The controller stores the previous timestamp and computes
        /// `sample_period = now_seconds - previous_now_seconds` internally.
        /// The first call uses the nominal `config.sample_period` because no
        /// previous timestamp exists yet. Call [`LadrcFirstOrder::reset_at`] to
        /// initialize the timestamp before the first variable-period update.
        pub fn update_at(
            &mut self,
            now_seconds: Float,
            reference: Float,
            measurement: Float,
        ) -> Result<FirstOrderOutput, ConfigError> {
            let sample_period = elapsed_sample_period(
                now_seconds,
                &mut self.last_update_at,
                self.config.sample_period,
            )?;
            Ok(self.update_unchecked(sample_period, reference, measurement))
        }

        /// Runs one LADRC sample at a monotonic timestamp in milliseconds.
        ///
        /// The controller computes `dt` with integer subtraction first, then
        /// converts only that short elapsed interval to seconds. This avoids
        /// losing millisecond precision after long uptime.
        pub fn update_at_millis(
            &mut self,
            now_millis: u64,
            reference: Float,
            measurement: Float,
        ) -> Result<FirstOrderOutput, ConfigError> {
            let sample_period = elapsed_sample_period_millis(
                now_millis,
                &mut self.last_update_at_millis,
                self.config.sample_period,
            )?;
            Ok(self.update_unchecked(sample_period, reference, measurement))
        }

        fn update_unchecked(
            &mut self,
            sample_period: Float,
            reference: Float,
            measurement: Float,
        ) -> FirstOrderOutput {
            self.update_observer(measurement, sample_period);

            let feedback = self.config.kp * (reference - self.z1);
            let unsaturated_control = (feedback - self.z2) / self.config.b0;
            let control = apply_limit(unsaturated_control, self.config.output_limit);
            self.last_control = control;

            FirstOrderOutput {
                control,
                unsaturated_control,
                feedback,
                estimate: self.estimate(),
            }
        }

        fn update_observer(&mut self, measurement: Float, sample_period: Float) {
            let e = self.z1 - measurement;
            let h = sample_period;

            self.z1 +=
                h * (self.z2 - self.config.observer_beta1 * e + self.config.b0 * self.last_control);
            self.z2 += h * (-self.config.observer_beta2 * e);
        }
    }

    /// Estimated second-order LADRC state.
    #[derive(Debug, Clone, Copy, Default, PartialEq)]
    pub struct SecondOrderEstimate {
        /// Estimated process output.
        pub position: Float,
        /// Estimated output derivative.
        pub velocity: Float,
        /// Estimated total disturbance.
        pub disturbance: Float,
    }

    /// One second-order LADRC update result.
    #[derive(Debug, Clone, Copy, PartialEq)]
    pub struct SecondOrderOutput {
        /// Saturated control signal.
        pub control: Float,
        /// Control before output saturation.
        pub unsaturated_control: Float,
        /// Pure feedback term before disturbance compensation.
        pub feedback: Float,
        /// Estimated state after the observer update.
        pub estimate: SecondOrderEstimate,
    }

    /// Second-order LADRC configuration.
    ///
    /// This configuration is for processes where the control command mostly
    /// changes the second derivative of the measured output:
    ///
    /// ```text
    /// y'' = f + b0 * u
    /// ```
    ///
    /// Examples include motor position, robot joint angle, gimbal angle, and
    /// linear actuator position loops.
    #[derive(Debug, Clone, Copy, PartialEq)]
    pub struct LadrcSecondOrderConfig {
        /// Nominal sampling period in seconds.
        ///
        /// Plain `update` uses this value directly. `update_at` uses it only
        /// for the first call when no previous timestamp is known yet.
        pub sample_period: Float,
        /// Estimated plant input gain from command to output acceleration.
        ///
        /// The value may be approximate, but the sign must match the real
        /// plant. A wrong sign usually makes the closed loop diverge.
        pub b0: Float,
        /// Proportional-like state feedback gain.
        pub kp: Float,
        /// Derivative-like state feedback gain.
        pub kd: Float,
        /// First observer gain for output estimation.
        pub observer_beta1: Float,
        /// Second observer gain for derivative estimation.
        pub observer_beta2: Float,
        /// Third observer gain for total-disturbance estimation.
        pub observer_beta3: Float,
        /// Optional output clamp.
        pub output_limit: Option<OutputLimit>,
    }

    impl LadrcSecondOrderConfig {
        /// Creates a configuration from controller and observer bandwidths.
        ///
        /// The gain placement is `kp = wc^2`, `kd = 2 * wc`,
        /// `beta1 = 3 * wo`, `beta2 = 3 * wo^2`, `beta3 = wo^3`.
        ///
        /// `controller_bandwidth` controls the closed-loop response speed.
        /// `observer_bandwidth` controls how fast the observer estimates the
        /// total disturbance. A common first value is `observer_bandwidth`
        /// around three to five times `controller_bandwidth`.
        #[inline]
        pub fn from_bandwidth(
            sample_period: Float,
            b0: Float,
            controller_bandwidth: Float,
            observer_bandwidth: Float,
        ) -> Self {
            Self {
                sample_period,
                b0,
                kp: controller_bandwidth * controller_bandwidth,
                kd: 2.0 * controller_bandwidth,
                observer_beta1: 3.0 * observer_bandwidth,
                observer_beta2: 3.0 * observer_bandwidth * observer_bandwidth,
                observer_beta3: observer_bandwidth * observer_bandwidth * observer_bandwidth,
                output_limit: None,
            }
        }

        /// Returns the same configuration with an output clamp.
        #[inline]
        pub const fn with_output_limit(mut self, limit: OutputLimit) -> Self {
            self.output_limit = Some(limit);
            self
        }

        /// Validates all parameters.
        pub fn validate(self) -> Result<(), ConfigError> {
            validate_sample_period(self.sample_period)?;
            validate_b0(self.b0)?;
            validate_positive(self.kp)?;
            validate_positive(self.kd)?;
            validate_positive(self.observer_beta1)?;
            validate_positive(self.observer_beta2)?;
            validate_positive(self.observer_beta3)?;
            validate_limit(self.output_limit)
        }
    }

    /// Second-order linear active disturbance rejection controller.
    ///
    /// This is the recommended default controller for position-like embedded
    /// plants. Call [`LadrcSecondOrder::update`] once per fixed sample period
    /// when the reference derivative is zero, or
    /// [`LadrcSecondOrder::update_with_rate`] when a trajectory generator
    /// provides both position and velocity references.
    #[derive(Debug, Clone, Copy, PartialEq)]
    pub struct LadrcSecondOrder {
        config: LadrcSecondOrderConfig,
        z1: Float,
        z2: Float,
        z3: Float,
        last_control: Float,
        last_update_at: Option<Float>,
        last_update_at_millis: Option<u64>,
    }

    impl LadrcSecondOrder {
        /// Creates a controller and validates the configuration.
        #[inline]
        pub fn new(config: LadrcSecondOrderConfig) -> Result<Self, ConfigError> {
            config.validate()?;
            Ok(Self {
                config,
                z1: 0.0,
                z2: 0.0,
                z3: 0.0,
                last_control: 0.0,
                last_update_at: None,
                last_update_at_millis: None,
            })
        }

        /// Returns the current configuration.
        #[inline]
        pub const fn config(&self) -> LadrcSecondOrderConfig {
            self.config
        }

        /// Returns the current observer estimate.
        #[inline]
        pub const fn estimate(&self) -> SecondOrderEstimate {
            SecondOrderEstimate {
                position: self.z1,
                velocity: self.z2,
                disturbance: self.z3,
            }
        }

        /// Returns the control signal used by the previous observer update.
        #[inline]
        pub const fn last_control(&self) -> Float {
            self.last_control
        }

        /// Returns the timestamp stored by the last [`LadrcSecondOrder::update_at`]
        /// call.
        #[inline]
        pub const fn last_update_at(&self) -> Option<Float> {
            self.last_update_at
        }

        /// Returns the millisecond timestamp stored by the last
        /// [`LadrcSecondOrder::update_at_millis`] call.
        #[inline]
        pub const fn last_update_at_millis(&self) -> Option<u64> {
            self.last_update_at_millis
        }

        /// Resets the observer to a measured output and clears derivative,
        /// disturbance, and control memory.
        #[inline]
        pub fn reset(&mut self, measurement: Float) {
            self.reset_with(measurement, 0.0, 0.0, 0.0);
        }

        /// Resets the observer and initializes the timestamp used by
        /// [`LadrcSecondOrder::update_at`].
        ///
        /// Use this before enabling a variable-period loop. It prevents the
        /// first `update_at` call from falling back to the nominal
        /// `config.sample_period`.
        #[inline]
        pub fn reset_at(
            &mut self,
            now_seconds: Float,
            measurement: Float,
        ) -> Result<(), ConfigError> {
            validate_time(now_seconds)?;
            self.reset(measurement);
            self.last_update_at = Some(now_seconds);
            self.last_update_at_millis = None;
            Ok(())
        }

        /// Resets the observer and initializes the millisecond timestamp used
        /// by [`LadrcSecondOrder::update_at_millis`].
        ///
        /// This is the preferred timestamp API for HAL clocks that return
        /// integer milliseconds, such as `esp-hal`'s
        /// `Instant::now().duration_since_epoch().as_millis()`.
        #[inline]
        pub fn reset_at_millis(&mut self, now_millis: u64, measurement: Float) {
            self.reset(measurement);
            self.last_update_at_millis = Some(now_millis);
            self.last_update_at = None;
        }

        /// Resets all controller state.
        #[inline]
        pub fn reset_with(
            &mut self,
            estimated_position: Float,
            estimated_velocity: Float,
            estimated_disturbance: Float,
            last_control: Float,
        ) {
            self.z1 = estimated_position;
            self.z2 = estimated_velocity;
            self.z3 = estimated_disturbance;
            self.last_control = last_control;
            self.last_update_at = None;
            self.last_update_at_millis = None;
        }

        /// Runs one LADRC sample with zero reference velocity.
        ///
        /// Use this for setpoint control where the desired target is a position
        /// or angle and the desired final velocity is zero.
        #[inline]
        pub fn update(&mut self, reference: Float, measurement: Float) -> SecondOrderOutput {
            self.update_with_rate(reference, 0.0, measurement)
        }

        /// Runs one LADRC sample with zero reference velocity and an explicit
        /// period in seconds.
        ///
        /// Use this when the control loop period is not perfectly constant and
        /// the application already computed the elapsed time since the previous
        /// sample.
        #[inline]
        pub fn update_with_period(
            &mut self,
            sample_period: Float,
            reference: Float,
            measurement: Float,
        ) -> Result<SecondOrderOutput, ConfigError> {
            self.update_with_period_and_rate(sample_period, reference, 0.0, measurement)
        }

        /// Runs one LADRC sample with zero reference velocity at a monotonic
        /// timestamp in seconds.
        ///
        /// The controller stores the previous timestamp and computes
        /// `sample_period = now_seconds - previous_now_seconds` internally.
        /// The first call uses the nominal `config.sample_period` because no
        /// previous timestamp exists yet. Call [`LadrcSecondOrder::reset_at`] to
        /// initialize the timestamp before the first variable-period update.
        #[inline]
        pub fn update_at(
            &mut self,
            now_seconds: Float,
            reference: Float,
            measurement: Float,
        ) -> Result<SecondOrderOutput, ConfigError> {
            self.update_at_with_rate(now_seconds, reference, 0.0, measurement)
        }

        /// Runs one LADRC sample with zero reference velocity at a monotonic
        /// timestamp in milliseconds.
        ///
        /// The controller computes `dt` with integer subtraction first, then
        /// converts only that short elapsed interval to seconds. This avoids
        /// losing millisecond precision after long uptime.
        #[inline]
        pub fn update_at_millis(
            &mut self,
            now_millis: u64,
            reference: Float,
            measurement: Float,
        ) -> Result<SecondOrderOutput, ConfigError> {
            self.update_at_millis_with_rate(now_millis, reference, 0.0, measurement)
        }

        /// Runs one LADRC sample with an explicit reference velocity.
        ///
        /// The extended state observer uses the previous saturated control
        /// signal, then the new control signal is computed and stored for the
        /// next call.
        ///
        /// Use this when an external trajectory generator provides both
        /// reference position and reference velocity.
        pub fn update_with_rate(
            &mut self,
            reference: Float,
            reference_rate: Float,
            measurement: Float,
        ) -> SecondOrderOutput {
            self.update_unchecked(
                self.config.sample_period,
                reference,
                reference_rate,
                measurement,
            )
        }

        /// Runs one LADRC sample with explicit period and explicit reference
        /// velocity.
        ///
        /// Use this when an external trajectory generator provides both
        /// reference position and reference velocity, and the application
        /// already computed the elapsed time since the previous sample.
        pub fn update_with_period_and_rate(
            &mut self,
            sample_period: Float,
            reference: Float,
            reference_rate: Float,
            measurement: Float,
        ) -> Result<SecondOrderOutput, ConfigError> {
            validate_sample_period(sample_period)?;
            Ok(self.update_unchecked(sample_period, reference, reference_rate, measurement))
        }

        /// Runs one LADRC sample at a monotonic timestamp with explicit
        /// reference velocity.
        ///
        /// The controller stores the previous timestamp and computes the sample
        /// period internally. Call [`LadrcSecondOrder::reset_at`] before the
        /// first call if you do not want to use the nominal sample period for
        /// the first update.
        pub fn update_at_with_rate(
            &mut self,
            now_seconds: Float,
            reference: Float,
            reference_rate: Float,
            measurement: Float,
        ) -> Result<SecondOrderOutput, ConfigError> {
            let sample_period = elapsed_sample_period(
                now_seconds,
                &mut self.last_update_at,
                self.config.sample_period,
            )?;
            Ok(self.update_unchecked(sample_period, reference, reference_rate, measurement))
        }

        /// Runs one LADRC sample at a monotonic millisecond timestamp with
        /// explicit reference velocity.
        ///
        /// This is useful with HAL clocks that expose integer milliseconds.
        pub fn update_at_millis_with_rate(
            &mut self,
            now_millis: u64,
            reference: Float,
            reference_rate: Float,
            measurement: Float,
        ) -> Result<SecondOrderOutput, ConfigError> {
            let sample_period = elapsed_sample_period_millis(
                now_millis,
                &mut self.last_update_at_millis,
                self.config.sample_period,
            )?;
            Ok(self.update_unchecked(sample_period, reference, reference_rate, measurement))
        }

        fn update_unchecked(
            &mut self,
            sample_period: Float,
            reference: Float,
            reference_rate: Float,
            measurement: Float,
        ) -> SecondOrderOutput {
            self.update_observer(measurement, sample_period);

            let feedback = self.config.kp * (reference - self.z1)
                + self.config.kd * (reference_rate - self.z2);
            let unsaturated_control = (feedback - self.z3) / self.config.b0;
            let control = apply_limit(unsaturated_control, self.config.output_limit);
            self.last_control = control;

            SecondOrderOutput {
                control,
                unsaturated_control,
                feedback,
                estimate: self.estimate(),
            }
        }

        fn update_observer(&mut self, measurement: Float, sample_period: Float) {
            let e = self.z1 - measurement;
            let h = sample_period;

            self.z1 += h * (self.z2 - self.config.observer_beta1 * e);
            self.z2 +=
                h * (self.z3 - self.config.observer_beta2 * e + self.config.b0 * self.last_control);
            self.z3 += h * (-self.config.observer_beta3 * e);
        }
    }

    /// Conventional alias for the second-order LADRC controller.
    pub type Ladrc = LadrcSecondOrder;
}

pub use ladrc::{
    FirstOrderEstimate, FirstOrderOutput, Ladrc, LadrcFirstOrder, LadrcFirstOrderConfig,
    LadrcSecondOrder, LadrcSecondOrderConfig, SecondOrderEstimate, SecondOrderOutput,
};

#[cfg(test)]
extern crate std;

#[cfg(test)]
mod tests {
    use super::*;

    fn assert_close(actual: Float, expected: Float, tolerance: Float) {
        assert!(
            (actual - expected).abs() <= tolerance,
            "actual={actual}, expected={expected}, tolerance={tolerance}"
        );
    }

    #[test]
    fn output_limit_clamps_values() {
        let limit = OutputLimit::new(-2.0, 3.0);

        assert_eq!(limit.apply(-4.0), -2.0);
        assert_eq!(limit.apply(1.0), 1.0);
        assert_eq!(limit.apply(5.0), 3.0);
    }

    #[test]
    fn validates_bad_configurations() {
        let bad_b0 = ladrc::LadrcSecondOrderConfig::from_bandwidth(0.001, 0.0, 10.0, 50.0);
        assert_eq!(
            ladrc::LadrcSecondOrder::new(bad_b0).unwrap_err(),
            ConfigError::ZeroPlantGain
        );

        let bad_h = ladrc::LadrcFirstOrderConfig::from_bandwidth(0.0, 1.0, 10.0, 50.0);
        assert_eq!(
            ladrc::LadrcFirstOrder::new(bad_h).unwrap_err(),
            ConfigError::NonPositiveSamplePeriod
        );

        let bad_limit = ladrc::LadrcFirstOrderConfig::from_bandwidth(0.001, 1.0, 10.0, 50.0)
            .with_output_limit(OutputLimit::new(1.0, -1.0));
        assert_eq!(
            ladrc::LadrcFirstOrder::new(bad_limit).unwrap_err(),
            ConfigError::InvalidOutputLimit
        );
    }

    #[test]
    fn ladrc_bandwidth_tuning_sets_expected_gains() {
        let first = ladrc::LadrcFirstOrderConfig::from_bandwidth(0.001, 2.0, 12.0, 40.0);
        assert_close(first.kp, 12.0, 1.0e-6);
        assert_close(first.observer_beta1, 80.0, 1.0e-6);
        assert_close(first.observer_beta2, 1_600.0, 1.0e-3);

        let second = ladrc::LadrcSecondOrderConfig::from_bandwidth(0.001, 2.0, 12.0, 40.0);
        assert_close(second.kp, 144.0, 1.0e-6);
        assert_close(second.kd, 24.0, 1.0e-6);
        assert_close(second.observer_beta1, 120.0, 1.0e-6);
        assert_close(second.observer_beta2, 4_800.0, 1.0e-3);
        assert_close(second.observer_beta3, 64_000.0, 1.0e-2);
    }

    #[test]
    fn first_order_ladrc_tracks_with_constant_disturbance() {
        let dt = 0.001;
        let config = ladrc::LadrcFirstOrderConfig::from_bandwidth(dt, 1.0, 18.0, 80.0)
            .with_output_limit(OutputLimit::new(-20.0, 20.0));
        let mut controller = ladrc::LadrcFirstOrder::new(config).unwrap();

        let mut y = 0.0;
        for _ in 0..5_000 {
            let output = controller.update(1.0, y);
            let disturbance = 0.35;
            let y_dot = -0.45 * y + output.control + disturbance;
            y += dt * y_dot;
        }

        assert_close(y, 1.0, 0.03);
        assert_close(controller.estimate().output, y, 0.03);
    }

    #[test]
    fn second_order_ladrc_tracks_with_model_error_and_disturbance() {
        let dt = 0.001;
        let config = ladrc::LadrcSecondOrderConfig::from_bandwidth(dt, 1.0, 14.0, 70.0)
            .with_output_limit(OutputLimit::new(-30.0, 30.0));
        let mut controller = ladrc::LadrcSecondOrder::new(config).unwrap();

        let mut y = 0.0;
        let mut v = 0.0;

        for _ in 0..7_000 {
            let output = controller.update(1.0, y);
            let acceleration = -1.6 * v - 2.0 * y + output.control + 0.4;
            v += dt * acceleration;
            y += dt * v;
        }

        assert_close(y, 1.0, 0.04);
        assert_close(v, 0.0, 0.08);
        assert_close(controller.estimate().position, y, 0.04);
    }

    #[test]
    fn first_order_ladrc_accepts_explicit_variable_period() {
        let config = ladrc::LadrcFirstOrderConfig::from_bandwidth(0.001, 1.0, 18.0, 80.0)
            .with_output_limit(OutputLimit::new(-20.0, 20.0));
        let mut controller = ladrc::LadrcFirstOrder::new(config).unwrap();
        let periods = [0.0007, 0.0012, 0.0009, 0.0015, 0.0010];

        let mut y = 0.0;
        for step in 0..5_000 {
            let dt = periods[step % periods.len()];
            let output = controller.update_with_period(dt, 1.0, y).unwrap();
            let disturbance = 0.35;
            let y_dot = -0.45 * y + output.control + disturbance;
            y += dt * y_dot;
        }

        assert_close(y, 1.0, 0.04);
    }

    #[test]
    fn second_order_ladrc_update_at_handles_variable_periods() {
        let config = ladrc::LadrcSecondOrderConfig::from_bandwidth(0.001, 1.0, 14.0, 70.0)
            .with_output_limit(OutputLimit::new(-30.0, 30.0));
        let mut controller = ladrc::LadrcSecondOrder::new(config).unwrap();
        let periods = [0.0007, 0.0013, 0.0009, 0.0011, 0.0015];

        let mut now = 0.0;
        let mut y = 0.0;
        let mut v = 0.0;
        controller.reset_at(now, y).unwrap();

        for step in 0..7_000 {
            let dt = periods[step % periods.len()];
            now += dt;
            let output = controller.update_at(now, 1.0, y).unwrap();
            let acceleration = -1.6 * v - 2.0 * y + output.control + 0.4;
            v += dt * acceleration;
            y += dt * v;
        }

        assert_close(y, 1.0, 0.05);
        assert_close(v, 0.0, 0.10);
    }

    #[test]
    fn second_order_ladrc_update_at_millis_handles_large_uptime() {
        let config = ladrc::LadrcSecondOrderConfig::from_bandwidth(0.001, 1.0, 14.0, 70.0)
            .with_output_limit(OutputLimit::new(-30.0, 30.0));
        let mut controller = ladrc::LadrcSecondOrder::new(config).unwrap();
        let periods_ms = [1_u64, 2, 1, 1, 2];

        let mut now_ms = 50_000_000_u64;
        let mut y = 0.0;
        let mut v = 0.0;
        controller.reset_at_millis(now_ms, y);

        for step in 0..6_000 {
            let dt_ms = periods_ms[step % periods_ms.len()];
            let dt = dt_ms as Float * 0.001;
            now_ms += dt_ms;

            let output = controller.update_at_millis(now_ms, 1.0, y).unwrap();
            let acceleration = -1.6 * v - 2.0 * y + output.control + 0.4;
            v += dt * acceleration;
            y += dt * v;
        }

        assert_close(y, 1.0, 0.05);
        assert_close(v, 0.0, 0.10);
        assert_eq!(controller.last_update_at_millis(), Some(now_ms));
    }

    #[test]
    fn update_at_rejects_non_monotonic_time() {
        let config = ladrc::LadrcSecondOrderConfig::from_bandwidth(0.001, 1.0, 14.0, 70.0);
        let mut controller = ladrc::LadrcSecondOrder::new(config).unwrap();
        controller.reset_at(1.0, 0.0).unwrap();

        assert_eq!(
            controller.update_at(1.0, 1.0, 0.0).unwrap_err(),
            ConfigError::NonPositiveSamplePeriod
        );
        assert_eq!(
            controller.update_at(0.9, 1.0, 0.0).unwrap_err(),
            ConfigError::NonPositiveSamplePeriod
        );
    }

    #[test]
    fn update_at_millis_rejects_non_monotonic_time() {
        let config = ladrc::LadrcSecondOrderConfig::from_bandwidth(0.001, 1.0, 14.0, 70.0);
        let mut controller = ladrc::LadrcSecondOrder::new(config).unwrap();
        controller.reset_at_millis(1_000, 0.0);

        assert_eq!(
            controller.update_at_millis(1_000, 1.0, 0.0).unwrap_err(),
            ConfigError::NonPositiveSamplePeriod
        );
        assert_eq!(
            controller.update_at_millis(999, 1.0, 0.0).unwrap_err(),
            ConfigError::NonPositiveSamplePeriod
        );
    }
}
