use ladrc_no_std::{LadrcSecondOrder, LadrcSecondOrderConfig, OutputLimit};

fn main() -> Result<(), ladrc_no_std::ConfigError> {
    let nominal_dt = 0.001;
    let target_position = 1.0;

    let config = LadrcSecondOrderConfig::from_bandwidth(nominal_dt, 2.0, 12.0, 60.0)
        .with_output_limit(OutputLimit::new(-5.0, 5.0));

    let mut controller = LadrcSecondOrder::new(config)?;
    let mut now_ms = 0_u64;
    let mut position = 0.0;
    let mut velocity = 0.0;
    let mut control = 0.0;

    controller.reset_at_millis(now_ms, position);

    let poll_periods_ms = [1_u64, 2, 1, 1, 2];

    for step in 0..5_000 {
        let actual_dt_ms = poll_periods_ms[step % poll_periods_ms.len()];
        let actual_dt = actual_dt_ms as f32 * 0.001;
        now_ms += actual_dt_ms;

        let out = controller.update_at_millis(now_ms, target_position, position)?;
        control = out.control;

        let load_disturbance = if step > 2_000 { -0.8 } else { 0.0 };
        let acceleration = -1.2 * velocity - 4.0 * position + 2.0 * control + load_disturbance;

        velocity += actual_dt * acceleration;
        position += actual_dt * velocity;
    }

    println!("target position: {target_position:.3}");
    println!("final position:  {position:.3}");
    println!("final velocity:  {velocity:.3}");
    println!("final command:   {control:.3}");
    println!("final time:      {:.3} s", now_ms as f32 * 0.001);

    Ok(())
}
