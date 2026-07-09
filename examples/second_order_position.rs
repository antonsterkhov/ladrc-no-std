use ladrc_no_std::{LadrcSecondOrder, LadrcSecondOrderConfig, OutputLimit};

fn main() -> Result<(), ladrc_no_std::ConfigError> {
    let dt = 0.001;
    let target_position = 1.0;

    let config = LadrcSecondOrderConfig::from_bandwidth(
        dt, 2.0, // one unit of command gives roughly +2 position-units/s^2
        12.0, 60.0,
    )
    .with_output_limit(OutputLimit::new(-5.0, 5.0));

    let mut controller = LadrcSecondOrder::new(config)?;
    let mut position = 0.0;
    let mut velocity = 0.0;
    let mut control = 0.0;

    for step in 0..5_000 {
        let out = controller.update(target_position, position);
        control = out.control;

        let load_disturbance = if step > 2_000 { -0.8 } else { 0.0 };
        let acceleration = -1.2 * velocity - 4.0 * position + 2.0 * control + load_disturbance;

        velocity += dt * acceleration;
        position += dt * velocity;
    }

    println!("target position: {target_position:.3}");
    println!("final position:  {position:.3}");
    println!("final velocity:  {velocity:.3}");
    println!("final command:   {control:.3}");

    Ok(())
}
