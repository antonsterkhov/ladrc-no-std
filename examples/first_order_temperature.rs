use ladrc_no_std::{LadrcFirstOrder, LadrcFirstOrderConfig, OutputLimit};

fn main() -> Result<(), ladrc_no_std::ConfigError> {
    let dt = 0.01;
    let target_temperature = 55.0;

    let config = LadrcFirstOrderConfig::from_bandwidth(
        dt, 2.0, // one unit of heater command gives roughly +2 C/s initially
        0.8, 4.0,
    )
    .with_output_limit(OutputLimit::new(0.0, 1.0));

    let mut controller = LadrcFirstOrder::new(config)?;
    let mut temperature = 20.0;
    let ambient = 20.0;
    let mut control = 0.0;

    for step in 0..6_000 {
        let out = controller.update(target_temperature, temperature);
        control = out.control;

        let open_window_disturbance = if step > 3_000 { -0.35 } else { 0.0 };
        let plant_cooling = -0.04 * (temperature - ambient);
        let plant_heating = 2.0 * control;

        temperature += dt * (plant_cooling + plant_heating + open_window_disturbance);
    }

    println!("target temperature: {target_temperature:.2} C");
    println!("final temperature:  {temperature:.2} C");
    println!("final command:      {:.1} %", control * 100.0);

    Ok(())
}
