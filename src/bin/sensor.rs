use chrono::{DateTime, Local};
use clap::Parser;

use metec_sensor::data::{CH4SensorData, N2OSensorData};
use metec_sensor::error::*;
use metec_sensor::mavlink::Telem;
use metec_sensor::task::{handle_location, handle_lora, handle_sensor_data};

use std::path::PathBuf;

/// Program for reading, storing and transmitting sensor data
#[derive(Parser, Debug)]
#[command(version, about)]
struct Args {
    #[arg(long, env, default_value_t = true)]
    sensor_a_enable: bool,

    /// Serial Port for Sensor A
    #[arg(long, env)]
    sensor_a_port: String,

    /// Baud Rate for Sensor A
    #[arg(long, env)]
    sensor_a_baud: u32,

    #[arg(long, env, default_value_t = true)]
    sensor_b_enable: bool,

    /// Serial Port for Sensor B
    #[arg(long, env)]
    sensor_b_port: String,

    /// Baud Rate for Sensor B
    #[arg(long, env)]
    sensor_b_baud: u32,

    /// Serial Port for Mavlink
    #[arg(long, env)]
    mavlink_port: String,

    /// Baud Rate for Mavlink
    #[arg(long, env)]
    mavlink_baud: u32,

    /// System ID for Mavlink
    #[arg(long, env)]
    mavlink_system_id: u8,

    /// Component ID for Mavlink
    #[arg(long, env)]
    mavlink_component_id: u8,

    /// Directory for storing log files
    #[arg(long, env)]
    output_directory: PathBuf,

    #[arg(long, env)]
    lora_radio_enable: bool,

    #[arg(long, env)]
    lora_radio_port: String,

    #[arg(long, env)]
    lora_radio_baud: u32,
}

impl Args {
    pub fn get_output_file(&self, postfix: &str) -> PathBuf {
        let now: DateTime<Local> = Local::now();
        let time = now.format("%F_%H%M%S");

        let filename = format!("{time}_{postfix}.csv");

        self.output_directory.join(filename)
    }
}

#[tokio::main]
async fn main() -> Result<()> {
    dotenv::dotenv().ok();
    pretty_env_logger::init();

    let args = Args::parse();

    log::info!(
        "Opening mavlink port -> {}:{}",
        args.mavlink_port,
        args.mavlink_baud
    );
    let telem = Telem::try_new(
        &args.mavlink_port,
        args.mavlink_baud,
        args.mavlink_system_id,
        args.mavlink_component_id,
    )?;

    let (mavlink_task, mavlink_tx, location_rx) = handle_location(telem);
    let (lora_task, lora_tx) = handle_lora(
        &args.lora_radio_port,
        args.lora_radio_baud,
        args.lora_radio_enable,
    );

    let boot_time = tokio::time::Instant::now();

    log::info!("Starting sensors");
    let _ = tokio::join!(
        mavlink_task,
        lora_task,
        handle_sensor_data::<CH4SensorData>(
            "A",
            &args.sensor_a_port,
            args.sensor_a_baud,
            args.sensor_a_enable,
            boot_time,
            args.get_output_file("A"),
            location_rx.clone(),
            mavlink_tx.clone(),
            lora_tx.clone()
        ),
        handle_sensor_data::<N2OSensorData>(
            "B",
            &args.sensor_a_port,
            args.sensor_a_baud,
            args.sensor_a_enable,
            boot_time,
            args.get_output_file("B"),
            location_rx.clone(),
            mavlink_tx.clone(),
            lora_tx.clone()
        )
    );

    unreachable!()
}
