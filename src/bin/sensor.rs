use chrono::{DateTime, Local};
use clap::Parser;

use csv_async::AsyncSerializer;
use metec_sensor::data::{CH4SensorData, N2OSensorData, SensorData};
use metec_sensor::error::*;
use metec_sensor::mavlink::Telem;
use metec_sensor::task::{handle_location, handle_sensor_data, lora_radio};
use tokio::fs::File;
use tokio::sync::{broadcast, watch};

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

    log::info!("Creating output directory at {:?}", args.output_directory);
    tokio::fs::create_dir_all(&args.output_directory).await?;

    let (sensor_tx, sensor_rx) = broadcast::channel::<SensorData>(16);
    let (location_tx, location_rx) = watch::channel(None);
    let mut handles = vec![];

    let mavlink = tokio::spawn(handle_location(telem, location_tx, sensor_rx));
    handles.push(mavlink);

    if args.sensor_a_enable {
        let output_file = File::create(args.get_output_file("ch4")).await?;
        let serializer = AsyncSerializer::from_writer(output_file);
        let sensor = tokio::spawn(handle_sensor_data::<CH4SensorData>(
            "A",
            args.sensor_a_port.clone(),
            args.sensor_a_baud,
            serializer,
            location_rx.clone(),
            sensor_tx.clone(),
        ));
        handles.push(sensor)
    }

    if args.sensor_b_enable {
        let output_file = File::create(args.get_output_file("n2o")).await?;
        let serializer = AsyncSerializer::from_writer(output_file);
        let sensor = tokio::spawn(handle_sensor_data::<N2OSensorData>(
            "B",
            args.sensor_b_port.clone(),
            args.sensor_b_baud,
            serializer,
            location_rx.clone(),
            sensor_tx.clone(),
        ));
        handles.push(sensor)
    }

    if args.lora_radio_enable {
        let lora = tokio::spawn(lora_radio(
            args.lora_radio_port.to_owned(),
            args.lora_radio_baud,
            sensor_tx.subscribe(),
        ));
        handles.push(lora)
    };

    futures_util::future::try_join_all(handles).await?;

    Ok(())
}
