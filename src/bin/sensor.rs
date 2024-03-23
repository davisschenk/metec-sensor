use chrono::{DateTime, Local};
use clap::Parser;
use csv_async::AsyncSerializer;
use futures::stream::StreamExt;
use mavlink::common::MavMessage;
use metec_sensor::data::{handle_sensor_data, open_serial_port, DroneLocation};
use metec_sensor::error::*;
use metec_sensor::mavlink::{heartbeat_stream, Telem};
use std::path::PathBuf;
use std::time::Duration;
use tokio::fs::File;

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

    log::info!("Opening Telem Serial Port");
    let mut telem = Telem::try_new(
        &args.mavlink_port,
        args.mavlink_baud,
        args.mavlink_system_id,
        args.mavlink_component_id,
    )?;

    log::info!("Creating output directory at {:?}", args.output_directory);
    tokio::fs::create_dir_all(&args.output_directory).await?;

    let (mut sensor_a, mut sensor_a_log) = if args.sensor_a_enable {
        log::info!("Opening Serial Port A: {}:{}", args.sensor_a_port, args.sensor_a_baud);

        let sensor_a = open_serial_port(&args.sensor_a_port, args.sensor_a_baud)?;

        let filename_a = args.get_output_file("sensor_a");
        log::info!("Writing log A at {:?}", filename_a);
        let sensor_a_log = AsyncSerializer::from_writer(File::create(filename_a).await?);

        (Some(sensor_a), Some(sensor_a_log))
    } else {
        (None, None)
    };

    let (mut sensor_b, mut sensor_b_log) = if args.sensor_b_enable {
        log::info!("Opening Serial Port B: {}:{}", args.sensor_b_port, args.sensor_b_baud);
        let sensor_b = open_serial_port(&args.sensor_b_port, args.sensor_b_baud)?;

        let filename_b = args.get_output_file("sensor_b");
        log::info!("Writing log B at {:?}", filename_b);
        let sensor_b_log = AsyncSerializer::from_writer(File::create(filename_b).await?);

        (Some(sensor_b), Some(sensor_b_log))
    } else {
        (None, None)
    };

    let boot_time = tokio::time::Instant::now();
    let mut heartbeat_stream = heartbeat_stream(&telem, Duration::from_secs(1));
    let mut current_position: Option<DroneLocation> = None;

    log::info!("Starting main loop");
    loop {
        // Check if we need to send a heartbeat
        if let Some(heartbeat_message) = heartbeat_stream.next().await {
            log::trace!("Sending heartbeat");
            telem.send(heartbeat_message).await?;
        };

        // Check if we have receieved any mavlink messages
        if let Some(Ok((_header, message))) = telem.recv().await {
            match message {
                MavMessage::HEARTBEAT(_) => (),
                MavMessage::GLOBAL_POSITION_INT(location) => {
                    current_position = Some(DroneLocation::from(location));

                    if let Some(loc) = current_position {
                        log::debug!(
                            "Current position: {} {} {}",
                            loc.longitude,
                            loc.latitude,
                            loc.altitude
                        );
                    }
                }
                msg => log::trace!("Recv: {msg:?}"),
            }
        };

        // Check if we need to handle sensor A
        if let (Some(ref mut sensor), Some(ref mut sensor_log)) = (&mut sensor_a, &mut sensor_a_log)
        {
            if let Some(sensor_result) = sensor.next().await {
                handle_sensor_data(
                    &mut telem,
                    sensor_log,
                    &current_position,
                    sensor_result,
                    boot_time,
                    "A"
                )
                .await?;
            }
        }

        // Check if we need to handle sensor B
        if let (Some(ref mut sensor), Some(ref mut sensor_log)) = (&mut sensor_b, &mut sensor_b_log)
        {
            if let Some(sensor_result) = sensor.next().await {
                handle_sensor_data(
                    &mut telem,
                    sensor_log,
                    &current_position,
                    sensor_result,
                    boot_time,
                    "B"
                )
                .await?;
            }
        }

        // Wait a little bit, helps to prevent any blocking issues and give the cpu time to do other things
        tokio::time::sleep(Duration::from_millis(1)).await;
    }
}
