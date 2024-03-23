use chrono::{DateTime, Local};
use clap::Parser;
use csv_async::{AsyncWriter, AsyncSerializer};
use futures::stream::StreamExt;
use mavlink::common::MavMessage;
use metec_sensor::data::{sensor_data_framed_reader, DroneLocation, handle_sensor_data};
use metec_sensor::error::*;
use metec_sensor::mavlink::{heartbeat_stream, Telem};
use tokio::fs::File;
use std::path::PathBuf;
use std::time::Duration;
use tokio_serial::SerialPortBuilderExt;

/// Program for reading, storing and transmitting sensor data
#[derive(Parser, Debug)]
#[command(version, about)]
struct Args {
    /// Serial Port for Sensor A
    #[arg(long, env)]
    sensor_a_port: PathBuf,

    /// Baud Rate for Sensor A
    #[arg(long, env)]
    sensor_a_baud: u32,

    /// Serial Port for Sensor B
    #[arg(long, env)]
    sensor_b_port: PathBuf,

    /// Baud Rate for Sensor B
    #[arg(long, env)]
    sensor_b_baud: u32,

    /// Serial Port for Mavlink
    #[arg(long, env)]
    mavlink_port: PathBuf,

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
        args.mavlink_port.to_str().unwrap(),
        args.mavlink_baud,
        args.mavlink_system_id,
        args.mavlink_component_id,
    )?;

    log::info!("Creating output directory at {:?}", args.output_directory);
    tokio::fs::create_dir_all(&args.output_directory).await?;

    log::info!("Opening Serial Port A: {:?}:{}", args.sensor_a_port, args.sensor_a_baud);
    let sensor_a_serial = tokio_serial::new(args.sensor_a_port.to_str().unwrap(), args.sensor_a_baud).open_native_async()?;
    let mut sensor_a_stream = sensor_data_framed_reader(sensor_a_serial);

    let filename_a = args.get_output_file("sensor_a");
    log::info!("Writing log at {:?}", filename_a);
    let mut sensor_a_log = AsyncSerializer::from_writer(File::create(filename_a).await?);

    // log::info!("Opening Serial Port B: {}:{}", args.sensor_b_port, args.sensor_b_baud);
    // let sensor_b_serial = tokio_serial::new(args.sensor_b_port.to_str().unwrap(), args.sensor_b_baud).open_native_async()?;
    // let mut sensor_b_stream = sensor_data_framed_reader(sensor_b_serial);
    // let mut sensor_b_log = AsyncSerializer::from_writer(File::create(args.get_output_file("sensor_b")).await?);

    let boot_time = tokio::time::Instant::now();
    let mut heartbeat_stream = heartbeat_stream(&telem, Duration::from_secs(1));
    let mut current_position: Option<DroneLocation> = None;
    let mut counter = 0f32;

    log::info!("Starting main loop");
    loop {
        tokio::select! {
            Some(heartbeat_message) = heartbeat_stream.next() => {
                log::trace!("Sending heartbeat");
                telem.send(heartbeat_message).await?;
                counter = counter + 1.0;
            },
            Some(Ok((_header, message))) = telem.recv() => {
                match message {
                    MavMessage::HEARTBEAT(_) => (),
                    MavMessage::GLOBAL_POSITION_INT(location) => {
                        current_position = Some(DroneLocation::from(location));

                        if let Some(loc) = current_position {
                            log::debug!("Current position: {} {} {}", loc.longitude, loc.latitude, loc.altitude);
                        }
                    },
                    msg @ _ => log::trace!("Recv: {msg:?}")
                }
            },
            Some(sensor_result) = sensor_a_stream.next() => {
                handle_sensor_data(&mut telem, &mut sensor_a_log, &current_position, sensor_result, boot_time).await?;
            },
            // Some(sensor_result) = sensor_b_stream.next() => {
            //     handle_sensor_data(&mut telem, &mut sensor_b_log, &current_position, sensor_result).await?;
            // }
        }

        tokio::time::sleep(Duration::from_millis(1)).await;
    }
}
