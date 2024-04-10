use std::time::Duration;

use crate::data::{open_serial_port, Sensor, SensorData};
use crate::error::*;
use crate::{data::DroneLocation, mavlink::Telem};
use csv_async::AsyncSerializer;
use mavlink::common::MavMessage;
use serde::de::DeserializeOwned;
use serde::Serialize;
use tokio::sync::broadcast;

use tokio::{fs::File, io::AsyncWriteExt, sync::watch};
use tokio_serial::SerialPortBuilderExt;
use tokio_stream::wrappers::IntervalStream;
use tokio_stream::StreamExt;

pub async fn handle_location(
    telem: Telem,
    location_tx: watch::Sender<Option<DroneLocation>>,
    sensor_rx: broadcast::Receiver<SensorData>,
) -> Result<()> {
    let mut sensor_rx = sensor_rx;
    let mut telem = telem;

    let heartbeat_message = telem.heartbeat_message();
    let mut heartbeat_stream = IntervalStream::new(tokio::time::interval(Duration::from_secs(1)))
        .map(move |_| heartbeat_message.clone());

    let boot_time = tokio::time::Instant::now();

    loop {
        tokio::select! {
            Some(Ok((_header, message))) = telem.recv() => {
                match message {
                    // Grab global position
                    MavMessage::GLOBAL_POSITION_INT(location) => {
                        let current_position = Some(DroneLocation::from(location));
                        log::debug!("Received location: {current_position:?}");

                        location_tx.send(current_position)?;
                    }
                    msg => log::trace!("Mavlink Received: {msg:?}"),
                }
            },
            Some(heartbeat) = heartbeat_stream.next() => {
                telem.send(heartbeat).await?;
            },
            Ok(data) = sensor_rx.recv() => {
                let messages = data.send_mavlink(data.sensor_name(), boot_time);
                for m in messages {
                    telem.send(m).await?;
                }
            }

        }
    }
}

pub async fn lora_radio(
    port: String,
    baud: u32,
    sensor_data: broadcast::Receiver<SensorData>,
) -> Result<()> {
    log::debug!("Opening LORA serial port!");
    let mut serial = tokio_serial::new(port, baud).open_native_async()?;
    let mut sensor_data = sensor_data;

    loop {
        let data = sensor_data.recv().await?;
        let lora_str = match data {
            SensorData::CH4(d) => format!("A,{}", csv_serialize(&d).await?),
            SensorData::N2O(d) => format!("B,{}", csv_serialize(&d).await?),
        };

        serial.write_all(lora_str.as_bytes()).await?;
    }
}

async fn csv_serialize<S: Serialize>(data: &S) -> Result<String> {
    let mut writer = csv_async::AsyncWriterBuilder::new()
        .has_headers(false)
        .create_serializer(vec![]);

    writer.serialize(data).await?;

    Ok(String::from_utf8(writer.into_inner().await.unwrap())?)
}

pub async fn file_writer(
    sensor_a: &mut Option<AsyncSerializer<File>>,
    sensor_b: &mut Option<AsyncSerializer<File>>,
    sensor_data: broadcast::Receiver<SensorData>,
) -> Result<()> {
    let mut sensor_data = sensor_data;

    loop {
        let data = sensor_data.recv().await?;

        match data {
            SensorData::CH4(d) => {
                if let Some(ref mut writer) = sensor_a {
                    writer.serialize(d).await?;
                }
            }
            SensorData::N2O(d) => {
                if let Some(ref mut writer) = sensor_b {
                    writer.serialize(d).await?;
                }
            }
        }
    }
}

pub async fn handle_sensor_data<
    S: Serialize + DeserializeOwned + Sensor + Send + Sync + std::fmt::Debug,
>(
    sensor_name: &str,
    port: String,
    baud: u32,
    serializer: AsyncSerializer<File>,
    location_rx: watch::Receiver<Option<DroneLocation>>,
    sensor_tx: broadcast::Sender<SensorData>,
) -> Result<()>
where
    SensorData: From<S>,
{
    let mut serializer = serializer;
    let sensor_name = sensor_name.to_owned();
    let port = port.to_owned();

    log::info!("Opening {sensor_name} port   -> {port}:{baud}");
    let mut sensor = open_serial_port::<S>(&port, baud)?;

    loop {
        match sensor.next().await {
            Some(Ok(Some(mut data))) => {
                log::info!("Received Data from Sensor: {sensor_name}: {data:?}");
                if let Some(location) = *location_rx.borrow() {
                    data.set_location(location);
                }

                serializer.serialize(&data).await?;
                sensor_tx.send(SensorData::from(data))?;
            }
            Some(Err(e)) => log::error!("Error while reading sensor {}: {}", sensor_name, e),
            _ => (),
        }
    }
}
