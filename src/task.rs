use std::path::PathBuf;
use std::time::Duration;

use crate::data::{open_serial_port, SensorData};
use crate::error::*;
use crate::{data::DroneLocation, mavlink::Telem};
use csv_async::AsyncSerializer;
use mavlink::common::MavMessage;
use serde::de::DeserializeOwned;
use serde::{Deserialize, Serialize};
use tokio::time::Instant;
use tokio::{
    fs::File,
    io::AsyncWriteExt,
    sync::{mpsc, watch},
    task::JoinHandle,
};
use tokio_serial::SerialPortBuilderExt;
use tokio_stream::wrappers::IntervalStream;
use tokio_stream::StreamExt;

pub fn handle_location(
    telem: Telem,
) -> (
    tokio::task::JoinHandle<Result<()>>,
    mpsc::Sender<MavMessage>,
    watch::Receiver<Option<DroneLocation>>,
) {
    let (mavlink_tx, mut mavlink_rx) = mpsc::channel(32);
    let (location_tx, location_rx) = watch::channel(None);
    let heartbeat_message = telem.heartbeat_message();
    let mut heartbeat_stream = IntervalStream::new(tokio::time::interval(Duration::from_secs(1)))
        .map(move |_| heartbeat_message.clone());
    let mut telem = telem;

    let task = tokio::spawn(async move {
        loop {
            tokio::select! {
                Some(Ok((_header, message))) = telem.recv() => {
                    match message {
                        // Grab global position
                        MavMessage::GLOBAL_POSITION_INT(location) => {
                            let current_position = Some(DroneLocation::from(location));

                            location_tx.send(current_position)?;
                        }
                        msg => log::trace!("Mavlink Received: {msg:?}"),
                    }
                },
                Some(message) = mavlink_rx.recv() => {
                    telem.send(message).await?;
                },
                Some(heartbeat) = heartbeat_stream.next() => {
                    telem.send(heartbeat).await?;
                }
            }
        }
    });

    (task, mavlink_tx, location_rx)
}

pub fn handle_lora(
    port: &str,
    baud: u32,
    enabled: bool,
) -> (JoinHandle<Result<()>>, mpsc::Sender<String>) {
    let (lora_tx, mut lora_rx) = mpsc::channel::<String>(32);
    let port = port.to_owned();

    let task = tokio::spawn(async move {
        if !enabled {
            lora_rx.close();
            let future = futures::future::pending();
            let () = future.await;
            unreachable!()
        }

        let mut serial = tokio_serial::new(port, baud).open_native_async()?;

        loop {
            while let Some(data) = lora_rx.recv().await {
                serial.write_all(data.as_bytes()).await?;
            }
        }
    });

    (task, lora_tx)
}

async fn csv_serialize<S: Serialize>(data: &S) -> Result<String> {
    let mut writer = csv_async::AsyncWriterBuilder::new()
        .has_headers(false)
        .create_serializer(vec![]);

    writer.serialize(data).await?;

    Ok(String::from_utf8(writer.into_inner().await.unwrap())?)
}

pub async fn handle_sensor_data<S: Serialize + DeserializeOwned + SensorData + Send + Sync>(
    sensor_name: &str,
    port: &str,
    baud: u32,
    enabled: bool,
    boot_time: Instant,
    filename: PathBuf,
    location_rx: watch::Receiver<Option<DroneLocation>>,
    mavlink_tx: mpsc::Sender<MavMessage>,
    lora_tx: mpsc::Sender<String>,
) -> JoinHandle<Result<()>> {
    let sensor_name = sensor_name.to_owned();
    let port = port.to_owned();
    let filename = filename.to_owned();

    tokio::spawn(async move {
        if !enabled {
            let future = futures::future::pending();
            let () = future.await;
            unreachable!()
        }

        let mut sensor = open_serial_port::<S>(&port, baud)?;
        let mut log = AsyncSerializer::from_writer(File::create(filename).await?);

        loop {
            match sensor.next().await {
                Some(Ok(Some(mut data))) => {
                    // Grab the most up to date location
                    if let Some(location) = *location_rx.borrow() {
                        data.set_location(location);
                    }

                    // Send data to LORA
                    if !lora_tx.is_closed() {
                        let lora_data = format!("{sensor_name},{}", csv_serialize(&data).await?);
                        let _ = lora_tx.send(lora_data).await;
                    }

                    // Send data over mavlink
                    data.send_mavlink(&sensor_name, boot_time, &mavlink_tx)
                        .await?;
                    log.serialize(data).await?;
                }
                Some(Err(e)) => log::error!("Error while reading sensor {}: {}", sensor_name, e),
                _ => (),
            }
        }
    })
}
