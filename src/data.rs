use crate::{error::*, mavlink::Telem};
use csv_async::AsyncSerializer;
use futures::Stream;
use futures_util::{StreamExt};
use mavlink::common::GLOBAL_POSITION_INT_DATA;
use serde::{Deserialize, Serialize};
use tokio::io::AsyncWriteExt;
use tokio::time::Instant;
use tokio::{fs::File, io::AsyncRead};
use tokio_serial::SerialPortBuilderExt;
use tokio_util::codec::{FramedRead, LinesCodec};

// 03/20/2024 12:14:34.580
// 0
// 239.983
// 28.1712
// 39.2037
// 28.1712
// 2.14587
// 8936.47
// 8.58853
// 0.988286
// 0.104494
// 11195
// 14599
// 558
// 35
// 40.5954666138
// -105.1388320923
#[derive(Serialize, Deserialize, Default, Debug)]
pub struct SensorData {
    #[serde(rename = "Time Stamp")]
    pub time_stamp: String,

    #[serde(rename = "Inlet Number")]
    pub inlet_number: u32,

    #[serde(rename = "P (mbars)")]
    pub p: f32,

    #[serde(rename = "T0 (degC)")]
    pub t0: f32,

    #[serde(rename = "T5 (degC)")]
    pub t5: f32,

    #[serde(rename = "Tgas(degC)")]
    pub t_gas: f32,

    #[serde(rename = "CH4 (ppm)")]
    pub ch4: f32,

    #[serde(rename = "H2O (ppm)")]
    pub h2o: f32,

    #[serde(rename = "C2H6 (ppb)")]
    pub c2h6: f32,

    #[serde(rename = "R")]
    pub r: f32,

    #[serde(rename = "C2/C1")]
    pub c2_c1: f32,

    #[serde(rename = "Battery Charge (V)")]
    pub battery: i32,

    #[serde(rename = "Power Input (mV)")]
    pub power_input: i32,

    #[serde(rename = "Current (mA)")]
    pub current: i32,

    #[serde(rename = "SOC (%)")]
    pub soc: i32,

    #[serde(rename = "Latitude")]
    pub lat: f64,

    #[serde(rename = "Longitude")]
    pub lon: f64,

    #[serde(skip_deserializing)]
    pub drone_lat: Option<f64>,

    #[serde(skip_deserializing)]
    pub drone_lon: Option<f64>,

    #[serde(skip_deserializing)]
    pub drone_alt: Option<f64>,
}

#[derive(Debug, Clone, Copy, Serialize)]
pub struct DroneLocation {
    pub latitude: f64,
    pub longitude: f64,
    pub altitude: f64,
}

fn loc_to_float(loc: i32) -> f64 {
    loc as f64 / 10_000_000f64
}

impl From<GLOBAL_POSITION_INT_DATA> for DroneLocation {
    fn from(value: GLOBAL_POSITION_INT_DATA) -> Self {
        DroneLocation {
            latitude: loc_to_float(value.lat),
            longitude: loc_to_float(value.lon),
            altitude: value.relative_alt as f64 / 1000f64,
        }
    }
}

pub fn sensor_data_framed_reader<T: AsyncRead>(
    reader: T,
) -> impl Stream<Item = Result<Option<SensorData>>> {
    let stream = FramedRead::new(reader, LinesCodec::default()).then(|line| async {
        let data = line?;

        let mut reader = csv_async::AsyncReaderBuilder::new()
            .has_headers(false)
            .create_deserializer(data.as_bytes());

        let mut dsr = reader.deserialize::<SensorData>();

        if let Some(result) = dsr.next().await {
            match result {
                Ok(d) => Ok(Some(d)),
                Err(e) => {
                    log::error!("Sensor Error: {e:?}");
                    Ok(None)
                }
            }
        } else {
            Ok(None)
        }
    });

    Box::pin(stream)
}

pub fn open_serial_port(
    port: &str,
    baud: u32,
) -> Result<impl Stream<Item = Result<Option<SensorData>>>> {
    let serial = tokio_serial::new(port, baud).open_native_async()?;
    let data = sensor_data_framed_reader(serial);

    Ok(data)
}

pub async fn handle_sensor_data(
    mavlink: &mut Telem,
    csv: &mut AsyncSerializer<File>,
    current_position: &Option<DroneLocation>,
    sensor_result: Result<Option<SensorData>>,
    boot_time: Instant,
    sensor_name: &str,
    lora: &mut Option<impl AsyncWriteExt + Unpin>
) -> Result<()> {
    let mut sensor: SensorData = if let Ok(Some(sensor)) = sensor_result {
        sensor
    } else {
        log::trace!("Did not receieve sensor data: {sensor_result:?}");
        return Ok(());
    };

    if let Some(location) = current_position {
        sensor.drone_lat = Some(location.latitude);
        sensor.drone_lon = Some(location.longitude);
        sensor.drone_alt = Some(location.altitude);
    }

    if let Some(ref mut lora) = lora {
        let mut writer = csv_async::AsyncWriterBuilder::new()
            .has_headers(false)
            .create_serializer(vec![]);

        writer.serialize(&sensor).await?;

        let data = String::from_utf8(writer.into_inner().await.unwrap()).unwrap();
        lora.write(format!("{sensor_name},{}", data).as_bytes()).await?;
    }

    log::debug!("Sensor Data: {sensor:?}");

    mavlink.send_float(&format!("CH4{sensor_name}"), sensor.ch4, boot_time).await?;
    mavlink.send_float(&format!("C2H6{sensor_name}"), sensor.c2h6, boot_time).await?;

    csv.serialize(sensor).await?;
    csv.flush().await?;

    Ok(())
}
