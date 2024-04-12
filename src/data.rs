use crate::error::*;
use crate::mavlink::named_float_message;

use enum_dispatch::enum_dispatch;
use futures::Stream;
use futures_util::StreamExt;
use mavlink::common::{MavMessage, GLOBAL_POSITION_INT_DATA};
use serde::de::DeserializeOwned;
use serde::de::Deserializer;
use serde::{Deserialize, Serialize};
use tokio::io::AsyncRead;

use tokio::time::Instant;
use tokio_serial::SerialPortBuilderExt;
use tokio_util::codec::{FramedRead, LinesCodec, LinesCodecError};

#[enum_dispatch]
pub trait Sensor: Send {
    fn set_location(&mut self, location: DroneLocation);
    fn send_mavlink(&self, sensor_name: &str, boot_time: Instant) -> Vec<MavMessage>;
}

#[enum_dispatch(Sensor)]
#[derive(Clone)]
pub enum SensorData {
    CH4(CH4SensorData),
    N2O(N2OSensorData),
}

impl SensorData {
    pub fn sensor_name(&self) -> &str {
        match self {
            SensorData::CH4(_) => "A",
            SensorData::N2O(_) => "B",
        }
    }
}

#[derive(Serialize, Deserialize, Default, Debug, Clone)]
pub struct CH4SensorData {
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

    #[serde(rename = "Latitude", deserialize_with = "deserialize_no_data")]
    pub lat: Option<f64>,

    #[serde(rename = "Latitude", deserialize_with = "deserialize_no_data")]
    pub lon: Option<f64>,

    #[serde(skip_deserializing)]
    pub drone_lat: Option<f64>,

    #[serde(skip_deserializing)]
    pub drone_lon: Option<f64>,

    #[serde(skip_deserializing)]
    pub drone_alt: Option<f64>,
}

impl Sensor for CH4SensorData {
    fn set_location(&mut self, location: DroneLocation) {
        self.drone_lat = Some(location.latitude);
        self.drone_lon = Some(location.longitude);
        self.drone_alt = Some(location.altitude);
    }

    fn send_mavlink(&self, _sensor_name: &str, boot_time: Instant) -> Vec<MavMessage> {
        vec![
            named_float_message("CH4", self.ch4, boot_time),
            named_float_message("C2H6", self.c2h6, boot_time),
        ]
    }
}

#[derive(Serialize, Deserialize, Default, Debug, Clone)]
pub struct N2OSensorData {
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

    #[serde(rename = "CO2 (ppm)")]
    pub co2: f32,

    #[serde(rename = "H2O (ppm)")]
    pub h2o: f32,

    #[serde(rename = "N2O (ppm)")]
    pub n2o: f32,

    #[serde(rename = "Battery Charge (V)")]
    pub battery: i32,

    #[serde(rename = "Power Input (mV)")]
    pub power_input: i32,

    #[serde(rename = "Current (mA)")]
    pub current: i32,

    #[serde(rename = "SOC (%)")]
    pub soc: i32,

    #[serde(rename = "Latitude", deserialize_with = "deserialize_no_data")]
    pub lat: Option<f64>,

    #[serde(rename = "Latitude", deserialize_with = "deserialize_no_data")]
    pub lon: Option<f64>,

    #[serde(skip_deserializing)]
    pub drone_lat: Option<f64>,

    #[serde(skip_deserializing)]
    pub drone_lon: Option<f64>,

    #[serde(skip_deserializing)]
    pub drone_alt: Option<f64>,
}

impl Sensor for N2OSensorData {
    fn set_location(&mut self, location: DroneLocation) {
        self.drone_lat = Some(location.latitude);
        self.drone_lon = Some(location.longitude);
        self.drone_alt = Some(location.altitude);
    }

    fn send_mavlink(&self, _sensor_name: &str, boot_time: Instant) -> Vec<MavMessage> {
        vec![named_float_message("N2O", self.n2o, boot_time)]
    }
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

async fn line_to_data<D: DeserializeOwned>(
    line: std::result::Result<String, LinesCodecError>,
) -> Result<Option<D>> {
    let data = line?;

    let mut reader = csv_async::AsyncReaderBuilder::new()
        .has_headers(false)
        .create_deserializer(data.as_bytes());

    let mut dsr = reader.deserialize::<D>();

    match dsr.next().await {
        Some(r) => Ok(Some(r?)),
        _ => Ok(None),
    }
}

pub fn sensor_data_framed_reader<D: DeserializeOwned, T: AsyncRead>(
    reader: T,
) -> impl Stream<Item = Result<Option<D>>> {
    let stream = FramedRead::new(reader, LinesCodec::default()).then(line_to_data);
    Box::pin(stream)
}

pub fn open_serial_port<D: DeserializeOwned>(
    port: &str,
    baud: u32,
) -> Result<impl Stream<Item = Result<Option<D>>>> {
    let serial = tokio_serial::new(port, baud).open_native_async()?;
    let data = sensor_data_framed_reader(serial);

    Ok(data)
}

// custom deserializer function
fn deserialize_no_data<'de, D, T: Deserialize<'de>>(
    deserializer: D,
) -> std::result::Result<Option<T>, D::Error>
where
    D: Deserializer<'de>,
{
    #[derive(Deserialize)]
    #[serde(untagged)]
    enum MaybeNA<U> {
        Value(Option<U>),
        NAString(String),
    }

    let value: MaybeNA<T> = Deserialize::deserialize(deserializer)?;
    match value {
        MaybeNA::Value(value) => Ok(value),
        MaybeNA::NAString(string) => {
            if string == "No Data" {
                Ok(None)
            } else {
                Err(serde::de::Error::custom("Unexpected string"))
            }
        }
    }
}
