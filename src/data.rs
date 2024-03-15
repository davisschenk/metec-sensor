use crate::{error::*, mavlink::Telem};
use csv_async::{AsyncWriter, AsyncSerializer};
use futures::Stream;
use futures_util::StreamExt;

use mavlink::common::GLOBAL_POSITION_INT_DATA;
use serde::{Deserialize, Serialize};
use tokio::{fs::File, io::AsyncRead, pin};
use tokio_util::codec::{FramedRead, LinesCodec};

#[derive(Serialize, Deserialize, Default)]
pub struct SensorData {
    #[serde(rename = "Time Stamp")]
    pub time_stamp: chrono::NaiveDateTime,

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

    #[serde(rename = "Laser PID Readout")]
    pub laser_pid: f32,

    #[serde(rename = "Det PID Readout")]
    pub det_pid: f32,

    #[serde(rename = "win0Fit0")]
    pub win_0_fit_0: f32,

    #[serde(rename = "win0Fit1")]
    pub win_0_fit_1: f32,

    #[serde(rename = "win0Fit2")]
    pub win_0_fit_2: f32,

    #[serde(rename = "win0Fit3")]
    pub win_0_fit_3: f32,

    #[serde(rename = "win0Fit4")]
    pub win_0_fit_4: f32,

    #[serde(rename = "win0Fit5")]
    pub win_0_fit_5: f32,

    #[serde(rename = "win0Fit6")]
    pub win_0_fit_6: f32,

    #[serde(rename = "win0Fit7")]
    pub win_0_fit_7: f32,

    #[serde(rename = "win0Fit8")]
    pub win_0_fit_8: f32,

    #[serde(rename = "win0Fit9")]
    pub win_0_fit_9: f32,

    #[serde(rename = "win1Fit0")]
    pub win_1_fit_0: f32,

    #[serde(rename = "win1Fit1")]
    pub win_1_fit_1: f32,

    #[serde(rename = "win1Fit2")]
    pub win_1_fit_2: f32,

    #[serde(rename = "win1Fit3")]
    pub win_1_fit_3: f32,

    #[serde(rename = "win1Fit4")]
    pub win_1_fit_4: f32,

    #[serde(rename = "win1Fit5")]
    pub win_1_fit_5: f32,

    #[serde(rename = "win1Fit6")]
    pub win_1_fit_6: f32,

    #[serde(rename = "win1Fit7")]
    pub win_1_fit_7: f32,

    #[serde(rename = "win1Fit8")]
    pub win_1_fit_8: f32,

    #[serde(rename = "win1Fit9")]
    pub win_1_fit_9: f32,

    #[serde(rename = "Det Bkgd")]
    pub det_background: f32,

    #[serde(rename = "Ramp Ampl")]
    pub ramp_amplitude: f32,

    #[serde(rename = "CO2 (ppm)-Wet")]
    pub co2_wet: f32,

    #[serde(rename = "CO2 (ppm)")]
    pub co2: f32,

    #[serde(rename = "H2O (ppm)")]
    pub h2o: f32,

    #[serde(rename = "N2O (ppm)-Wet")]
    pub n2o_wet: f32,

    #[serde(rename = "N2O (ppm)")]
    pub n2o: f32,

    #[serde(rename = "Battery Charge (V)")]
    pub battery_charge: f32,

    #[serde(rename = "Power Input (mV)")]
    pub power_input: f32,

    #[serde(rename = "Current (mA)")]
    pub current: f32,

    #[serde(rename = "SOC (%)")]
    pub soc: f32,

    #[serde(rename = "Battery T (degC)")]
    pub battery_temp: f32,

    #[serde(rename = "FET T (degC)")]
    pub fet_temp: f32,

    #[serde(rename = "GPS Time")]
    pub gps_time: f64,

    #[serde(rename = "Latitude")]
    pub latitude: f64,

    #[serde(rename = "Longitude")]
    pub longitude: f64,

    #[serde(rename = "Alt. (m)")]
    pub altitude: f64,

    #[serde(rename = "mpfit (ms)")]
    pub mpfit: u32,

    #[serde(rename = "buffer size")]
    pub buffer_size: u32,

    pub drone_latitude: Option<f64>,

    pub drone_longitude: Option<f64>,

    pub drone_altitude: Option<f64>,
}

#[derive(Debug, Clone, Copy)]
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
    let stream = FramedRead::new(reader, LinesCodec::default()).then(|data| async {
        let data = data?;

        let mut reader = csv_async::AsyncReaderBuilder::new()
            .has_headers(false)
            .create_deserializer(data.as_bytes());

        let mut dsr = reader.deserialize::<SensorData>();

        if let Some(Ok(result)) = dsr.next().await {
            let record: SensorData = result;
            Ok(Some(record))
        } else {
            Ok(None)
        }
    });


    Box::pin(stream)
}

pub async fn handle_sensor_data(
    mavlink: &mut Telem,
    csv: &mut AsyncSerializer<File>,
    current_position: &Option<DroneLocation>,
    sensor_result: Result<Option<SensorData>>,
) -> Result<()> {
    let mut sensor: SensorData = if let Ok(sensor_option) = sensor_result {
        if let Some(sensor) = sensor_option {
            sensor
        } else {
            return Ok(());
        }
    } else {
        return Ok(());
    };

    if let Some(location) = current_position {
        sensor.drone_latitude = Some(location.latitude);
        sensor.drone_longitude = Some(location.longitude);
        sensor.drone_altitude = Some(location.altitude);
    };

    csv.serialize(sensor).await?;

    Ok(())
}
