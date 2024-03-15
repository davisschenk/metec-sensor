use std::sync::atomic::AtomicU8;
use std::time::Duration;

use crate::error::*;

use futures::sink::SinkExt;
use futures::stream::{Stream, StreamExt};
use mavlink::common as MavCommon;
use mavlink::common::MavMessage;
use mavlink::MavHeader;

use tokio::time::Instant;
use tokio_serial::SerialPortBuilderExt;

use tokio_stream::wrappers::IntervalStream;
use tokio_util::codec::Framed;

use super::MavMessageCodec;

pub struct Telem {
    system_id: u8,
    component_id: u8,
    mavlink: Framed<tokio_serial::SerialStream, MavMessageCodec<MavMessage>>,
    sequence: AtomicU8,
}

impl Telem {
    pub fn try_new(
        serial_port: &str,
        baud_rate: u32,
        system_id: u8,
        component_id: u8,
    ) -> Result<Self> {
        let port = tokio_serial::new(serial_port, baud_rate).open_native_async()?;
        let mavlink = Framed::new(port, MavMessageCodec::<MavMessage>::new());
        let sequence = AtomicU8::new(0);

        Ok(Self {
            system_id,
            component_id,
            mavlink,
            sequence,
        })
    }

    pub async fn send(&mut self, message: MavMessage) -> Result<()> {
        let sequence = self
            .sequence
            .fetch_add(1, std::sync::atomic::Ordering::Relaxed);

        let header = MavHeader {
            system_id: self.system_id,
            component_id: self.component_id,
            sequence,
        };

        self.mavlink.send((header, message)).await
    }

    pub async fn recv(&mut self) -> Option<Result<(MavHeader, MavMessage)>> {
        self.mavlink.next().await
    }

    pub fn heartbeat_message(&self) -> MavMessage {
        MavMessage::HEARTBEAT(MavCommon::HEARTBEAT_DATA {
            custom_mode: 0,
            mavtype: MavCommon::MavType::MAV_TYPE_ONBOARD_CONTROLLER,
            autopilot: MavCommon::MavAutopilot::MAV_AUTOPILOT_INVALID,
            base_mode: MavCommon::MavModeFlag::empty(),
            system_status: MavCommon::MavState::MAV_STATE_STANDBY,
            mavlink_version: 0x3,
        })
    }

    pub async fn send_float(&mut self, name: &str, value: f32, boot_time: Instant) -> Result<()> {
        self.send(MavMessage::NAMED_VALUE_FLOAT(
            MavCommon::NAMED_VALUE_FLOAT_DATA {
                time_boot_ms: boot_time.elapsed().as_millis() as u32,
                value,
                name: crate::util::from_string_to_u8(name),
            },
        ))
        .await
    }
}

pub fn heartbeat_stream(telem: &Telem, interval: Duration) -> impl Stream<Item = MavMessage> {
    let heartbeat_message = telem.heartbeat_message();

    IntervalStream::new(tokio::time::interval(interval)).map(move |_| heartbeat_message.clone())
}
