use mavlink::error::ParserError;
use thiserror::Error;
use tokio_util::codec::LinesCodecError;

pub type Result<T> = std::result::Result<T, SensorError>;

#[derive(Error, Debug)]
pub enum SensorError {
    #[error("mavlink connection failed")]
    MavlinkConnectionError(#[from] std::io::Error),

    #[error(transparent)]
    SerialError(#[from] tokio_serial::Error),

    #[error("mavlink send error")]
    MavlinkSendError,

    #[error("mavlink parsing error")]
    MavlinkParsingError(#[from] ParserError),

    #[error("linecodec error")]
    LineCodecError(#[from] LinesCodecError),

    #[error("csv error")]
    CsvError(#[from] csv_async::Error),

    #[error("mavlink recv error")]
    MavlinkRecvError,

    #[error(transparent)]
    Other(#[from] anyhow::Error),
}
