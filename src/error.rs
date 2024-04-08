use std::string::FromUtf8Error;

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

    #[error("failed to join tasks")]
    JoinError(#[from] tokio::task::JoinError),

    #[error("tokio channel")]
    TokioChannelError,

    #[error("from utf8 erro")]
    Utf8Error(#[from] FromUtf8Error),

    #[error(transparent)]
    Other(#[from] anyhow::Error),
}

impl<T> From<tokio::sync::watch::error::SendError<T>> for SensorError {
    fn from(value: tokio::sync::watch::error::SendError<T>) -> Self {
        Self::TokioChannelError
    }
}

impl<T> From<tokio::sync::mpsc::error::SendError<T>> for SensorError {
    fn from(value: tokio::sync::mpsc::error::SendError<T>) -> Self {
        Self::TokioChannelError
    }
}
