use crate::error::SensorError;
use bytes::{Buf, BufMut};
use crc_any::CRCu16;
use mavlink::{MavHeader, MavlinkVersion};
use std::marker::PhantomData;
use tokio_util::codec::{Decoder, Encoder};

pub struct MavMessageCodec<M> {
    _phantom: PhantomData<M>,
}

impl<M: mavlink::Message> Default for MavMessageCodec<M> {
    fn default() -> Self {
        Self::new()
    }
}

impl<M: mavlink::Message> MavMessageCodec<M> {
    pub fn new() -> MavMessageCodec<M> {
        MavMessageCodec {
            _phantom: PhantomData {},
        }
    }
}

impl<M: mavlink::Message> Encoder<(mavlink::MavHeader, M)> for MavMessageCodec<M> {
    type Error = SensorError;

    fn encode(
        &mut self,
        message: (mavlink::MavHeader, M),
        dst: &mut bytes::BytesMut,
    ) -> Result<(), Self::Error> {
        let (header, data) = message;
        let mut writer = dst.writer();

        mavlink::write_v2_msg(&mut writer, header, &data)
            .map_err(|_| SensorError::MavlinkSendError)?;

        Ok(())
    }
}

impl<M: mavlink::Message> Decoder for MavMessageCodec<M> {
    type Item = (mavlink::MavHeader, M);
    type Error = SensorError;

    fn decode(
        &mut self,
        src: &mut bytes::BytesMut,
    ) -> std::result::Result<Option<Self::Item>, Self::Error> {
        match src.iter().position(|&byte| byte == mavlink::MAV_STX_V2) {
            Some(index) => {
                src.advance(index);
                let payload_len = match src.get(1) {
                    Some(&len) => len as usize,
                    None => return Ok(None),
                };
                let has_signature = match src.get(2) {
                    Some(flags) => flags & 0x01 == 0x01, // MAVLINK_IFLAG_SIGNED
                    None => return Ok(None),
                };
                let mut message_len = 12 + payload_len;
                if has_signature {
                    message_len += 13;
                };
                if src.remaining() >= message_len {
                    /* skip over STX */
                    src.advance(1);
                    let payload_len = src.get_u8() as usize;
                    let incompat_flags = src.get_u8();
                    let compat_flags = src.get_u8();
                    let seq = src.get_u8();
                    let sysid = src.get_u8();
                    let compid = src.get_u8();
                    let mut msgid_buf = [0; 4];
                    msgid_buf[0] = src.get_u8();
                    msgid_buf[1] = src.get_u8();
                    msgid_buf[2] = src.get_u8();

                    let header_buf = &[
                        payload_len as u8,
                        incompat_flags,
                        compat_flags,
                        seq,
                        sysid,
                        compid,
                        msgid_buf[0],
                        msgid_buf[1],
                        msgid_buf[2],
                    ];
                    let msgid: u32 = u32::from_le_bytes(msgid_buf);
                    let payload = src.split_to(payload_len);
                    let crc = src.get_u16_le();
                    if has_signature {
                        src.advance(13);
                    }
                    let mut crc_calc = CRCu16::crc16mcrf4cc();
                    crc_calc.digest(&header_buf[..]);
                    crc_calc.digest(&payload[..]);
                    let extra_crc = M::extra_crc(msgid);

                    crc_calc.digest(&[extra_crc]);
                    let recvd_crc = crc_calc.get_crc();
                    if recvd_crc == crc {
                        /* hack: we should have a CRC error here */
                        M::parse(MavlinkVersion::V2, msgid, &payload[..])
                            .map(|msg| {
                                Some((
                                    MavHeader {
                                        sequence: seq,
                                        system_id: sysid,
                                        component_id: compid,
                                    },
                                    msg,
                                ))
                            })
                            .map_err(|err| err.into())
                    } else {
                        /* CRC check failed, skip this message */
                        Ok(None)
                    }
                } else {
                    Ok(None)
                }
            }
            None => Ok(None),
        }
    }
}
