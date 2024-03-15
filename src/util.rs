pub fn from_string_to_u8<const N: usize>(src: &str) -> [u8; N] {
    let bytes = src.as_bytes();
    let mut dst = [0u8; N];
    let len = std::cmp::min(bytes.len(), N);
    dst[..len].copy_from_slice(&bytes[..len]);
    dst
}
