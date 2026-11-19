//! CRC16-CCITT-FALSE (poly 0x1021, init 0xFFFF) — robot_link_protocol.md 準拠。

pub fn crc16_ccitt(data: &[u8]) -> u16 {
    let mut crc: u16 = 0xFFFF;
    for &byte in data {
        crc ^= (byte as u16) << 8;
        for _ in 0..8 {
            crc = if crc & 0x8000 != 0 {
                (crc << 1) ^ 0x1021
            } else {
                crc << 1
            };
        }
    }
    crc
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn known_vector() {
        // "123456789" の CRC16-CCITT-FALSE は 0x29B1。
        assert_eq!(crc16_ccitt(b"123456789"), 0x29B1);
    }

    #[test]
    fn empty() {
        assert_eq!(crc16_ccitt(b""), 0xFFFF);
    }
}
