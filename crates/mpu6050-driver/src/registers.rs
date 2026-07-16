pub(crate) const SMPLRT_DIV: u8 = 0x19;
pub(crate) const CONFIG: u8 = 0x1A;
pub(crate) const GYRO_CONFIG: u8 = 0x1B;
pub(crate) const ACCEL_CONFIG: u8 = 0x1C;
pub(crate) const FIFO_EN: u8 = 0x23;
pub(crate) const INT_ENABLE: u8 = 0x38;
pub(crate) const INT_STATUS: u8 = 0x3A;
pub(crate) const ACCEL_XOUT_H: u8 = 0x3B;
pub(crate) const USER_CTRL: u8 = 0x6A;
pub(crate) const PWR_MGMT_1: u8 = 0x6B;
pub(crate) const FIFO_COUNTH: u8 = 0x72;
pub(crate) const FIFO_R_W: u8 = 0x74;
pub(crate) const WHO_AM_I: u8 = 0x75;

pub(crate) const ACCEL_RANGE_MASK: u8 = 0x18;
pub(crate) const GYRO_RANGE_MASK: u8 = 0x18;
pub(crate) const SELF_TEST_MASK: u8 = 0xE0;
pub(crate) const DLPF_CFG_MASK: u8 = 0x07;
pub(crate) const USER_CTRL_FIFO_EN: u8 = 1 << 6;
pub(crate) const USER_CTRL_FIFO_RESET: u8 = 1 << 2;
pub(crate) const INT_ENABLE_DATA_RDY: u8 = 1 << 0;
pub(crate) const INT_ENABLE_FIFO_OFLOW: u8 = 1 << 4;
pub(crate) const INT_STATUS_DATA_RDY: u8 = 1 << 0;
pub(crate) const INT_STATUS_FIFO_OFLOW: u8 = 1 << 4;

// FIFO_EN, Register 35: XG/YG/ZG bits[6:4] plus ACCEL bit[3].
// Temperature FIFO bit[7] is intentionally omitted so each FIFO motion frame is
// 6 axes * 2 bytes = 12 bytes.
pub(crate) const FIFO_SOURCES_ACCEL_XYZ_GYRO_XYZ: u8 = (1 << 6) | (1 << 5) | (1 << 4) | (1 << 3);
