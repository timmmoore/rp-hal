// types of device failures
#[allow(dead_code)]
#[derive(Copy, Clone)]
pub enum DeviceError {
    Bmp280Unidentified = 1,
    Bmp280Error = 2,
    ShtcxIdCrc = 3,
    ShtcxIdI2c = 4,
    Bmp280MeasureError = 5,
    ShtcxWakeupError = 6,
    ShtcxStartMeasurementError = 7,
    ShtcxGetMeasurementCrcError = 8,
    ShtcxGetMeasurementI2cError = 9,
    Pms7003SendFailed = 10,
    Pms7003ReadFailed = 11,
    Pms7003ChecksumError = 12,
    Pms7003IncorrectResponse = 13,
    Pms7003NoResponse = 14,               
    Sgp40MeasureVocIndexError = 15,
    WifiConnectError = 16,
    WifiSendError = 17,
    WifiDisconnectError = 18,
    WifiATError = 19,
    WifiStationError = 20,
    WifiSSIDError = 21,
    WifiIPAddressError = 22,
}

pub fn error_code(code: DeviceError) -> u32 {
    1 << (code as u8)
}
