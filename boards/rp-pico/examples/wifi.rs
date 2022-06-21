use embedded_time::duration::Extensions;
use embedded_hal::prelude::_embedded_hal_serial_Read;
use embedded_hal::prelude::_embedded_hal_timer_CountDown;
use rp2040_hal::Timer;
use rp_pico::hal;
/// Import the GPIO pins we use
use hal::gpio::pin::bank0::{Gpio4, Gpio5};
use core::fmt::Write;

#[path = "device/mod.rs"]
mod device;
#[path = "air_config.rs"]
mod air_config;
#[path = "fmtbuf.rs"]
mod fmtbuf;

pub type Wifiuartpins = (
    hal::gpio::Pin<Gpio4, hal::gpio::Function<hal::gpio::Uart>>,
    hal::gpio::Pin<Gpio5, hal::gpio::Function<hal::gpio::Uart>>,
);/// Alias the type for our wifi UART to make things clearer.
pub type Wifiuart = hal::uart::UartPeripheral<hal::uart::Enabled, hal::pac::UART1, Wifiuartpins>;

pub fn init_wifi(wifi: &mut Wifiuart, timer: &Timer) -> u32 {
    // connect ESP8266 to wifi
    let mut buf = fmtbuf::FmtBuf::new();
    let mut fail = 0;

    if send_cmd(wifi, "AT", "OK", &timer, 2000) == false {
        fail += device::error_code(device::DeviceError::WifiATError);
    }
    // switch to station mode
    if send_cmd(wifi, "AT+CWMODE=1","OK", &timer, 2000) == false {
        fail += device::error_code(device::DeviceError::WifiStationError);
    }
    // connect to wifi SSID
    buf.reset();
    write!(&mut buf, "AT+CWJAP=\"{}\",\"{}\"",
        air_config::get_config(air_config::Config::SSID),
        air_config::get_config(air_config::Config::PASSWORD)).unwrap();
    if send_cmd(wifi, buf.as_str(), "OK", &timer, 20000) == false {
        fail += device::error_code(device::DeviceError::WifiSSIDError);
    }
    // get ip address
    if send_cmd(wifi, "AT+CIFSR", "OK", &timer, 2000) == false {
        fail += device::error_code(device::DeviceError::WifiIPAddressError);
    }
    fail
}

fn send_cmd(wifi: &mut Wifiuart, cmd: &str, ack: &str, timer: &Timer, timeout:u32) -> bool {
    let mut buf: [u8; 100] = [0; 100];
    let mut index: usize = 0;

    wifi.write_full_blocking(cmd.as_bytes());
    wifi.write_full_blocking(b"\r\n");
    // start the timeout
    let mut delaywifi = timer.count_down();
    delaywifi.start(timeout.milliseconds());
    'delayloop: loop {
        match delaywifi.wait() {
            // still waiting for timeout
            Err(_) => {
                // non-blocking read
                match wifi.read() {
                    Ok(byte) => {
                        // end of command
                        if byte == b'\r' {
                            // length incorrect, ignore line
                            if ack.len() != index { index = 0; continue; }
                            let ackbytes = ack.as_bytes();
                            let mut i = 0;
                            for b in ackbytes.iter() {
                                // char not correct
                                if *b != buf[i] {
                                    index = 0;
                                    // break out of for loop
                                    continue 'delayloop;
                                }
                                i += 1;
                            }
                            // all correct
                            if i == index  { return true; }
                            // no match, ignore line
                            index = 0;
                        }
                        //ignore end of line
                        else if byte != b'\n' {
                            buf[index] = byte;
                            index += 1;
                        }
                        // too long a line, ignore and restart line
                        if index == buf.len() {
                            index = 0;
                            continue;
                        }
                    },
                    // ignore error, rely on the timeout
                    _ => {}
                }
            },
            // timeout
            Ok(()) => {
                return false;
            }
        }
    }
}

pub fn send_remote(wifi: &mut Wifiuart, timer: &Timer, pm1: u16, pm2_5: u16, pm10: u16, temp: f32, pres: f32, hum: f32, voc: u16, gfail: u32) -> u32 {
    let api_key = air_config::get_config(air_config::Config::APIKEY);
    let cmd2 = "AT+CIPSTART=\"TCP\",\"184.106.153.149\",80";
    let cmd3 = "AT+CIPCLOSE";
    let mut buf = fmtbuf::FmtBuf::new();
    let mut cmdbuf = fmtbuf::FmtBuf::new();
    let mut fail = 0;

    buf.reset();
    write!(&mut buf, "GET /update?key={}&field1={}&field2={}&field3={}&field4={}&field5={}&field6={}&field7={}&field8={}\r\n\r\n",
        api_key,
        pm1,
        pm2_5,
        pm10,
        temp,
        pres,
        hum,
        voc,
        gfail).unwrap();

    // Connect to thingspeak server
    if send_cmd(wifi, cmd2, "OK", &timer, 10000) == false {
        fail += device::error_code(device::DeviceError::WifiConnectError);
    }
    else {
        // Send the data length
        cmdbuf.reset();
        write!(&mut cmdbuf, "AT+CIPSEND={}\r\n", buf.ptr).unwrap();
        if send_cmd(wifi, cmdbuf.as_str(), "OK", &timer, 2000) == false {
            fail += device::error_code(device::DeviceError::WifiSendError);
            // close the tcp connection
            if send_cmd(wifi, cmd3, "OK", &timer, 5000) == false {
                fail += device::error_code(device::DeviceError::WifiDisconnectError);
            }
        }
        else {
            // send the data
            wifi.write_full_blocking(&buf.buf[0..buf.ptr]);
        }
    }
    fail
}
