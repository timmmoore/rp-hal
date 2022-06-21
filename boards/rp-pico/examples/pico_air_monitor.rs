//! For https://www.tindie.com/products/sbc/esp8266-air-wifi-monitoring-for-raspberry-pi-pico/
//! with  Raspberry Pi Pico
//!
//! See the `Cargo.toml` file for Copyright and license details.

#![no_std]
#![no_main]

// For string formatting.
use core::fmt::Write;

// The macro for our start-up function
use cortex_m_rt::entry;

// Time handling traits:
use embedded_time::duration::*;
use embedded_time::rate::Extensions;

// CountDown timer for the counter on the display:
use embedded_hal::timer::CountDown;
use rp2040_hal::Timer;

use embedded_hal::prelude::_embedded_hal_serial_Read;

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

// Pull in any important traits
use rp_pico::hal::prelude::*;

// GPIO traits
use embedded_hal::digital::v2::OutputPin;

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use rp_pico::hal::pac;

// A shorter alias for the Hardware Abstraction Layer, which provides
// higher-level drivers.
use rp_pico::hal;

// For in the graphics drawing utilities like the font
// and the drawing routines:
use embedded_graphics::{
    mono_font::{ascii::FONT_6X9, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};

// shared i2c
use shared_bus;
// The display driver:
use ssd1306::{prelude::*, Ssd1306};
// pressure, temp, humity
use bme280::BME280;
// air monitor
use pms_7003::*;
// voc
use sgp40::*;
// temp, humidity
use shtcx::*;

/// Import the GPIO pins we use
use hal::gpio::pin::bank0::{Gpio4, Gpio5};

mod air_config;

type Wifiuartpins = (
    hal::gpio::Pin<Gpio4, hal::gpio::Function<hal::gpio::Uart>>,
    hal::gpio::Pin<Gpio5, hal::gpio::Function<hal::gpio::Uart>>,
);/// Alias the type for our wifi UART to make things clearer.
type Wifiuart = hal::uart::UartPeripheral<hal::uart::Enabled, hal::pac::UART1, Wifiuartpins>;

// types of device failures
#[derive(Copy, Clone)]
enum DeviceError {
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

fn error_code(code: DeviceError) -> u32 {
    1 << (code as u8)
}
/// Entry point to our bare-metal application.
///
/// The `#[entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables are initialised.
///
/// The function configures the RP2040 peripherals,
/// gets a handle on the I2C peripheral,
/// initializes the SSD1306 driver, initializes the text builder
#[entry]
fn main() -> ! {
    let mainloopdelay = 20000;

    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    //
    // The default is to generate a 125 MHz system clock
    let clocks = hal::clocks::init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let freq = clocks.peripheral_clock.freq();

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Set the pins up according to their function on this particular board
    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
    // Configure two pins as being I²C, not GPIO
    let sda_pin = pins.gpio20.into_mode::<hal::gpio::FunctionI2C>();
    let scl_pin = pins.gpio21.into_mode::<hal::gpio::FunctionI2C>();

    // Create the I²C driver, using the two pre-configured pins. This will fail
    // at compile time if the pins are in the wrong mode, or if this I²C
    // peripheral isn't available on these pins!
    let i2c = hal::I2C::i2c0(
        pac.I2C0,
        sda_pin,
        scl_pin,
        400.kHz(),
        &mut pac.RESETS,
        clocks.peripheral_clock,
    );
    // create shared bus for I2C
    let bus = shared_bus::BusManagerSimple::new(i2c);

    // Create the I²C display interface:
    let interface = ssd1306::I2CDisplayInterface::new(bus.acquire_i2c());
    // Create a driver instance and initialize:
    let mut display = Ssd1306::new(interface, DisplaySize128x32, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    display.init().unwrap();
    // Empty the display:
    display.clear();
    display.flush().unwrap();

    // Create a text style for drawing the font:
    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_6X9)
        .text_color(BinaryColor::On)
        .build();

    let mut fail: u32 = 0;

    // Create I2C BME280 - temperature, pressure and humidity sensor
    let delayp = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());
    let mut bme280 = BME280::new_primary(bus.acquire_i2c(), delayp);
    match bme280.init() {
        Ok(()) => {}
        Err(bme280::Error::UnsupportedChip) => {
            fail += error_code(DeviceError::Bmp280Unidentified);
        }
        Err(_) => {
            fail += error_code(DeviceError::Bmp280Error);
        }
    }

    // sgp40
    let timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS);
    let mut delay = timer.count_down();
    let mut sensor = Sgp40::new(bus.acquire_i2c(), 0x59, &mut delay);
    
    // shtc3
    let mut sht = shtcx::shtc3(bus.acquire_i2c());
    match sht.device_identifier() {
        Ok(_) => {}
        Err(shtcx::Error::Crc) => {
            fail += error_code(DeviceError::ShtcxIdCrc);
        }
        Err(shtcx::Error::I2c(_)) => {
            fail += error_code(DeviceError::ShtcxIdI2c);
        }
    }
    
    // particle monitor
    // init serial and device
    let air_uart_pins = (
        // UART TX (characters sent from RP2040) on pin 1 (GPIO0)
        pins.gpio0.into_mode::<hal::gpio::FunctionUart>(),
        // UART RX (characters received by RP2040) on pin 2 (GPIO1)
        pins.gpio1.into_mode::<hal::gpio::FunctionUart>(),
    );
    let air_uart = hal::uart::UartPeripheral::new(pac.UART0, air_uart_pins, &mut pac.RESETS)
        .enable(
            hal::uart::common_configs::_9600_8_N_1,
            freq,
        )
        .unwrap();

    let mut airsensor = Pms7003Sensor::new(air_uart);

    // ESP8266 wifi
    // init serial and device
    let wifi_uart_pins = (
        // UART TX (characters sent from RP2040) on pin 6 (GPIO4)
        pins.gpio4.into_mode::<hal::gpio::FunctionUart>(),
        // UART RX (characters received by RP2040) on pin 7 (GPIO5)
        pins.gpio5.into_mode::<hal::gpio::FunctionUart>(),
    );

    let mut wifi_uart = hal::uart::UartPeripheral::new(pac.UART1, wifi_uart_pins, &mut pac.RESETS)
        .enable(
            hal::uart::common_configs::_115200_8_N_1,
            freq,
        )
        .unwrap();

    // initialize the ESP8266 and connect to Thingspeak
    fail += init_wifi(&mut wifi_uart, &timer);

    // Set the LED to be an output for errors
    let mut led_pin = pins.led.into_push_pull_output();

    let mut delay = timer.count_down();

    // buffer for text for lcd
    let mut buf = FmtBuf::new();

    let mut pm1 = 0;
    let mut pm2_5 = 0;
    let mut pm10 = 0;
    let mut temp = 0.0;
    let mut pres = 0.0;
    let mut hum = 0.0;
    let mut voc: u16 = 0;
    let mut vocinit = false;
    loop {
        // update led with status of the devices
        (if fail != 0 { led_pin.set_high() } else { led_pin.set_low() })
            .unwrap();

        // update measurements from devices
        match bme280.measure() {
            Ok(measurements) => {
                pres = measurements.pressure;
                temp = measurements.temperature;
                hum = measurements.humidity;
            },
            Err(_) => {
                fail += error_code(DeviceError::Bmp280MeasureError);
            },
        }
        // overides bmp280 for temp and humidity if available
        match sht.start_wakeup() {
            Ok(()) => {}
            Err(_) => {
                fail += error_code(DeviceError::ShtcxWakeupError);
            }
        }
        // 3 milliseconds to wakeup
        pause(&mut delay, 3);
        match sht.start_measurement(PowerMode::NormalMode) {
            Ok(()) => {}
            Err(_) => {
                fail += error_code(DeviceError::ShtcxStartMeasurementError);
            }
        }
        // 14.5 mullisecond to respond
        pause(&mut delay, 20);
        // if error wait 1 millsecond and try again
        for _ in 1..20 {
            match sht.get_measurement_result() {
                Ok(measurement) => {
                    temp = measurement.temperature.as_degrees_celsius();
                    hum = measurement.humidity.as_percent();
                    break;
                }
                Err(shtcx::Error::Crc) => {
                    fail += error_code(DeviceError::ShtcxGetMeasurementCrcError);
                    pause(&mut delay, 1);
                }
                Err(shtcx::Error::I2c(_)) => {
                    fail += error_code(DeviceError::ShtcxGetMeasurementI2cError);
                    pause(&mut delay, 1);
                }
            }
        }
        // Get info from air particle sensor
        match airsensor.read(&timer) {
            Ok(frame) => {
                // update if available
                pm1 = frame.pm1_0;
                pm2_5 = frame.pm2_5;
                pm10 = frame.pm10;
            },
            Err(pms_7003::Error::SendFailed) => {
                fail += error_code(DeviceError::Pms7003SendFailed);
            },
            Err(pms_7003::Error::ReadFailed) => {
                fail += error_code(DeviceError::Pms7003ReadFailed);
            },
            Err(pms_7003::Error::ChecksumError) => {
                fail += error_code(DeviceError::Pms7003ChecksumError);
            },
            Err(pms_7003::Error::IncorrectResponse) => {
                fail += error_code(DeviceError::Pms7003IncorrectResponse);
            },
            Err(pms_7003::Error::NoResponse) => {
                fail += error_code(DeviceError::Pms7003NoResponse);
            },
        }
        // get voc index using current temperature and humidity
        match sensor.measure_voc_index_with_rht((hum*1000.0) as u16, (temp*1000.0) as i16) {
            Ok(result) => {
                voc = result;
            }
            Err(_) => {
                fail += error_code(DeviceError::Sgp40MeasureVocIndexError);
            }
        }

        // Update the local display
        display.clear();
        // Format info into buffer and draw on LCD
        buf.reset();
        write!(&mut buf, "Pres:{:.1} {:01x}", pres, fail).unwrap();
        Text::with_baseline(buf.as_str(), Point::new(0, 0), text_style, Baseline::Top)
            .draw(&mut display)
            .unwrap();
        // Format info into buffer and draw on LCD
        buf.reset();
        write!(&mut buf, "Temp:{:.2}, Hum:{:.2}", temp, hum).unwrap();
        Text::with_baseline(buf.as_str(), Point::new(0, 10), text_style, Baseline::Top)
            .draw(&mut display)
            .unwrap();
        // Format info into buffer and draw on LCD
        buf.reset();
        write!(&mut buf, "Air: {} {} {} {}", pm1, pm2_5, pm10, voc).unwrap();
        Text::with_baseline(buf.as_str(), Point::new(0, 20), text_style, Baseline::Top)
            .draw(&mut display)
            .unwrap();
        display.flush().unwrap();

        // send data to thingspeak.com
        fail = send_remote(&mut wifi_uart, &timer, pm1, pm2_5, pm10, temp, pres, hum, voc, fail);

        // delay before the next measurement
        // while voc is starting up (first 45 redings at 1sec intervals) use this time to continue its startup
        // note first loop delay will be 45sec rather than normal delay (20sec)
        if vocinit == false {
            for _ in 1..45 {
                match sensor.measure_voc_index() { _ => () }
                pause(&mut delay, 1000);
            }
            vocinit = true;
        }
        else {
            // Wait a bit:
            pause(&mut delay, mainloopdelay);
        }
    }
}

/// This is a very simple buffer to pre format a short line of text
/// limited arbitrarily to 256 bytes - more than enough for the send to thingspeak.
struct FmtBuf {
    buf: [u8; 256],
    ptr: usize,
}

impl FmtBuf {
    fn new() -> Self {
        Self {
            buf: [0; 256],
            ptr: 0,
        }
    }

    fn reset(&mut self) {
        self.ptr = 0;
    }

    fn as_str(&self) -> &str {
        core::str::from_utf8(&self.buf[0..self.ptr]).unwrap()
    }
}

impl core::fmt::Write for FmtBuf {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let rest_len = self.buf.len() - self.ptr;
        let len = if rest_len < s.len() {
            rest_len
        } else {
            s.len()
        };
        self.buf[self.ptr..(self.ptr + len)].copy_from_slice(&s.as_bytes()[0..len]);
        self.ptr += len;
        Ok(())
    }
}

fn pause<D>(delay: &mut D, pause: u32)
  where
    D: CountDown,
    D::Time: From<Milliseconds>
{
    delay.start(pause.milliseconds());
    nb::block!(delay.wait()).unwrap();
}
/*
use rp2040_hal::I2C;
use hal::gpio::pin::bank0::{Gpio20, Gpio21};
type SdaPint = hal::gpio::Pin<Gpio20, hal::gpio::Function<hal::gpio::I2C>>;
type SclPint = hal::gpio::Pin<Gpio21, hal::gpio::Function<hal::gpio::I2C>>;

fn setup_i2c(pins: rp_pico::Pins, pac: pac::Peripherals, clocks: rp2040_hal::clocks::ClocksManager) -> &'static shared_bus::BusManagerSimple<I2C<<pac::Peripherals>::I2C0, SdaPint, SclPint>> {
    // Configure two pins as being I²C, not GPIO
    let sda_pin = pins.gpio20.into_mode::<hal::gpio::FunctionI2C>();
    let scl_pin = pins.gpio21.into_mode::<hal::gpio::FunctionI2C>();

    // Create the I²C driver, using the two pre-configured pins. This will fail
    // at compile time if the pins are in the wrong mode, or if this I²C
    // peripheral isn't available on these pins!
    let i2c = hal::I2C::i2c0(
        pac.I2C0,
        sda_pin,
        scl_pin,
        400.kHz(),  
        &mut pac.RESETS,
        clocks.peripheral_clock,
    );
    // create shared bus for I2C
    shared_bus::BusManagerSimple::new(i2c)
}
*/
// 
fn init_wifi(wifi: &mut Wifiuart, timer: &Timer) -> u32 {
    // connect ESP8266 to wifi
    //let cmd1 = "AT+CWJAP=\"DUMMYSSID\",\"DUMMYPASSWORD\"";
    let mut buf = FmtBuf::new();
    let mut fail = 0;

    if send_cmd(wifi, "AT", "OK", &timer, 2000) == false {
        fail += error_code(DeviceError::WifiATError);
    }
    // switch to station mode
    if send_cmd(wifi, "AT+CWMODE=1","OK", &timer, 2000) == false {
        fail += error_code(DeviceError::WifiStationError);
    }
    // connect to wifi SSID
    buf.reset();
    write!(&mut buf, "AT+CWJAP=\"{}\",\"{}\"",
        air_config::get_config(air_config::Config::SSID),
        air_config::get_config(air_config::Config::PASSWORD)).unwrap();
    if send_cmd(wifi, buf.as_str(), "OK", &timer, 20000) == false {
        fail += error_code(DeviceError::WifiSSIDError);
    }
    // get ip address
    if send_cmd(wifi, "AT+CIFSR", "OK", &timer, 2000) == false {
        fail += error_code(DeviceError::WifiIPAddressError);
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

fn send_remote(wifi: &mut Wifiuart, timer: &Timer, pm1: u16, pm2_5: u16, pm10: u16, temp: f32, pres: f32, hum: f32, voc: u16, gfail: u32) -> u32 {
    //let api_key = "DUMMYAPIKEY";    //write your API Key
    let api_key = air_config::get_config(air_config::Config::APIKEY);    //write your API Key
    let cmd2 = "AT+CIPSTART=\"TCP\",\"184.106.153.149\",80";
    let cmd3 = "AT+CIPCLOSE";
    let mut buf = FmtBuf::new();
    let mut cmdbuf = FmtBuf::new();
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
        fail += error_code(DeviceError::WifiConnectError);
    }
    else {
        // Send the data length
        cmdbuf.reset();
        write!(&mut cmdbuf, "AT+CIPSEND={}\r\n", buf.ptr).unwrap();
        if send_cmd(wifi, cmdbuf.as_str(), "OK", &timer, 2000) == false {
            fail += error_code(DeviceError::WifiSendError);
            // close the tcp connection
            if send_cmd(wifi, cmd3, "OK", &timer, 5000) == false {
                fail += error_code(DeviceError::WifiDisconnectError);
            }
        }
        else {
            // send the data
            wifi.write_full_blocking(&buf.buf[0..buf.ptr]);
        }
    }
    fail
}
// End of file
