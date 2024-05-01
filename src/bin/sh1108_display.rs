//! Display a square on a SH1108 OLED display.

#![no_std]
#![no_main]

use defmt_rtt as _;
use display_interface_spi::SPIInterface;
use embassy_executor::Spawner;
use embassy_rp::{
    gpio::{Level, Output},
    spi::{self, Spi},
};
use embassy_time::Delay;
use embedded_hal_bus::spi::ExclusiveDevice;
use panic_probe as _;
use sh1108::{mode::GraphicsMode, Builder};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let sck = p.PIN_10;
    let mosi = p.PIN_11;
    let mut res = Output::new(p.PIN_12, Level::High);
    let dc = Output::new(p.PIN_13, Level::High);
    let cs = Output::new(p.PIN_14, Level::High);
    let mut config = spi::Config::default();
    config.frequency = 400_000;
    let spi = Spi::new_blocking_txonly(p.SPI1, sck, mosi, config);
    let spi_device = ExclusiveDevice::new(spi, cs, Delay);
    let display_interface = SPIInterface::new(spi_device, dc);

    let mut display: GraphicsMode<_> = Builder::new().connect(display_interface).into();

    display.reset(&mut res, &mut Delay).unwrap();
    display.init().unwrap();
    display.flush().unwrap();

    let mut x = 0;
    let mut y = 0;
    loop {
        display.clear();
        display.set_pixel(x, y, 1);
        display.set_pixel(x + 1, y, 1);
        display.set_pixel(x, y + 1, 1);
        display.set_pixel(x + 1, y + 1, 1);
        x += 2;
        if x == 128 {
            x = 0;
            y += 2;
        }
        if y == 64 {
            break;
        }
        display.flush().unwrap();
    }
}
