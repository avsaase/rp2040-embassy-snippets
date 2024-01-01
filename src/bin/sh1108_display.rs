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
    let spi = Spi::new_txonly(p.SPI1, sck, mosi, p.DMA_CH2, config);

    let display_interface = SPIInterface::new(spi, dc, cs);

    let mut display: GraphicsMode<_> = Builder::new().connect(display_interface).into();

    display.reset(&mut res, &mut Delay).unwrap();
    display.init().unwrap();
    display.flush().unwrap();

    // Top side
    display.set_pixel(0, 0, 1);
    display.set_pixel(1, 0, 1);
    display.set_pixel(2, 0, 1);
    display.set_pixel(3, 0, 1);

    // Right side
    display.set_pixel(3, 0, 1);
    display.set_pixel(3, 1, 1);
    display.set_pixel(3, 2, 1);
    display.set_pixel(3, 3, 1);

    // Bottom side
    display.set_pixel(0, 3, 1);
    display.set_pixel(1, 3, 1);
    display.set_pixel(2, 3, 1);
    display.set_pixel(3, 3, 1);

    // Left side
    display.set_pixel(0, 0, 1);
    display.set_pixel(0, 1, 1);
    display.set_pixel(0, 2, 1);
    display.set_pixel(0, 3, 1);

    display.flush().unwrap();
}
