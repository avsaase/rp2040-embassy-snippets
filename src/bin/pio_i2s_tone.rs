//! Play a pure sine wave tone via I2S using the RP2040's PIO.
//! Code adapted from
//! https://github.com/bschwind/super-sanic/blob/395a64b2522431e518b3ec3caa6365fc47898246/src/main.rs

#![no_std]
#![no_main]

use core::f32::consts::TAU;

use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::{
    bind_interrupts,
    peripherals::PIO0,
    pio::{Config, FifoJoin, InterruptHandler, Pio, ShiftConfig, ShiftDirection},
    Peripheral,
};
use fixed::traits::ToFixed;
use panic_probe as _;

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
});

const SAMPLE_RATE: u32 = 8_000;
const BIT_DEPTH: u32 = 32;
const CHANNELS: u32 = 2;
const MIN_FREQUENCY: u32 = 200;
const BUFFER_SIZE: u32 = SAMPLE_RATE / MIN_FREQUENCY * CHANNELS;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    let mut pio = Pio::new(p.PIO0, Irqs);

    #[rustfmt::skip]
    let pio_program = pio_proc::pio_asm!(
        ".side_set 2",
        "    set x, 30          side 0b01", // side 0bWB - W = Word Clock, B = Bit Clock
        "left_data:",
        "    out pins, 1        side 0b00",
        "    jmp x-- left_data  side 0b01",
        "    out pins 1         side 0b10",
        "    set x, 30          side 0b11",
        "right_data:",
        "    out pins 1         side 0b10",
        "    jmp x-- right_data side 0b11",
        "    out pins 1         side 0b00",
    );

    let data_pin = p.PIN_6;
    let bit_clock_pin = p.PIN_7;
    let left_right_clock_pin = p.PIN_8;

    let data_pin = pio.common.make_pio_pin(data_pin);
    let bit_clock_pin = pio.common.make_pio_pin(bit_clock_pin);
    let left_right_clock_pin = pio.common.make_pio_pin(left_right_clock_pin);

    let cfg = {
        let mut cfg = Config::default();
        cfg.use_program(
            &pio.common.load_program(&pio_program.program),
            &[&bit_clock_pin, &left_right_clock_pin],
        );
        cfg.set_out_pins(&[&data_pin]);
        let clock_frequency = SAMPLE_RATE * BIT_DEPTH * CHANNELS;
        cfg.clock_divider = (125_000_000. / clock_frequency as f64 / 2.).to_fixed();
        cfg.shift_out = ShiftConfig {
            threshold: 32,
            direction: ShiftDirection::Left,
            auto_fill: true,
        };
        cfg.fifo_join = FifoJoin::TxOnly;
        cfg
    };
    pio.sm0.set_config(&cfg);
    pio.sm0.set_enable(true);
    pio.sm0.set_pin_dirs(
        embassy_rp::pio::Direction::Out,
        &[&data_pin, &left_right_clock_pin, &bit_clock_pin],
    );

    let mut dma_buffer = [0u32; BUFFER_SIZE as usize];

    let mut dma_ref = p.DMA_CH0.into_ref();
    let tx = pio.sm0.tx();

    let samples = generate_samples(&mut dma_buffer, 1000, -6.0);

    loop {
        tx.dma_push(dma_ref.reborrow(), samples).await;
    }
}

/// Generates one period of audio samples of a sine wave of the given frequency.
///
/// Returns a slice containing the samples.
pub fn generate_samples(buf: &mut [u32], frequency: u32, volume_db: f32) -> &[u32] {
    let mut current = 0.0;
    let delta = frequency as f32 * TAU / SAMPLE_RATE as f32;
    let samples_per_period = SAMPLE_RATE / frequency;
    let amplitude = libm::powf(10.0, volume_db / 20.).min(1.0) * i32::MAX as f32;

    for frame in buf
        .chunks_mut(CHANNELS as usize)
        .take(samples_per_period as usize)
    {
        let val = (libm::sinf(current) * amplitude) as i32 as u32;
        frame[0] = val;
        frame[1] = val;
        current += delta;
    }
    &buf[0..((samples_per_period * CHANNELS) as usize)]
}
