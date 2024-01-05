//! Play pure sine wave tones via I2S using the RP2040's PIO.
//!
//! If you're using an Adafruit MAX98357 I2S Class-D Mono Amp, connect it as follows:
//!
//! Amp  -> Pi Pico
//! GND  -> GND
//! Vin  -> 5V or 3.3V (make sure your power supply can provide enough current)
//! LRC  -> GPIO8
//! BCLK -> GPIO7
//! DIN  -> GPIO6

#![no_std]
#![no_main]

use core::f32::consts::TAU;

use defmt::info;
use defmt_rtt as _;
use embassy_executor::{InterruptExecutor, Spawner};
use embassy_rp::{
    bind_interrupts,
    dma::AnyChannel,
    interrupt::{self, InterruptExt, Priority},
    peripherals::{self, PIO0},
    pio::{Config, FifoJoin, InterruptHandler, Pio, ShiftConfig, ShiftDirection},
    Peripheral,
};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::{Instant, Timer};
use fixed::traits::ToFixed;
use futures::future::FutureExt;
use heapless::Vec;
use panic_probe as _;

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
});

static INTERRUPT_EXECUTOR: InterruptExecutor = InterruptExecutor::new();

const SAMPLE_RATE: u32 = 8_000;
const BIT_DEPTH: u32 = 32;
const CHANNELS: u32 = 2;
const MIN_FREQUENCY: u32 = 200;
const BUFFER_SIZE: usize = (SAMPLE_RATE / MIN_FREQUENCY * CHANNELS) as usize;

// Signal used to communicate between main() and the PIO task.
static SIGNAL: Signal<CriticalSectionRawMutex, Vec<u32, 80>> = Signal::new();

// Table of sine values to speed up calculation of the audio samples.
include!(concat!(env!("OUT_DIR"), "/sine_table.rs"));

#[cortex_m_rt::interrupt]
unsafe fn SWI_IRQ_1() {
    INTERRUPT_EXECUTOR.on_interrupt()
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    info!("Started Embassy");

    // Start the interupt executor. This executor runs tasks with higher priority than the normal
    // tasks.
    interrupt::SWI_IRQ_1.set_priority(Priority::P2);
    let interrupt_spawner = INTERRUPT_EXECUTOR.start(interrupt::SWI_IRQ_1);

    // Spawn the task responsible for pushing data to the PIO FIFO. This task runs on the interrupt
    // executor so that it preempts other tasks and the FIFO is always topped up.
    interrupt_spawner
        .spawn(pio_task(
            Pio::new(p.PIO0, Irqs),
            p.PIN_6,
            p.PIN_7,
            p.PIN_8,
            p.DMA_CH0.into(),
        ))
        .unwrap();
    info!("Spawned PIO task");

    // Change the tone frequency, compute the audio samples and push them to the signal.
    let mut frequency = 300;
    loop {
        let start = Instant::now();
        let samples = generate_samples(frequency, -20.0);
        info!(
            "Calculated audio samples for frequency {}Hz in {}us",
            frequency,
            start.elapsed().as_micros(),
        );
        SIGNAL.signal(samples);

        frequency += 100;
        if frequency > 1000 {
            frequency = 300
        }
        Timer::after_millis(500).await;
    }
}

#[embassy_executor::task]
async fn pio_task(
    mut pio: Pio<'static, PIO0>,
    data_pin: peripherals::PIN_6,
    bit_clock_pin: peripherals::PIN_7,
    left_right_clock_pin: peripherals::PIN_8,
    dma_channel: AnyChannel,
) {
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

    let mut dma_ref = dma_channel.into_ref();
    let tx = pio.sm0.tx();

    let mut buf = Vec::<u32, BUFFER_SIZE>::from_slice(&[0, 0]).unwrap();
    loop {
        // If there is a new set of samples, replace the buffer with the new values.
        if let Some(samples) = SIGNAL.wait().now_or_never() {
            buf = samples;
        }

        // Push the values to the state machine FIFO using DMA.
        tx.dma_push(dma_ref.reborrow(), &buf).await;
    }
}

/// Generates one period of audio samples of a sine wave of the given frequency.
pub fn generate_samples(frequency: u32, volume_db: f32) -> Vec<u32, BUFFER_SIZE> {
    let mut current = 0.0;
    let delta = frequency as f32 * TAU / SAMPLE_RATE as f32;
    let amplitude = libm::powf(10.0, volume_db / 20.).min(1.0) * i32::MAX as f32;

    let mut buffer = Vec::new();

    while current < TAU {
        let val = (lookup_sine(current) * amplitude) as i32 as u32;
        buffer.push(val).unwrap(); // Left channel
        buffer.push(val).unwrap(); // Right channel
        current += delta;
    }
    buffer
}

fn lookup_sine(angle: f32) -> f32 {
    const TABLE_SIZE: usize = SINE_TABLE.len();
    let index = angle / TAU * TABLE_SIZE as f32;
    SINE_TABLE[index as usize % TABLE_SIZE]
}
