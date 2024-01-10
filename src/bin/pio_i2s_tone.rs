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
use embassy_time::Timer;
use fixed::{traits::ToFixed, types::I16F16};
use futures::future::FutureExt;
use panic_probe as _;

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
});

static INTERRUPT_EXECUTOR: InterruptExecutor = InterruptExecutor::new();

const SAMPLE_RATE: u32 = 8_000;
const BIT_DEPTH: u32 = 16;
const CHANNELS: u32 = 2;
const BUFFER_SIZE: usize = (SAMPLE_RATE * CHANNELS) as usize / 200; // 5ms buffer

// Signal used to communicate between main() and the PIO task.
static SIGNAL: Signal<CriticalSectionRawMutex, f32> = Signal::new();

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

    let mut frequency = 250f32;
    let mut delta = 5.0;
    loop {
        SIGNAL.signal(frequency);

        frequency += delta;
        if frequency > 1500.0 {
            delta = -5.0;
        } else if frequency < 250.0 {
            delta = 5.0;
        }
        Timer::after_millis(10).await;
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
        "    set x, 14          side 0b01", // side 0bWB - W = Word Clock, B = Bit Clock
        "left_data:",
        "    out pins, 1        side 0b00",
        "    jmp x-- left_data  side 0b01",
        "    out pins 1         side 0b10",
        "    set x, 14          side 0b11",
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
            threshold: 16,
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

    let mut frequency = 300.0;
    let mut generator = ToneGenerator::new();

    loop {
        // Check if the frequency should be changed
        if let Some(new_frequency) = SIGNAL.wait().now_or_never() {
            frequency = new_frequency;
        }

        // Generate the audio samples
        generator.generate_samples(frequency, 0.1);

        // Push the values to the state machine FIFO using DMA.
        tx.dma_push(dma_ref.reborrow(), &generator.buffer).await;
    }
}

struct ToneGenerator {
    phase: I16F16,
    buffer: [u16; BUFFER_SIZE],
}

impl ToneGenerator {
    fn new() -> Self {
        Self {
            phase: I16F16::ZERO,
            buffer: [0u16; BUFFER_SIZE],
        }
    }

    fn generate_samples(&mut self, frequency: f32, volume: f32) {
        let delta = (frequency * TAU / SAMPLE_RATE as f32).to_fixed::<I16F16>();
        let amplitude = I16F16::from_num(volume);

        for frame in self.buffer.chunks_mut(2) {
            let sine_fixed = I16F16::from_num(lookup_sine(self.phase));
            let value = (sine_fixed * amplitude).to_num::<i16>() as u16;
            frame[0] = value;
            frame[1] = value;

            self.phase += delta;
            if self.phase > I16F16::TAU {
                self.phase -= I16F16::TAU;
            }
        }
    }
}

fn lookup_sine(angle: I16F16) -> i16 {
    const TABLE_SIZE: usize = SINE_TABLE.len();
    let index = (angle / I16F16::TAU * TABLE_SIZE.to_fixed::<I16F16>())
        .round()
        .to_num::<usize>();
    SINE_TABLE[index % TABLE_SIZE]
}
