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
use panic_probe as _;

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
});

static INTERRUPT_EXECUTOR: InterruptExecutor = InterruptExecutor::new();

const SAMPLE_RATE: u32 = 8_000;
const BIT_DEPTH: u32 = 16;
const CHANNELS: u32 = 2;
const SAMPLE_COUNT: usize = 32;
const BUFFER_SIZE: usize = SAMPLE_COUNT * CHANNELS as usize;

// Signal used to communicate between main() and the PIO task.
static SIGNAL: Signal<CriticalSectionRawMutex, f32> = Signal::new();

// Table of sine values to speed up calculation of the audio samples.
include!(concat!(env!("OUT_DIR"), "/sine_table.rs"));
const LUT_SIZE: usize = SINE_TABLE.len();

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
        Timer::after_millis(15).await;
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

    let mut generator = ToneGenerator::new();
    generator.set_volume(0.005);

    loop {
        // Check if the frequency should be changed
        if let Some(new_frequency) = SIGNAL.wait().now_or_never() {
            generator.set_frequency(new_frequency);
        }

        // Generate the audio samples
        let start = Instant::now();
        generator.generate_samples();
        let elapsed = start.elapsed().as_micros();
        info!("Elapsed: {}us", elapsed);

        // Push the values to the state machine FIFO using DMA.
        tx.dma_push(dma_ref.reborrow(), &generator.buffer).await;
    }
}

struct ToneGenerator {
    offset: i32,
    increment: i32,
    amplitude: u16,
    new_amplitude: Option<u16>,
    prev_val: i16,
    buffer: [u16; BUFFER_SIZE],
}

impl ToneGenerator {
    fn new() -> Self {
        Self {
            offset: 0,
            increment: 0,
            amplitude: u16::MAX,
            new_amplitude: None,
            prev_val: 0,
            buffer: [0u16; BUFFER_SIZE],
        }
    }

    fn set_frequency(&mut self, frequency: f32) {
        let samp_per_cyc = SAMPLE_RATE as f32 / frequency;
        let fincr = LUT_SIZE as f32 / samp_per_cyc;
        let incr = (((1 << 24) as f32) * fincr) as i32;
        self.increment = incr;
    }

    fn set_volume(&mut self, amplitude: f32) {
        let amplitude = amplitude.clamp(0.0, 1.0);
        let amplitude = (amplitude * u16::MAX as f32) as u16;
        self.new_amplitude = Some(amplitude);
    }

    // Code from https://jamesmunns.com/blog/fixed-point-math/
    fn generate_samples(&mut self) {
        for frame in self.buffer.chunks_mut(2) {
            let val = self.offset as u32;
            let idx_now = ((val >> 24) & 0xFF) as u8;
            let idx_nxt = idx_now.wrapping_add(1);
            let base_val = SINE_TABLE[idx_now as usize] as i32;
            let next_val = SINE_TABLE[idx_nxt as usize] as i32;
            let off = ((val >> 16) & 0xFF) as i32;
            let cur_weight = base_val.wrapping_mul((LUT_SIZE as i32).wrapping_sub(off));
            let nxt_weight = next_val.wrapping_mul(off);
            let ttl_weight = cur_weight.wrapping_add(nxt_weight);
            let ttl_val = (ttl_weight >> 8) as i16;

            if let Some(new_amplitude) = self.new_amplitude {
                if ttl_val.signum() != self.prev_val.signum() {
                    // This is the first sample after a zero crossing,
                    // change the gain now so it's doesn't cause clicking.
                    self.amplitude = new_amplitude;
                    self.new_amplitude = None;
                }
            }
            self.prev_val = ttl_val;

            let ttl_val = ((ttl_val as i32 * self.amplitude as i32) >> 16) as u16;

            frame[0] = ttl_val;
            frame[1] = ttl_val;

            self.offset = self.offset.wrapping_add(self.increment);
        }
    }
}
