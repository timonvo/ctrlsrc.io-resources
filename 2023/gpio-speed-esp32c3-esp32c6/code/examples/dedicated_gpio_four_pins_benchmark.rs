#![no_std]
#![no_main]
// We use this feature to write inline assembly with readable register names.
#![feature(asm_const)]

use core::arch::asm;
#[cfg(feature = "esp32c3")]
use esp32c3_hal as hal;
#[cfg(feature = "esp32c6")]
use esp32c6_hal as hal;
// Provides a panic_handler implementation that prints to UART.
use esp_backtrace as _;
use hal::{clock::ClockControl, gpio::IO, peripherals::Peripherals, prelude::*};
use log::info;

// This CSRs isn't defined by the esp-rs/esp-hal framework yet (nor by the espressif/svd files), so
// we hardcode their addresses. See the "1.14 Dedicated IO" chapter of the technical reference
// manual.
const CSR_CPU_GPIO_OUT: u32 = 0x805;

/// A simple benchmark for gauging the upper bound on the GPIO speed of ESP32-C6 and similar
/// devices, when using the "Dedicated IO" feature rather than the "Simple GPIO output" method. In
/// this benchmark we manipulate four output pins at once, with goal of generating four 40 MHz
/// waveforms with 50% duty cycles, two in phase and two 180 degrees out of phase. This shows how we
/// can manipulate multiple GPIO outputs in a single CPU cycle.
///
/// On my ESP32-C6 this goal is achieved. The resulting waveform is quite sinusoidal. On my ESP32-C3
/// I instead see a 32 MHz waveform with a duty cycle of 40/60%, which is due to the fact that the
/// the jump instruction on the C3 takes two cycles rather than one.
#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    #[cfg(feature = "esp32c3")]
    let system = peripherals.SYSTEM.split();
    #[cfg(feature = "esp32c6")]
    let system = peripherals.PCR.split();

    // Use the maximum available clock speed, since we want to maximize the GPIO speed we can
    // achieve.
    let clocks = ClockControl::max(system.clock_control).freeze();
    // Validate that this results in CPU clock of 160 MHz, APB clock of 80 MHz.
    assert_eq!(clocks.cpu_clock.to_MHz(), 160);
    assert_eq!(clocks.apb_clock.to_MHz(), 80);

    // Set up the logger.
    esp_println::logger::init_logger_from_env();
    info!("Booted up!");

    #[cfg(debug_assertions)]
    panic!("This binary must be compiled with --release mode in order to give accurate results!");

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    benchmark_dedicated_io(io)
}

// Ensures that the code is placed in the RAM at boot, to avoid any possible delays caused by having
// to load the code from flash at runtime.
#[ram]
fn benchmark_dedicated_io(io: IO) -> ! {
    // Set the GPIOs as an output. As per ESP-IDF's docs, we must do this even though we'll access
    // them via the Dedicated IO feature.
    #[cfg(feature = "esp32c6")]
    let (mut gpio_a, mut gpio_b, mut gpio_c, mut gpio_d) = (
        io.pins.gpio10.into_push_pull_output(),
        io.pins.gpio11.into_push_pull_output(),
        io.pins.gpio22.into_push_pull_output(),
        io.pins.gpio23.into_push_pull_output(),
    );
    #[cfg(feature = "esp32c3")]
    let (mut gpio_a, mut gpio_b, mut gpio_c, mut gpio_d) = (
        io.pins.gpio6.into_push_pull_output(),
        io.pins.gpio7.into_push_pull_output(),
        io.pins.gpio9.into_push_pull_output(),
        io.pins.gpio10.into_push_pull_output(),
    );
    // The CPU peripheral signal is named slightly differently for ESP32-C3 and C6 chips.
    #[cfg(feature = "esp32c6")]
    let (output_signal_a, output_signal_b, output_signal_c, output_signal_d) = (
        hal::gpio::OutputSignal::CPU_GPIO_OUT0,
        hal::gpio::OutputSignal::CPU_GPIO_OUT1,
        hal::gpio::OutputSignal::CPU_GPIO_OUT2,
        hal::gpio::OutputSignal::CPU_GPIO_OUT3,
    );
    #[cfg(feature = "esp32c3")]
    let (output_signal_a, output_signal_b, output_signal_c, output_signal_d) = (
        hal::gpio::OutputSignal::CPU_GPIO_0,
        hal::gpio::OutputSignal::CPU_GPIO_1,
        hal::gpio::OutputSignal::CPU_GPIO_2,
        hal::gpio::OutputSignal::CPU_GPIO_3,
    );
    // Connect the GPIOs to the CPU_GPIO_OUTn peripheral signals.
    gpio_a.connect_peripheral_to_output_with_options(
        output_signal_a,
        false,
        false,
        // Ensures that the pin input/output status is determined by the GPIO matrix configuration,
        // rather than the CPU_GPIO_OEN CSR.
        /* enable_from_gpio: */
        true,
        false,
    );
    gpio_b.connect_peripheral_to_output_with_options(output_signal_b, false, false, true, false);
    gpio_c.connect_peripheral_to_output_with_options(output_signal_c, false, false, true, false);
    gpio_d.connect_peripheral_to_output_with_options(output_signal_d, false, false, true, false);

    // Toggle the pins in groups of two in a busy loop.
    unsafe {
        asm!(
            "
            // Ensures that the '1' label corresponds to a 2-byte aligned address,
            // thereby allowing the `j` instruction to take only a single CPU cycle on ESP32-C6
            // chips.
            .align 2
            1:
            // Set pins A & B high and pins C & D low.
            csrrwi zero, {csr_cpu_gpio_out}, {ab_high_cd_low}
            // Wait one CPU cycle, to ensure the loop takes a total of four CPU cycles on ESP32-C6.
            // One ESP32-C3 the jump takes two cycles, for a total of five instructions per loop.
            nop
            // Set pins A & B low and pins C & D high.
            csrrwi zero, {csr_cpu_gpio_out}, {ab_low_cd_high}
            // Loop back to label 1.
            j 1b",
            csr_cpu_gpio_out = const(CSR_CPU_GPIO_OUT),
            ab_low_cd_high = const((1 << 0) | (1 << 1)),// CPU_GPIO_OUT0/OUT1 high.
            ab_high_cd_low = const((1 << 2) | (1 << 3)),// CPU_GPIO_OUT2/OUT3 high.
            options(noreturn)
        );
    }
}
