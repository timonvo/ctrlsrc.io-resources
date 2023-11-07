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
/// devices, when using the "Dedicated IO" feature rather than the "Simple GPIO output" method.
/// In this benchmark we unroll the toggling loop 8 times, to reduce the impact of the jump
/// instruction's CPU cycle on the achieved output signal frequency.
///
/// On my ESP32-C6 this achieves a signal frequency of about 80 MHz (from 0 to 1 and back to 0)
/// during the unrolled loop portion of the code, with a duty cycle of close to 50%, as measured on
/// an oscilloscope. This confirm the technical reference manual's claim that the GPIO pins can be
/// driven within a single CPU cycle. Because we need to execute a jump instruction for perform the
/// benchmark in a loop, we cannot sustain the 80 MHz signal frequency forever, however.
///
/// The output waveform is quite sinusoidal, which is to be expected given that our other simple
/// GPIO benchmarks already established a rise time (10% to 90%) of about 3ns. Since at 80MHz the
/// signal period is only ~12.5 ns, each high and low signal level is only held for 6.25 ns, meaning
/// we're practically going from rising edge straight to falling edge.
///
/// I get similar results on my ESP32-C3.
///
/// For more info on ESP32-C6's "Dedicated IO" feature, see
/// https://docs.espressif.com/projects/esp-idf/en/latest/esp32c6/api-reference/peripherals/dedic_gpio.html
/// and the "1.14 Dedicated IO" chapter of the technical reference manual. The ESP32-C3 supports the
/// feature as well, but it's not documented in as much detail in the manual. The implementation
/// below is loosely based on esp-idf's C implementation at
/// https://github.com/espressif/esp-idf/blob/324725/components/driver/gpio/dedic_gpio.c#L193.
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
    // Set GPIO10 as an output. As per ESP-IDF's docs, we must do this even though we'll access them
    // via the Dedicated IO feature.
    let mut gpio10 = io.pins.gpio10.into_push_pull_output();
    // The CPU peripheral signal is named slightly differently for ESP32-C3 and C6 chips.
    #[cfg(feature = "esp32c3")]
    const OUTPUT_SIGNAL: hal::gpio::OutputSignal = hal::gpio::OutputSignal::CPU_GPIO_0;
    #[cfg(feature = "esp32c6")]
    const OUTPUT_SIGNAL: hal::gpio::OutputSignal = hal::gpio::OutputSignal::CPU_GPIO_OUT0;
    // Connect GPIO10 to the CPU_GPIO_OUT0 peripheral signal.
    gpio10.connect_peripheral_to_output_with_options(
        OUTPUT_SIGNAL,
        false,
        false,
        // Ensures that the pin input/output status is determined by the GPIO matrix configuration,
        // rather than the CPU_GPIO_OEN CSR.
        /* enable_from_gpio: */
        true,
        false,
    );

    // Toggle the pin in a busy loop.
    unsafe {
        asm!(
            ".macro toggle_pin
            // Set the pin high.
            csrrsi zero, {csr_cpu_gpio_out}, {cpu_gpio_signal}
            // Set the pin low.
            csrrci zero, {csr_cpu_gpio_out}, {cpu_gpio_signal}
            .endm

            // Ensures that the '1' label corresponds to a 4 byte-aligned address,
            // thereby allowing the `j` instruction to take only a single CPU cycle on ESP32-C6
            // chips.
            .align 4
            1:
            // Toggle the pin 8 times, and then loop back. This manual unrolling of the loop
            // ensures we can gauge the absolute maximum achievable speed, even if we can only
            // reach it for short bursts of time.
            toggle_pin,
            toggle_pin,
            toggle_pin,
            toggle_pin,
            toggle_pin,
            toggle_pin,
            toggle_pin,
            toggle_pin,
            // Loop back to label 1.
            j 1b",
            csr_cpu_gpio_out = const(CSR_CPU_GPIO_OUT),
            cpu_gpio_signal = const(1),// CPU_GPIO_OUT0 maps to bit index #1.
            options(noreturn)
        );
    }
}
