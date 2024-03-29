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
///
/// On my ESP32-C6 this achieves a signal frequency of about 53 MHz (from 0 to 1 and back to 0)
/// during the unrolled loop portion of the code, with a duty cycle around 35%, as measured on an
/// oscilloscope. This is because the jump instruction takes up one CPU cycle, with the actual
/// toggling taking up the two other CPU cycles in the loop.
///
/// Note that the '.align 4' statement below is crucial to achieving this three-CPU cycle loop.
/// Without it the `j` instruction can take up to two CPU cycles if the target address is
/// misaligned, resulting in a frequency of only 40MHz and a duty cycle of 25%.
///
/// On my ESP32-C3 this achieves a signal frequency of about 40MHz and a duty cycle of 25%,
/// seemingly indicating that on the ESP32-C3 the jump takes two CPU cycles even when the target is
/// 4 byte-aligned. If I ensure the target is *not* 4 byte-aligned, then the signal frequency is
/// 32MHz, which seems to indicate that unaligned jumps on ESP32-C3 thus take three cycles (one more
/// than aligned jumps).
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
        asm!("
            // Ensures that the '1' label corresponds to a 4 byte-aligned address,
            // thereby allowing the `j` instruction to take only a single CPU cycle on ESP32-C6
            // chips.
            .align 4
            1:
            // Set the pin high.
            csrrsi zero, {csr_cpu_gpio_out}, {cpu_gpio_signal}
            // Set the pin low.
            csrrci zero, {csr_cpu_gpio_out}, {cpu_gpio_signal}
            // Loop back to label 1.
            j 1b",
            csr_cpu_gpio_out = const(CSR_CPU_GPIO_OUT),
            cpu_gpio_signal = const(1),// CPU_GPIO_OUT0 maps to register bit 0.
            options(noreturn)
        );
    }
}
