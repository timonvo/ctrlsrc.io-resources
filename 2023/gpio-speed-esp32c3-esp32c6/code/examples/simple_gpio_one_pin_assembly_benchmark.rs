#![no_std]
#![no_main]

use core::arch::asm;
#[cfg(feature = "esp32c3")]
use esp32c3_hal as hal;
#[cfg(feature = "esp32c6")]
use esp32c6_hal as hal;
// Provides a panic_handler implementation that prints to UART.
use esp_backtrace as _;
use hal::{clock::ClockControl, gpio::IO, peripherals::Peripherals, prelude::*};
use log::info;

/// A simple benchmark for gauging the upper bound on the GPIO speed of ESP32-C6 and similar
/// devices, when using the standard "Simple GPIO" output method but using assembly instructions to
/// modify the GPIO registers, instead of using the standard APIs,.
///
/// Simply toggles a single GPIO pin in a busy loop.
///
/// On my ESP32-C6 this achieves a signal frequency of between 2 to 2.22 MHz (from 0 to 1 and back
/// to 0), with a duty cycle close to 50%, as measured on an oscilloscope. The observed
/// waveform is nice and square at these frequencies, with a rise time (10% to 90%) of about 3.3 ns.
/// According to Section 14.1 of the ESP32-C6 technical reference manual the bottleneck here is the
/// APB bus.
///
/// On my ESP32-C3 this achieves a signal frequency of 8.85 MHz, with a duty cycle around 44%. The
/// waveform looks a bit less square and exhibits more ringing.
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
    benchmark_simple_gpio_one_pin_with_assembly(io)
}

// Ensures that the code is placed in the RAM at boot, to avoid any possible delays caused by having
// to load the code from flash at runtime.
#[ram]
fn benchmark_simple_gpio_one_pin_with_assembly(mut io: IO) -> ! {
    // Set GPIO10 as output.
    io.pins.gpio10.set_to_push_pull_output();

    // The GPIO matrix address for ESP32-C3, see "Table 3-­3" in the technical reference manual.
    #[cfg(feature = "esp32c3")]
    const GPIO_MATRIX_ADDRESS: usize = 0x6000_4000;
    // The GPIO matrix address for ESP32-C6, see "Table 5-2" in the technical reference manual.
    #[cfg(feature = "esp32c6")]
    const GPIO_MATRIX_ADDRESS: usize = 0x6009_1000;

    // Toggle the pin's state in a busy loop.
    unsafe {
        asm!("
            // Ensures that the '1' label corresponds to a 2-byte aligned address,
            // thereby allowing the `j` instruction to take only a single CPU cycle on ESP32-C6
            // chips.
            .align 2
            1:
            // Set the pin high.
            sw {gpio_pin}, 0x8({gpio_matrix}) // 0x8 is the GPIO_OUT_W1TS_REG offset.
            // Set the pin low.
            sw {gpio_pin}, 0xc({gpio_matrix}) // 0xc is the GPIO_OUT_W1TC_REG offset.
            // Loop back to label 1.
            j 1b",
            // GPIO10 corresponds to bit 11 in the GPIO_OUT_W1TS_REG and
            // GPIO_OUT_W1TC_REG registers.
            gpio_pin = in(reg) 1 << 10,
            gpio_matrix = in(reg) GPIO_MATRIX_ADDRESS,
            // The assembly code will loop forever and never return.
            options(noreturn)
        );
    }
}
