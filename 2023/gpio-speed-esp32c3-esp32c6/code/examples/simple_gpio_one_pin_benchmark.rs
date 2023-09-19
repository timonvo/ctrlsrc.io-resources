#![no_std]
#![no_main]

#[cfg(feature = "esp32c3")]
use esp32c3_hal as hal;
#[cfg(feature = "esp32c6")]
use esp32c6_hal as hal;
// Provides a panic_handler implementation that prints to UART.
use esp_backtrace as _;
use hal::{clock::ClockControl, gpio::IO, peripherals::Peripherals, prelude::*};
use log::info;

/// A simple benchmark for gauging the upper bound on the GPIO speed of ESP32-C6 and similar
/// devices, when using the standard "Simple GPIO" output method via the standard GPIO API.
///
/// Simply toggles a single GPIO pin in a busy loop.
///
/// On my ESP32-C6 this achieves a signal frequency of about 1.74 MHz (from 0 to 1 and back to 0),
/// with with a duty cycle of approximately 48%, as measured on an oscilloscope. The observed
/// waveform is nice and square at these frequencies, with a rise time (10% to 90%) of about 3.3 ns.
///
/// On my ESP32-C3 this achieves a signal frequency of 4.22 MHz, with a duty cycle of around 47%. The
/// waveform looks similarly square, but with a little more ringing.
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
    benchmark_simple_gpio_one_pin(io)
}

// Ensures that the code is placed in the RAM at boot, to avoid any possible delays caused by having
// to load the code from flash at runtime.
#[ram]
fn benchmark_simple_gpio_one_pin(io: IO) -> ! {
    // Set GPIO10 as output.
    let mut gpio10 = io.pins.gpio10.into_push_pull_output();

    // Toggle the pin's state in a busy loop.
    loop {
        gpio10.set_high().unwrap();
        gpio10.set_low().unwrap();
    }
}
