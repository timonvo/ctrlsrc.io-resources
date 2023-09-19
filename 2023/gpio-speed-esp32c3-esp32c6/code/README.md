# Code samples for the "The GPIO speed of ESP32-C3 and ESP32-C6 microcontrollers" post on ctrlsrc.io

See https://ctrlsrc.io/posts/2023/gpio-speed-esp32c3-esp32c6/.

These are a series of benchmarks that can run on ESP32-C3 or ESP32-C6
microcontrollers. The chips' GPIO10 output is intended to be observed with an
oscilloscope while the benchmarks are running (see the ../screenshots
directory).

This code uses the [esp-hal](https://github.com/esp-rs/esp-hal) framework. To
run the commands below you'll need to install `espflash` and `espmonitor`. See
the [Rust on ESP book](https://esp-rs.github.io/book/introduction.html) for more
info.

Examples can be run on ESP32-C6 using

```sh
cargo run --release --target riscv32imac-unknown-none-elf --features esp32c6 --example {the_example_name}
```

Examples can be run on ESP32-C3 using

```sh
cargo run --release --target riscv32imc-unknown-none-elf --features esp32c3 --example {the_example_name}
```

The generated code can be inspected using `objdump`. E.g for ESP32-C3 binaries:

```sh
riscv64-unknown-elf-objdump -d target/riscv32imc-unknown-none-elf/release/examples/{the_example_name}
```

or using the `disassemble_function.sh` script. E.g. for ESP32-C6 binaries:

```sh
./disassembly_function.sh riscv32imac {the_example_name} {the_function_to_disassemble}
```
