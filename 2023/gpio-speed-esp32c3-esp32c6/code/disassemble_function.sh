#!/bin/sh

# Usage:
#
#   ./disassemble_function.sh {target} {the_example} {the_function_to_disassemble}
#
# For example:
#
#   ./disassemble_function.sh riscv32imac simple_gpio_one_pin_benchmark benchmark_simple_gpio_one_pin

riscv64-unknown-elf-objdump -d "target/$1-unknown-none-elf/release/examples/$2" \
  | awk -v RS= "/^[[:xdigit:]]+ <[^<]+$3[^>]+>/"