# Task-1: Environment Setup & RISC-V Reference Bring-Up

## Environment Used
GitHub Codespace (Linux)

## RISC-V Reference Program
- Repository: vsd-riscv2
- Program Location: samples/
- Successfully compiled and executed using Spike
- Output: "Sum from 1 to 9 is 45"

## VSDFPGA Labs
- Repository: vsdfpga_labs
- Lab: basicRISCV Firmware build
- Generated riscv_logo.bram.hex successfully
- FPGA flashing and hardware steps skipped as per Task-1 instructions

## Understanding Check

### 1. Where is the RISC-V program located?
The RISC-V program is located in the `samples` directory of the vsd-riscv2 repository.

### 2. How is the program compiled and loaded into memory?
It is cross-compiled using `riscv64-unknown-elf-gcc` and executed using Spike with the proxy kernel, which loads the ELF into simulated memory.

### 3. How does the RISC-V core access memory and memory-mapped IO?
The core uses standard RISC-V load/store instructions over the system bus, with peripherals accessed through memory-mapped IO addresses.

### 4. Where would a new FPGA IP block logically integrate?
A new FPGA IP block would integrate as a memory-mapped peripheral connected to the SoC interconnect.

## Notes
GitHub Codespace was used throughout. FPGA tools and hardware flashing were intentionally not used, as instructed.

