# FPGA-IP-DEVELOPMENT

This repository documents the work completed as part of the **VSD RISC-V FPGA IP Development Program**, covering **Task-1**, **Task-2**, **Task-3**, and **Task-4**.

All development and validation were performed using **GitHub Codespaces (Linux)**.
FPGA hardware validation was completed using the VSDSquadron board.

***

## Task-1: Environment Setup & RISC-V Bring-Up

In Task-1, the RISC-V development environment was successfully verified.

- RISC-V toolchain (`riscv64-unknown-elf-gcc`) was validated
- Spike ISA simulator was verified and used
- A reference RISC-V program was compiled and executed using Spike
- VSDFPGA firmware was built successfully, generating `riscv_logo.bram.hex`
- Firmware execution was validated through simulator-based output

This confirms a working software toolchain and firmware build flow.

***

## Task-2: Memory-Mapped GPIO IP Integration

In Task-2, a simple **memory-mapped GPIO output IP** was designed and integrated into the existing RISC-V SoC.

- The existing SoC architecture and bus interface were studied
- A 32-bit GPIO register was implemented
- The GPIO IP was memory-mapped into the SoC I/O region
- Read and write access from the CPU was enabled
- Firmware was updated to write to and read from the GPIO register
- Correct integration was validated through successful firmware build

Simulation-level validation was completed. FPGA hardware validation will be performed once the board is available.

***

## Task-3: Multi-Register GPIO IP with Software Control

In Task-3, the simple GPIO IP from Task-2 was extended into a **realistic, production-style GPIO peripheral** with multiple registers and full software control.

- A **multi-register GPIO IP** was designed with a clear register map:
  - `GPIO_DATA` for output data
  - `GPIO_DIR` for per-pin direction control
  - `GPIO_READ` for pin state readback
- Proper **address offset decoding** was implemented inside the IP
- Direction control logic was added to distinguish input vs output behavior
- The enhanced GPIO IP was **integrated into the RISC-V SoC**
- Firmware was written to:
  - Configure GPIO direction
  - Write output values
  - Read back GPIO state
- End-to-end validation was completed:
  - **Simulation validation** using Icarus Verilog
  - **FPGA hardware validation** using the VSDSquadron board
- GPIO outputs were successfully driven to **physical LEDs**, confirming correct software-to-hardware operation

This task strengthened understanding of **memory-mapped I/O**, **register-level peripheral design**, and **full-stack SoC validation** from C software to FPGA hardware.

***

## Task-4: Timer IP Development and Hardware Validation

In Task-4, a **programmable Timer IP** was designed and integrated as a real peripheral into the RISC-V SoC.

- A **memory-mapped Timer IP** was implemented with a well-defined register interface:
  - `TIMER_CTRL` for enable and mode control
  - `TIMER_LOAD` for countdown initialization
  - `TIMER_VALUE` for current count readback
  - `TIMER_STAT` for sticky timeout status (W1C)
- One-shot and periodic operating modes were supported
- Proper **register decoding and counter logic** were implemented
- The Timer IP was integrated into the SoC address map
- Firmware was written to configure and control the timer via polling
- End-to-end validation was completed:
  - **RTL simulation** using GTKWave
  - **FPGA hardware validation** using the VSDSquadron board
- Timer-generated timeout events were used to drive **hardware LED toggling**, demonstrating true hardware-timed behavior independent of software delay loops

This task validated the complete workflow of **IP design, documentation, software control, and real FPGA hardware verification**.

***

## Status

- ✅ RISC-V toolchain verified
- ✅ Simulator-based execution validated
- ✅ VSDFPGA firmware build working
- ✅ Memory-mapped GPIO IP integrated
- ✅ Multi-register GPIO IP validated in simulation and hardware
- ✅ Timer IP integrated and validated in simulation and FPGA hardware

***

## Notes

- Platform used: **GitHub Codespaces**
- This repository will be extended with further tasks
