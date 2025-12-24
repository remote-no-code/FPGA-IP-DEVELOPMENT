# FPGA-IP-DEVELOPMENT
# Task-1: Environment Setup & RISC-V Reference Bring-Up

# FPGA-IP-DEVELOPMENT

This repository documents the work completed as part of the **VSD RISC-V FPGA IP Development Program**, covering **Task-1** and **Task-2**.

All development and validation were performed using **GitHub Codespaces (Linux)**. FPGA hardware execution is planned for a later stage.

---

## Task-1: Environment Setup & RISC-V Bring-Up

In Task-1, the RISC-V development environment was successfully verified.

- RISC-V toolchain (`riscv64-unknown-elf-gcc`) was validated
- Spike ISA simulator was verified and used
- A reference RISC-V program was compiled and executed using Spike
- VSDFPGA firmware was built successfully, generating `riscv_logo.bram.hex`
- Firmware execution was validated through simulator-based output

This confirms a working software toolchain and firmware build flow.

---

## Task-2: Memory-Mapped GPIO IP Integration

In Task-2, a simple **memory-mapped GPIO output IP** was designed and integrated into the existing RISC-V SoC.

- The existing SoC architecture and bus interface were studied
- A 32-bit GPIO register was implemented
- The GPIO IP was memory-mapped into the SoC I/O region
- Read and write access from the CPU was enabled
- Firmware was updated to write to and read from the GPIO register
- Correct integration was validated through successful firmware build

Simulation-level validation was completed. FPGA hardware validation will be performed once the board is available.

---

## Status

- ✅ RISC-V toolchain verified  
- ✅ Simulator-based execution validated  
- ✅ VSDFPGA firmware build working  
- ✅ Memory-mapped GPIO IP integrated  

---

## Notes

- Platform used: **GitHub Codespaces**
- FPGA flashing and hardware testing were intentionally skipped
- This repository will be extended with further tasks
