# Task-1: RISC-V Toolchain & Simulator Verification

This repository contains the evidence and documentation for **Task-1**, demonstrating the successful setup, verification, and usage of the **RISC-V toolchain, simulators, and firmware build flow** within a **GitHub Codespace (Linux)** environment.

The objective of this task was to validate a known-good RISC-V reference flow and ensure readiness for future RTL, IP, and FPGA integration tasks.

---

## Environment Used
- **Platform:** GitHub Codespace
- **OS:** Linux
- **Execution Interface:** VS Code Terminal inside Codespace

All commands and builds were executed inside the Codespace environment as instructed.

---

## 📸 Evidence Gallery

### 1. Compiler & Simulator Version Check  
**Screenshot:** `toolchain_versions.png`

This screenshot verifies that the required tools are correctly installed and accessible via the system PATH:
- **RISC-V Cross Compiler:** `riscv64-unknown-elf-gcc`
- **Verilog Simulator:** `iverilog`

This confirms that the base toolchain environment is correctly configured.

---

### 2. Spike Simulator Verification  
**Screenshot:** `spike_help.png`

The Spike ISA simulator in this environment does not support the `--version` flag.  
Instead, the `--help` command is used to confirm that Spike is installed and responding correctly to user commands.

This verifies the availability of the RISC-V ISA simulator.

---

### 3. Initial RISC-V Program Execution  
**Screenshot:** `riscv_sum_1_to_9.png`

A simple **Sum 1 to N** RISC-V program was compiled and executed using the following flow:
```bash
riscv64-unknown-elf-gcc -o sum1ton.o sum1ton.c
spike pk sum1ton.o
Observed Output:

Sum from 1 to 9 is 45


This confirms:

Correct compilation

Proper ELF loading via the Spike proxy kernel (pk)

Successful execution on the RISC-V simulator

4. Modified Program Execution (Optional Confidence Task)

Screenshot: riscv_sum_1_to_10.png

To validate workflow confidence, the source code was modified to calculate the sum from 1 to 10 instead of 1 to 9.
The program was recompiled and executed successfully.

Observed Output:

Sum from 1 to 10 is 55


This demonstrates the ability to:

Modify RISC-V C code

Rebuild the program

Observe correct behavioral changes at runtime

5. VSDFPGA Firmware Build (No FPGA Hardware)

Screenshot: vsdfpga_firmware_build.png

This screenshot confirms the successful execution of the VSDFPGA lab firmware build process:

make riscv_logo.bram.hex


The generation of riscv_logo.bram.hex verifies:

Multi-repository workflow readiness

Correct integration of the RISC-V toolchain with the FPGA firmware build flow

FPGA flashing and hardware execution were intentionally skipped, as Task-1 does not require physical board usage.

✅ Summary of Verification

The evidence provided in this repository confirms:

Environment Setup: RISC-V toolchain and simulators are correctly installed and configured

Functional Simulation: Spike simulator successfully executes RISC-V binaries

Workflow Validation: Programs can be modified, recompiled, and re-executed correctly

Firmware Generation: VSDFPGA firmware artifacts are built successfully without FPGA hardware

Notes

GitHub Codespace was used throughout the task.

FPGA tools and board flashing were intentionally not performed, as per Task-1 instructions.

This task establishes a stable baseline for upcoming RTL, IP, and FPGA bring-up work.
