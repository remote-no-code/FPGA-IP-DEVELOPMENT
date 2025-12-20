# Task-1: RISC-V Toolchain & Simulator Verification

This repository contains the evidence and documentation for **Task-1**, demonstrating the successful setup, verification, and usage of the **RISC-V toolchain, simulators, and firmware build flow** within a **GitHub Codespace (Linux)** environment.

The objective of this task was to validate a known-good RISC-V reference flow and ensure readiness for future RTL, IP, and FPGA integration tasks.

---

## Environment Used
- **Platform:** GitHub Codespace
- **OS:** Linux
- **Execution Interface:** VS Code Terminal inside Codespace
- **Additional Interface:** noVNC graphical terminal provided by Codespace

All commands and builds were executed inside the Codespace environment as instructed. No local FPGA hardware was used.

---

## 📸 Evidence Gallery

### 1. Compiler & Simulator Version Check  
**Screenshot:** `toolchain_versions.png`

This screenshot verifies that the required tools are correctly installed and accessible via the system PATH:
- **RISC-V Cross Compiler:** `riscv64-unknown-elf-gcc`
- **Verilog Simulator:** `iverilog`

This confirms that the base RISC-V development environment is correctly configured.

---

### 2. Spike Simulator Verification  
**Screenshot:** `spike_help.png`

The Spike ISA simulator in this environment does not support the `--version` flag.  
Instead, the `--help` command is used to confirm that Spike is installed and responding correctly to user commands.

This verifies the availability of the RISC-V ISA simulator.

---

### 3. Initial RISC-V Program Execution  
**Screenshot:** `riscv_sum_1_to_9.png`

A simple **Sum from 1 to N** RISC-V program was compiled and executed using the following flow:

riscv64-unknown-elf-gcc -o sum1ton.o sum1ton.c  
spike pk sum1ton.o  

Observed output:  
Sum from 1 to 9 is 45

This confirms:
- Correct cross-compilation
- Proper ELF loading using the Spike proxy kernel
- Successful execution on the RISC-V simulator

---

### 4. Modified Program Execution (Optional Confidence Task)  
**Screenshot:** `riscv_sum_1_to_10.png`

To further validate confidence with the toolchain, the source code was modified to compute the sum from **1 to 10** instead of **1 to 9**. The program was recompiled and executed using Spike.

Observed output:  
Sum from 1 to 10 is 55

This demonstrates the ability to:
- Modify RISC-V C source code
- Rebuild using the cross-compiler
- Observe correct behavioral changes through simulation

---

## VSDFPGA Firmware Build (No FPGA Hardware)

**Screenshot:** `vsdfpga_firmware_build.png`

The VSDFPGA lab firmware was built successfully using the following command:

make riscv_logo.bram.hex

The build process generated the file `riscv_logo.bram.hex`, confirming:
- Correct integration of the RISC-V toolchain
- Readiness for SoC/FPGA firmware workflows
- Successful multi-repository usage inside the same environment

FPGA flashing and physical hardware execution were intentionally skipped, as Task-1 does not require board-level validation.

---

## VSDFPGA RISC-V Logo Execution Using Spike (Simulator-Based)

**Screenshot:** `spike_riscv_logo_output.png`

In addition to firmware HEX generation, the VSDFPGA firmware source (`riscv_logo.c`) was compiled into an ELF and executed using the Spike simulator with proxy kernel support.

The execution was performed using:

riscv64-unknown-elf-gcc -o riscv_logo.o riscv_logo.c  
spike pk riscv_logo.o  

The screenshot shows the ASCII RISC-V logo being printed repeatedly, matching the banner defined inside `riscv_logo.c`:

LEARN TO THINK LIKE A CHIP  
VSDSQUADRON FPGA MINI  
BRINGS RISC-V TO VSD CLASSROOM  

In the screenshot:
- The **left side** shows execution inside the **VS Code terminal** within GitHub Codespaces.
- The **right side** shows the same output displayed through the **noVNC graphical terminal** provided by the Codespace environment.

This confirms:
- Correct firmware logic
- Successful compilation of VSDFPGA firmware
- Valid runtime behavior through simulation without FPGA hardware

On actual FPGA hardware, the same firmware would display the ASCII logo over a UART connection using `make terminal`. Hardware execution is intentionally out of scope for Task-1.

---



## Summary of Verification

The evidence provided in this repository confirms:

- **Environment Setup:**  
  RISC-V toolchain and simulators are correctly installed and configured.

- **Functional Simulation:**  
  Spike simulator successfully executes compiled RISC-V binaries.

- **Workflow Validation:**  
  Programs can be modified, recompiled, and re-executed correctly.

- **Firmware Generation:**  
  VSDFPGA firmware artifacts are built successfully without requiring FPGA hardware.

- **Firmware Execution Validation:**  
  VSDFPGA firmware behavior was validated through simulator-based execution using Spike, confirming correct ASCII logo generation without requiring FPGA hardware.

