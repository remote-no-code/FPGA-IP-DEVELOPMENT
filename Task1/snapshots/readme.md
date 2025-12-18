Task 1: RISC-V Toolchain & Simulator Verification

This repository contains the evidence for Task 1, demonstrating the successful setup and verification of the RISC-V toolchain, simulators, and firmware build process within a GitHub Codespace environment.

📸 Evidence Gallery

1. Compiler & Simulator Version Check

This screenshot verifies that the RISC-V cross-compiler and the Verilog simulator are correctly installed and accessible in the system path.

Compiler: riscv64-unknown-elf-gcc

Simulator: iverilog

2. Spike Simulator Verification

Since the current Spike build does not support the --version flag, the help command is used to demonstrate that the ISA simulator is installed and responding to commands.

3. Initial RISC-V Program Execution

Validation of the toolchain by running a "Sum 1 to N" program. This confirms that the object files are being linked and executed correctly by the Spike proxy kernel (pk).

Output: Sum from 1 to 9 is 45

4. Modified Program Execution

To demonstrate workflow proficiency, the source code was modified to calculate the sum from 1 to 10. The output confirms successful recompilation and execution of the updated logic.

Output: Sum from 1 to 10 is 55

5. VSDFPGA Firmware Build

This screenshot confirms the successful generation of the BRAM hex file for the VSDFPGA labs. It demonstrates that the build system can successfully produce the riscv_logo.bram.hex file required for FPGA deployment.

✅ Summary of Verification

The screenshots above provide definitive proof of the following:

[x] Environment Setup: Toolchain and dependencies are correctly configured.

[x] Functional Simulation: Spike simulator is operational and capable of running RISC-V binaries.

[x] Workflow Validation: Ability to modify, compile, and re-run C-based RISC-V programs.

[x] Firmware Generation: Successful execution of the hardware-specific build process for FPGA.

Generated as part of the VSD-FPGA Task-1 requirements.
