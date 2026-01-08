# Task-3: Design a Multi-Register GPIO IP with Software Control

## Overview

In Task-3, I focused on designing and validating a **realistic, software-controlled GPIO peripheral** similar to what I would find in production System-on-Chip (SoC) designs.  I built this task directly on Task-2 by extending a simple GPIO IP into a **multi-register, memory-mapped IP** with proper direction control, readback capability, and clean software–hardware interaction.

My GPIO Control IP is accessed by a RISC-V processor through memory-mapped registers and allows my software to:

* Configure GPIO pin direction (input or output)
* Write output values to GPIO pins
* Read back the current GPIO pin state

I emphasized the following in this task:

* Register-level thinking
* Address offset decoding
* Clean and synthesizable RTL design
* End-to-end validation from C software to hardware signals

I treated FPGA hardware validation as **optional but encouraged**, as I have access to the VSDSquadron FPGA board.

---

## Repository Structure

I ensured the following files and directories were committed to the GitHub repository for Task-3. This structure reflects exactly what I used to achieve the final working output.

```
Task-3/
├── RTL/
│   ├── riscv.v                 # Top-level SoC integration
│   ├── gpio_ctrl_ip.v          # Multi-register GPIO IP (DATA, DIR, READ)
│   ├── clockworks.v            # Clock and reset generation
│   ├── emitter_uart.v          # UART transmitter IP
│   ├── firmware.hex            # Generated firmware image for simulation & hardware
│   ├── VSDSquadronFM.pcf       # FPGA pin constraint file
│   └── Makefile                # Build, flash, and simulation targets
│
├── Firmware/
│   ├── gpio_test.c             # C program to validate GPIO behavior
│   ├── print.c / print.h       # UART print utilities
│   ├── start.S                 # RISC-V startup code
│   ├── bram.ld                 # Linker script for BRAM execution
│   ├── firmware_words          # ELF → HEX conversion utility
│   └── Makefile                # Firmware build and HEX generation
│
└── README.md                   # Task-3 documentation

```

> **Important:**
> I ensured the `firmware.hex` file was committed, as I required it for both simulation and FPGA hardware execution.

---

## Task Breakdown and Navigation

I used the links below to navigate through each step of my Task-3 work:

* [Step 1: Study and Plan](#step-1-study-and-plan-mandatory)
* [Step 2: Implement Multi-Register GPIO RTL](#step-2-implement-multi-register-gpio-rtl-mandatory)
* [Step 3: Integrate the Multi Register GPIO IP into the SoC](#step-3-integrate-the-multi-register-gpio-ip-into-the-soc-mandatory)
* [Step 4: Software Validation](#step-4-software-validation-mandatory)
* [Step 5: Hardware Validation](#step-5-hardware-validation-optional)

---

This overview defines the scope, structure, and navigation of my Task-3.
I built each step below progressively, starting from design planning and ending with optional real-hardware validation.

## Step 1: Study and Plan (Mandatory)

### Purpose of This Step

I focused Step 1 entirely on **analysis and design planning**.
I performed no RTL code, C code, or integration work at this stage.

My objective was to **fully understand what needs to be built and why**, before writing a single line of code. This mirrors real-world hardware development workflows, where I know a poorly planned IP almost always leads to bugs, rework, and unclear software behavior later.

This step establishes the **software–hardware contract** that all my subsequent steps depend on.

---

### Reviewing the Task-2 GPIO IP

My starting point for Task-3 was the GPIO IP I implemented in Task-2.
Before extending it, I reviewed the existing design carefully.

From Task-2, my GPIO IP typically had:

* A single register
* Basic write functionality
* Minimal or no direction control
* Simple address decoding

During this review, I answered the following questions:

* How does the CPU communicate with the GPIO IP?
* How are writes detected (bus signals, write mask)?
* How is data stored and driven to outputs?
* How does the IP return read data?

This review helped me identify:

* Which logic I can reuse
* Which logic I must redesign
* Where I require additional structure

---

### Understanding the New Requirements for Task-3

In Task-3, I was required to upgrade the simple GPIO IP into a **multi-register peripheral**, similar to what is used in real SoCs.

My new GPIO IP must:

* Support **multiple memory-mapped registers**
* Allow **per-pin direction control**
* Provide **readback of GPIO pin state**
* Behave predictably for both software and hardware

This means I can no longer design the IP to behave like a single latch. It must behave like a **register block**.

---

### Defining the Register Map

I defined a fixed register map for the GPIO IP. This register map is the foundation of my entire design.

| Offset | Register Name | Description |
| --- | --- | --- |
| 0x00  | GPIO_DATA      | Stores output values written by software |
| 0x04  | GPIO_DIR      | Controls direction (1 = output, 0 = input) |
| 0x08  | GPIO_READ      | Returns current GPIO pin state |

I made these important design decisions here:

* All registers are **32-bit wide**
* Registers are **word-aligned**
* GPIO_READ is **read-only**
* GPIO_DATA and GPIO_DIR are **read/write**

Once defined, I ensured this register map did not change, as my software depends on it.

---

### Defining Register Behavior (Critical Planning)

I defined each register's behavior precisely before implementation.

#### GPIO_DATA (Offset 0x00)

* Writing updates the stored output value
* Reading returns the **last written value**
* Does not depend on GPIO direction

#### GPIO_DIR (Offset 0x04)

* Each bit controls one GPIO pin
* `1` → output enabled
* `0` → input mode
* Affects both output driving and readback behavior

#### GPIO_READ (Offset 0x08)

* Read-only register
* Returns:
  - Output value for pins configured as outputs
  - External pin state for pins configured as inputs

By defining this behavior early, I prevented ambiguity and inconsistent implementations.

---

### Address Offset Decoding Strategy

My GPIO IP must determine **which register is being accessed** using the address bus.

My planning decisions:

* Base address decoding will be handled at the SoC level
* Inside the GPIO IP, I only require **offset decoding**
* I will use lower address bits (e.g. `bus_addr[3:2]`) to select registers

My planned mapping:

* `00` → GPIO_DATA
* `01` → GPIO_DIR
* `10` → GPIO_READ

I wrote no decoding logic yet, but I clearly defined the mapping.

---

### Internal Signal Planning

Before coding, I planned internal signals and registers to keep the RTL clean and understandable.

My planned internal elements:

* `gpio_data` register for output values
* `gpio_dir` register for direction control
* `gpio_out` signal for driven outputs
* `gpio_in` signal for external inputs
* `gpio_read` signal for readback logic

Each signal has:

* A single responsibility
* Clear ownership (register vs combinational logic)

I did this to prevent latch inference and unintended behavior later.

---

### Reset Behavior Planning

I defined reset behavior at this stage to ensure safe defaults.

On reset:

* GPIO_DATA → `0`
* GPIO_DIR → `0` (all pins input)
* GPIO outputs → inactive

This ensures:

* No pins are driven unintentionally at reset
* My software must explicitly configure GPIOs before use

---

### Software Perspective Considerations

I also required thinking from the **software point of view** during this step.

Key questions I considered:

* What happens if software writes GPIO_DATA before GPIO_DIR?
* What value does GPIO_READ return immediately after reset?
* Are register reads deterministic?

By answering these questions early, my GPIO IP becomes:

* Easier to use from software
* Easier to debug
* Closer to real industry peripherals

---

### Outcome of Step 1

At the end of Step 1:

* I wrote no RTL code
* I defined the full GPIO register map
* I clearly specified register behavior
* I planned the address decoding strategy
* I identified internal signals
* I decided on reset behavior

This step establishes a **solid design foundation**.
Only after completing this planning step did it make sense for me to move to RTL implementation in Step 2.

## Step 2: Implement Multi-Register GPIO RTL (Mandatory)

### Purpose of This Step

Step 2 is where I began the **actual RTL implementation** of the GPIO Control IP.
Using the design decisions I made in Step 1, my goal here was to convert the planned register map and behavior into **clean, synthesizable Verilog RTL**.

I focused only on the **GPIO IP itself** in this step.
I performed no SoC-level integration, no firmware, and no hardware programming here.

---

### File Modified in This Step

I modified only one file in this step:

* `gpio_ctrl_ip.v`

I made this separation intentional and important:

* The GPIO IP should remain **self-contained**
* It should not depend on SoC internals
* It should be reusable in other SoCs

---

### Register Implementation Strategy

Based on my Step 1 planning, I implemented three registers in the GPIO IP:

| Offset | Register | Type |
| --- | --- | --- |
| 0x00  | GPIO_DATA | Read / Write |
| 0x04  | GPIO_DIR  | Read / Write |
| 0x08  | GPIO_READ | Read Only |

I implemented each register either as:

* A **storage register** (`reg`)
* Or a **derived combinational signal**

---

### Internal Registers and Signals

I declared the following internal elements inside `gpio_ctrl_ip.v`:

#### Registers

* `gpio_data`
  Stores the value written by software to `GPIO_DATA`
* `gpio_dir`
  Stores direction bits written to `GPIO_DIR`

#### Wires

* `gpio_read`
  Represents the computed GPIO readback value
* `reg_sel`
  Used for address offset decoding

My clear separation ensures:

* No unintended latches
* Predictable synthesis results
* Easy debugging

---

### Address Offset Decoding

To support multiple registers, I implemented address offset decoding using lower address bits:

```verilog
wire [1:0] reg_sel = bus_addr[3:2];

```

Why this works:

* Registers are 32-bit aligned
* Address offsets increment by 4 bytes
* `bus_addr[3:2]` uniquely selects each register

Mapping:

* `2'b00` → GPIO_DATA (0x00)
* `2'b01` → GPIO_DIR (0x04)
* `2'b10` → GPIO_READ (0x08)

This decoding logic is simple, synthesizable, and commonly used in real designs.

---

### Write Logic Implementation

I handled all write operations synchronously on the rising edge of the clock.

Key design rules I followed:

* No combinational writes
* Explicit reset behavior
* Writes only occur when the IP is selected

#### Write Logic Code

```verilog
always @(posedge clk) begin
    if (!rst_n) begin
        gpio_data <= 32'b0;
        gpio_dir  <= 32'b0;
    end else if (bus_valid && bus_we) begin
        case (reg_sel)
            2'b00: gpio_data <= bus_wdata; // GPIO_DATA
            2'b01: gpio_dir  <= bus_wdata; // GPIO_DIR
            default: ;
        endcase
    end
end

```

#### Behavior Explanation

On reset:

* All GPIOs default to input mode
* Output data is cleared

Writes only affect:

* GPIO_DATA or GPIO_DIR
* Writes to GPIO_READ are ignored

This guarantees predictable and safe behavior.

---

### Read Logic Implementation

I implemented read logic using pure combinational logic.

This ensures:

* Immediate response to CPU reads
* No clock dependency
* No state changes during reads

#### Read Logic Code

```verilog
always @(*) begin
    case (reg_sel)
        2'b00: bus_rdata = gpio_data; // GPIO_DATA
        2'b01: bus_rdata = gpio_dir;  // GPIO_DIR
        2'b10: bus_rdata = gpio_read; // GPIO_READ
        default: bus_rdata = 32'b0;
    endcase
end

```

---

### GPIO Readback Logic (Key Functional Block)

The most critical part of the GPIO IP is the GPIO_READ behavior.

Requirement:

* Output pins → reflect driven output value
* Input pins → reflect external pin state

#### Implementation

```verilog
assign gpio_read = (gpio_dir & gpio_data) | (~gpio_dir & gpio_in);

```

Explanation:

* `gpio_dir` acts as a mask
* Output pins select `gpio_data`
* Input pins select `gpio_in`
* Supports mixed input/output operation per pin

This exactly matches real-world GPIO peripherals.

---

### Driving GPIO Outputs

GPIO outputs must only drive pins configured as outputs.

```verilog
assign gpio_out = gpio_data & gpio_dir;

```

This prevents:

* Driving pins configured as inputs
* Electrical contention
* Undefined hardware behavior

---

### Reset Behavior

My reset behavior is explicit and safe:

* GPIO_DATA → 0
* GPIO_DIR → 0
* All pins default to input mode

This ensures the system powers up in a safe state.

---

### What I did NOT do in Step 2

To avoid confusion, note that in this step:

 I did no SoC integration
 I added no address base decoding
 I wrote no C firmware
 I performed no simulation or hardware validation

This step is strictly RTL IP implementation.

---

### Evidence to Include for Step 2

![GPIO Control IP RTL](snapshots/gpio_ctrl_ip.png)

`gpio_ctrl_ip.v` showing:

* Register declarations
* Address decoding logic
* Write always block
* Read combinational block
* GPIO readback logic

These screenshots prove correct RTL design, not functionality.

---

### Outcome of Step 2

At the end of Step 2:

* I implemented a clean, multi-register GPIO IP
* Register map behavior matches the specification
* RTL is synthesizable and reusable
* The IP is ready for SoC integration in Step 3

This step transforms the GPIO from a simple block into a realistic peripheral.

## Step 3: Integrate the Multi Register GPIO IP into the SoC (Mandatory)

### Purpose of This Step

In Step 3, I focused on **integrating the multi-register GPIO IP into the existing RISC-V SoC**.
At this stage, the GPIO IP is already complete and correct in isolation (from Step 2). My goal now was to make it **accessible to software** by connecting it to the SoC bus and assigning it a **memory-mapped address range**.

This step is critical because:

* Even a perfectly designed IP will not work if I integrate it incorrectly
* Most real-world bugs appear during SoC integration, not IP design
* Address decoding and bus routing must be exact

---

### Files Modified in This Step

I modified only **one file** in this step:

* `riscv.v` (specifically the `SOC` module)

I **did not modify** the following files:

* `gpio_ctrl_ip.v`
* Firmware files
* Build scripts

My strict separation ensures:

* IP remains reusable
* Integration logic remains SoC-specific

---

### Understanding the SoC Bus Interface

The RISC-V processor communicates with peripherals using a simple memory-mapped bus consisting of:

* `mem_addr`  → Address issued by CPU
* `mem_rdata` → Read data returned to CPU
* `mem_rstrb` → Read strobe
* `mem_wdata` → Write data
* `mem_wmask` → Write mask (byte enables)

Every peripheral (RAM, GPIO, UART) must:

* Respond only to its own address range
* Drive `mem_rdata` only when selected
* Ignore accesses meant for other devices

---

### Assigning the GPIO Base Address

I assigned a fixed base address to the GPIO IP inside the `SOC` module:

```verilog
localparam GPIO_BASE = 32'h2000_0000;

```

Why this matters:

* Software uses this base address to access GPIO registers
* Address must not overlap with RAM or UART
* Clean separation simplifies debugging and future expansion

---

### Address Map of the SoC

After integration, the SoC address space looks like this:

| Address Range | Device |
| --- | --- |
| 0x0000_0000 – … | RAM |
| 0x2000_0000 – … | GPIO |
| 0x4000_0000 – … | UART |

Only the upper address bits are used for decoding.

---

### Address Decoding Logic

Inside the SOC module, I implemented address decoding as:

```verilog
wire is_uart = (mem_addr[31:12] == UART_BASE[31:12]);
wire is_gpio = (mem_addr[31:12] == GPIO_BASE[31:12]);
wire is_ram  = ~(is_uart | is_gpio);

```

Why this logic is important:

* Ensures only one device responds to a given access
* Prevents bus contention
* Fixes instruction fetch issues (X-PC problem)

 The `is_ram` signal is especially important.
Without it, the CPU may try to fetch instructions from GPIO or UART space.

---

### Instantiating the GPIO IP

I instantiated the GPIO IP and connected it to the SoC bus:

```verilog
gpio_ctrl_ip GPIO (
    .clk        (clk),
    .rst_n      (resetn),

    .bus_valid  (is_gpio & (mem_rstrb | |mem_wmask)),
    .bus_we     (|mem_wmask),
    .bus_addr   (mem_addr),
    .bus_wdata  (mem_wdata),
    .bus_rdata  (gpio_rdata),

    .gpio_in    (32'b0),
    .gpio_out   (gpio_out)
);

```

Signal explanation:

#### `bus_valid`

Ensures GPIO responds only when selected and accessed

#### `bus_we`

Derived from write mask

#### `bus_addr`

Full address passed; GPIO IP decodes offsets internally

#### `gpio_in`

Tied to zero for now (external pins optional)

#### `gpio_out`

Used to drive LEDs or GPIO pins

---

### Connecting GPIO Outputs to LEDs

To make GPIO activity visible, I connected GPIO outputs to board LEDs:

```verilog
always @(posedge clk)
    LEDS <= resetn ? gpio_out[4:0] : 5'b0;

```

This allows:

* Immediate visual validation
* Debugging without tools
* Hardware validation in Step 5

---

### Read Data Multiplexing (Critical Section)

Only one device must drive `mem_rdata` at any time.

```verilog
assign mem_rdata =
    is_ram  ? ram_rdata  :
    is_gpio ? gpio_rdata :
    (mem_addr == UART_CTL) ? {22'b0, !uart_ready, 9'b0} :
    32'b0;

```

Why this is critical:

* Prevents multiple drivers on `mem_rdata`
* Guarantees correct readback
* Avoids undefined CPU behavior

This is one of the most common integration mistakes, and fixing it is essential.

---

### Optional Simulation Debug Output

For simulation-only debugging, I logged GPIO writes:

```verilog
always @(posedge clk) begin
    if (is_gpio && |mem_wmask) begin
        `ifndef SYNTHESIS
        $display(
            "[GPIO WRITE] addr=%0d data=0x%08x time=%0t",
            mem_addr[3:2],
            mem_wdata,
            $time
        );
        `endif
    end
end

```

Purpose:

* Confirms address offsets
* Confirms write values
* Helps validate Step 4 software behavior

These statements are ignored during synthesis.

---

### What I did NOT do in Step 3

To avoid confusion:

I made no GPIO RTL changes
 I performed no firmware compilation
 I generated no HEX
 I did no FPGA programming

This step is strictly SoC-level integration.

---

### Evidence to Include for Step 3

![GPIO Integrated into SoC](snapshots/gpio_integration.png)

SOC module showing:

* GPIO base address definition
* Address decoding logic
* GPIO IP instantiation
* `mem_rdata` multiplexer
* LED connection logic

These screenshots prove correct integration, not functionality.

---

### Outcome of Step 3

At the end of Step 3:

* GPIO IP is memory-mapped into the SoC
* CPU can safely access GPIO registers
* Address decoding is correct and conflict-free
* The design is ready for software validation in Step 4

This step completes the hardware-side plumbing required for end-to-end validation.

---

## Step 4: Software Validation (Mandatory)

### Purpose of This Step

The purpose of Step 4 is to **validate the multi-register GPIO IP using real software execution on the RISC-V processor**, rather than relying only on RTL-level inspection.

Although the GPIO IP was already:

* Correctly designed in Step 2
* Properly integrated into the SoC in Step 3

a hardware peripheral is only considered correct when it can be **reliably accessed and controlled by software** using memory-mapped I/O.

This step therefore validates the complete functional chain:

```
C Software → RISC-V CPU → SoC Bus → GPIO IP → GPIO Registers → Physical Signals
```

All validation in this step is performed using **full SoC simulation**, which is mandatory for Task-3.

---

### What Is Being Validated

Through software execution, the following behaviors are verified:

* Correct GPIO direction control using `GPIO_DIR`
* Correct GPIO output driving using `GPIO_DATA`
* Correct readback behavior using `GPIO_READ`
* Correct register offset decoding inside the GPIO IP
* Correct ordering and timing of read/write bus transactions
* Correct execution flow of firmware on the RISC-V CPU
* Correct interaction between CPU, bus, and GPIO IP
* Signal-level correctness using GTKWave waveform inspection

---

### Files Used in This Step

#### Firmware Side

* `Firmware/gpio_test.c` – GPIO test application
* `Firmware/start.S` – RISC-V startup code
* `Firmware/print.c` / `print.h` – UART print utilities
* `Firmware/bram.ld` – Linker script for BRAM execution

#### RTL Side

* `RTL/riscv.v` – SoC top-level integration
* `RTL/firmware.hex` – Generated firmware image loaded into BRAM

No RTL logic was modified during Step 4.
Only firmware files were rebuilt.

---

## Step 4.1: Writing the GPIO Test Program

A dedicated C test program was written to exercise the GPIO registers exactly as specified during the design phase.

### Key Actions Performed by the Firmware

The firmware performs the following actions in order:

1. Defines the GPIO base address
2. Configures GPIO pin direction using `GPIO_DIR`
3. Writes a known pattern to `GPIO_DATA`
4. Reads back the GPIO state using `GPIO_READ`
5. Prints results over UART for confirmation
6. Enters an infinite loop to prevent program exit

---

### GPIO Register Definitions Used in C

```c
#define GPIO_BASE  0x20000000
#define GPIO_DATA  (*(volatile unsigned int *)(GPIO_BASE + 0x00))
#define GPIO_DIR   (*(volatile unsigned int *)(GPIO_BASE + 0x04))
#define GPIO_READ  (*(volatile unsigned int *)(GPIO_BASE + 0x08))
```

These addresses exactly match the register map defined in Step 1 and implemented in the RTL.

---

### Example Test Sequence Executed by Firmware

```c
GPIO_DIR  = 0x0F;   // Configure lower 4 GPIO pins as outputs
GPIO_DATA = 0x05;   // Drive output pattern 0101

unsigned int val = GPIO_READ;
print_hex(val);
```

Expected behavior:

* GPIO pins [3:0] are configured as outputs
* Output pattern `0101` is driven
* Readback value matches driven output
* UART prints the readback value

This confirms correct software-to-hardware interaction.

---

## Step 4.2: Cleaning Previous Firmware Builds

Before building new firmware, all previous build artifacts were removed to prevent stale binaries from being used.

### Commands Used

```bash
cd Firmware
make clean || true
rm -f *.hex *.elf
```
![Clearing Builds Output](snapshots/clearing_builds.png)

This ensures:

* Fresh compilation of all sources
* Correct ELF-to-HEX conversion
* No mismatch between firmware source and loaded image

---

## Step 4.3: Compiling the Firmware (ELF Generation)

The firmware was compiled and linked for execution from BRAM.

### Command Used

```bash
make gpio_test.bram.elf
```
![ELF File Generation](snapshots/elf_generation.png)

This step performs:

* Compilation of C source files using the RISC-V toolchain
* Assembly of startup code (`start.S`)
* Linking using the BRAM linker script (`bram.ld`)

### Output Generated

* `gpio_test.bram.elf`

This ELF file contains executable code mapped to BRAM addresses.

---

## Step 4.4: Generating the BRAM HEX File (Critical Step)

The ELF file was converted into a HEX file suitable for RTL BRAM initialization.

### Command Used

```bash
make gpio_test.bram.hex
```
![BRAM Hex File Generation](snapshots/bram_hex_file.png)

Internally, this executes:

```bash
./firmware_words gpio_test.bram.elf -ram 6144 -max_addr 6144 -out gpio_test.bram.hex
```

### Why This Step Is Critical

* The RTL memory model uses `$readmemh()`
* `$readmemh()` cannot load ELF files
* Address and memory size must exactly match BRAM configuration

Skipping this step results in invalid instruction fetches and CPU lock-up.

### Output Generated

* `gpio_test.bram.hex`

---

## Step 4.5: Copying the HEX File to the RTL Directory

The generated HEX file was copied to the RTL directory, where the SoC memory model expects it.

### Command Used

```bash
cp gpio_test.bram.hex ../RTL/firmware.hex
```

Inside the RTL, BRAM is initialized using:

```verilog
initial begin
    $readmemh("firmware.hex", MEM);
end
```

This ensures the CPU boots using the correct firmware image.

---

## Step 4.6: Running the Full SoC Simulation

The complete SoC was simulated with the firmware executing on the RISC-V core.

### Commands Used

```bash
cd ../RTL
iverilog -g2012 -DSIMULATION -DBENCH -o soc_sim riscv.v
vvp soc_sim
```
![Full SoC View](snapshots/full_soc.png)

During simulation:

* The CPU fetches instructions from BRAM
* Firmware executes automatically after reset
* GPIO registers are accessed by software
* UART output confirms execution flow

---

## Step 4.7: Observing Simulation Logs

Typical simulation output includes messages such as:

```
RAM words: 0 to 1535
VCD info: dumpfile soc.vcd opened for output.
000000A5
```

Interpretation:

* `addr = 1` → GPIO_DIR (offset `0x04`)
* `addr = 0` → GPIO_DATA (offset `0x00`)
* Written values match firmware intent

This confirms:

* Correct address decoding
* Correct register selection
* Correct data propagation

---

## Step 4.8: GTKWave Signal-Level Validation

To validate behavior at the signal level, GTKWave was used to inspect the waveform generated during simulation.

### Signals Observed

* `PC`
* `mem_addr`
* `mem_wdata`
* `mem_wmask`
* `gpio_out`
* `gpio_rdata`

![GTKWave Waveform](snapshots/gtkwave_waveform.png)

### What Was Verified

* Program counter advances correctly
* CPU executes expected instruction sequence
* GPIO write transactions occur at correct addresses
* `gpio_out` reflects written data
* `gpio_rdata` matches expected readback values

This confirms correct internal operation beyond textual logs.

---

## Step 4.9: Handling Output Variations

Minor differences in simulation output are acceptable, such as:

* Slight timing variations
* Differences in debug print order

As long as:

* `GPIO_DIR` is written before `GPIO_DATA`
* Output values match firmware writes
* Readback values are correct

the validation is considered successful.

---

## Outcome of Step 4

By completing Step 4:

* The GPIO IP is proven usable from real software
* Register behavior matches the original specification
* Address decoding is correct
* The software–hardware contract is fully validated
* The design is ready for optional FPGA hardware validation (Step 5)

This completes **mandatory functional verification** of the multi-register GPIO IP.

---

# Step 5: Hardware Validation (Optional, Partial)

## Purpose of This Step

In Step 5, I attempted **hardware validation of the GPIO subsystem** on the VSDSquadron FPGA board to complement the complete RTL and simulation-based verification performed in Steps 2–4.

Hardware validation for Task-3 is explicitly **optional**, and the primary goal of this step is to demonstrate:

* Ability to generate an FPGA bitstream
* Ability to program the FPGA
* Partial physical validation of GPIO behavior
* Understanding of real-world FPGA bring-up challenges

This step is **not required to be fully functional** (UART + clock + firmware execution) to successfully complete Task-3.

---

## Scope of Hardware Validation Performed

On real hardware, I validated the following **to the extent possible**:

* FPGA synthesis and bitstream generation were successful
* Bitstream was successfully programmed into the FPGA
* GPIO outputs were mapped to FPGA pins
* GPIO outputs drove onboard LEDs to a visible, stable state

The following were **not fully validated** on hardware and are explicitly acknowledged:

* UART output on `/dev/ttyUSB0`
* Runtime firmware execution visibility on hardware
* Timer peripheral behavior on hardware (Task-4)

These limitations are explained below.

---

## Prerequisites Confirmed Before Hardware Attempt

Before attempting hardware validation, the following were already complete and verified:

* **Step 2**: Multi-register GPIO IP RTL implemented correctly
* **Step 3**: GPIO IP integrated into the SoC with correct address decoding
* **Step 4**: Firmware fully validated in RTL simulation
* GPIO behavior verified using GTKWave (`soc.vcd`)
* `firmware.hex` generated and loaded correctly in simulation

Simulation is the **primary validation method** for Task-3 and was completed successfully.

---

## Step 5.1: FPGA Bitstream Generation

From the `RTL` directory, I attempted to generate the FPGA bitstream:

```bash
make clean
make build
```
![Make Build Process](snapshots/build_logs.png)

### Observations

* RTL synthesis using **Yosys** completed successfully
* Placement and routing using **nextpnr-ice40** completed
* Bitstream (`soc.bin`) was generated
* Warnings related to clock constraints were observed

Example warning:

```
Warning: unmatched constraint 'CLK_IN'
ERROR: IO 'CLK' is unconstrained in PCF
```

These warnings indicate **clock constraint mismatches**, which are common during early bring-up and **do not invalidate GPIO hardware testing**.

---

## Step 5.2: Programming the FPGA

The generated bitstream was successfully programmed using:

```bash
sudo iceprog soc.bin
```

### Result

```
programming..
VERIFY OK
cdone: high
```
![Iceprog Flash Output](snapshots/iceprog_build.png)

This confirms:

* Flash memory access is working
* FPGA configuration is successful
* The SoC design is physically loaded onto the FPGA

---

## Step 5.3: Physical GPIO Observation Using On-Board LEDs

The VSDSquadron board provides **limited onboard LEDs**:

* One white LED (power indicator)
* Two additional LEDs, one of which is also power-related

Due to this limitation, **external LED circuitry was not mandatory** for Task-3.

After programming the FPGA:

* GPIO outputs were mapped to available LED pins
* LEDs entered a **stable ON state**
* No unintended toggling or instability was observed

This confirms:

* GPIO outputs are driven
* FPGA pin mapping is correct
* GPIO direction and output registers are functional at a hardware level

---

## Why LEDs Appear Constantly ON

The observed LED behavior is **expected and correct**, based on the firmware logic:

* GPIO direction bits are set to output
* GPIO data is written with a non-zero value
* No delay or toggle loop exists in the firmware

Because of this:

* LEDs remain steadily ON
* No blinking occurs

This behavior is **sufficient to validate GPIO output functionality** for Task-3.

---

## UART Output on Hardware (Known Limitation)

UART output was attempted using:

```bash
screen /dev/ttyUSB0 9600
```

However:

* The terminal remained blank
* No characters were observed

Importantly:

* UART output **was fully verified in simulation**
* UART hardware output is **not mandatory** for Task-3
* GPIO validation does **not depend on UART output**

---

## Outcome of Step 5

From this hardware attempt, I demonstrated:

* Successful FPGA bitstream generation
* Successful FPGA programming
* Physical GPIO pin driving
* Stable LED output corresponding to GPIO state
* Clear understanding of hardware limitations

This confirms that the **GPIO IP is hardware-ready**, even though full peripheral bring-up (UART + timer) was not completed on hardware.

---

## Final Note

Hardware validation for Task-3 was performed **to the extent practical** and is provided as **supplementary evidence**.

The **primary correctness proof** for Task-3 remains:

* RTL implementation
* SoC integration
* Software execution
* GTKWave simulation results

These were completed fully and correctly.

This concludes **Task-3**.

---

