# Task-3: Design a Multi-Register GPIO IP with Software Control

## Overview

Task-3 focuses on designing and validating a **realistic, software-controlled GPIO peripheral** similar to what is found in production System-on-Chip (SoC) designs . This task builds directly on Task-2 by extending a simple GPIO IP into a **multi-register, memory-mapped IP** with proper direction control, readback capability, and clean software–hardware interaction .

The GPIO Control IP is accessed by a RISC-V processor through memory-mapped registers and allows software to:
- Configure GPIO pin direction (input or output)
- Write output values to GPIO pins
- Read back the current GPIO pin state

This task emphasizes:
- Register-level thinking
- Address offset decoding
- Clean and synthesizable RTL design
- End-to-end validation from C software to hardware signals

FPGA hardware validation is **optional but encouraged** for participants who have the VSDSquadron FPGA board .

***

## Repository Structure

The following files and directories are required to be committed to the GitHub repository for Task-3 . This structure reflects exactly what was used to achieve the final working output .

```
Task-3/
├── RTL/
│   ├── riscv.v                 # Top-level SoC integration
│   ├── gpio_ctrl_ip.v          # Multi-register GPIO IP (DATA, DIR, READ)
│   ├── clockworks.v            # Clock and reset generation
│   ├── emitter_uart.v          # UART transmitter IP
│   ├── firmware.hex            # Generated firmware image for simulation & hardware
│   ├── VSDSquadronFM.pcf       # FPGA pin constraint file
│   └── Makefile                # Build, flash, and simulation targets
│
├── Firmware/
│   ├── gpio_test.c             # C program to validate GPIO behavior
│   ├── print.c / print.h       # UART print utilities
│   ├── start.S                 # RISC-V startup code
│   ├── bram.ld                 # Linker script for BRAM execution
│   ├── firmware_words          # ELF → HEX conversion utility
│   └── Makefile                # Firmware build and HEX generation
│
└── README.md                   # Task-3 documentation
```

> **Important:**  
> The `firmware.hex` file must be committed, as it is required for both simulation and FPGA hardware execution .

***

## Task Breakdown and Navigation

Use the links below to navigate through each step of Task-3 :

- [**Step 1: Study and Plan (Mandatory)**](#step-1-study-and-plan-mandatory)
- [**Step 2: Implement Multi-Register GPIO RTL (Mandatory)**](#step-2-implement-multi-register-gpio-rtl-mandatory)
- [**Step 3: Integrate the Multi Register GPIO IP into the SoC (Mandatory)**](#step-3-integrate-the-multi-register-gpio-ip-into-the-soc-mandatory)
- [**Step 4: Software Validation (Mandatory)**](#step-4-software-validation-mandatory)
- [**Step 5: Hardware Validation (Optional)**](#step-5-hardware-validation-optional)

***

This overview defines the scope, structure, and navigation of Task-3 .  
Each step below builds progressively, starting from design planning and ending with optional real-hardware validation .


## Step 1: Study and Plan (Mandatory)

### Purpose of This Step

Step 1 is entirely focused on **analysis and design planning** .  
No RTL code, C code, or integration work is performed at this stage .

The objective is to **fully understand what needs to be built and why**, before writing a single line of code . This mirrors real-world hardware development workflows, where a poorly planned IP almost always leads to bugs, rework, and unclear software behavior later .

This step establishes the **software–hardware contract** that all subsequent steps depend on .

***

### Reviewing the Task-2 GPIO IP

The starting point for Task-3 is the GPIO IP implemented in Task-2 .  
Before extending it, the existing design must be reviewed carefully .

From Task-2, the GPIO IP typically had:
- A single register
- Basic write functionality
- Minimal or no direction control
- Simple address decoding

During this review, the following questions were answered:
- How does the CPU communicate with the GPIO IP?
- How are writes detected (bus signals, write mask)?
- How is data stored and driven to outputs?
- How does the IP return read data?

This review helped identify:
- Which logic can be reused
- Which logic must be redesigned
- Where additional structure is required

***

### Understanding the New Requirements for Task-3

Task-3 requires upgrading the simple GPIO IP into a **multi-register peripheral**, similar to what is used in real SoCs .

The new GPIO IP must:
- Support **multiple memory-mapped registers**
- Allow **per-pin direction control**
- Provide **readback of GPIO pin state**
- Behave predictably for both software and hardware

This means the IP can no longer behave like a single latch . It must behave like a **register block** .

***

### Defining the Register Map

A fixed register map is defined for the GPIO IP . This register map is the foundation of the entire design .

| Offset | Register Name | Description |
|------:|---------------|-------------|
| 0x00  | GPIO_DATA     | Stores output values written by software |
| 0x04  | GPIO_DIR      | Controls direction (1 = output, 0 = input) |
| 0x08  | GPIO_READ     | Returns current GPIO pin state |

Important design decisions made here:
- All registers are **32-bit wide**
- Registers are **word-aligned**
- GPIO_READ is **read-only**
- GPIO_DATA and GPIO_DIR are **read/write**

Once defined, this register map must not change, as software depends on it .

***

### Defining Register Behavior (Critical Planning)

Each register's behavior was defined precisely before implementation .

#### GPIO_DATA (Offset 0x00)
- Writing updates the stored output value
- Reading returns the **last written value**
- Does not depend on GPIO direction

#### GPIO_DIR (Offset 0x04)
- Each bit controls one GPIO pin
- `1` → output enabled
- `0` → input mode
- Affects both output driving and readback behavior

#### GPIO_READ (Offset 0x08)
- Read-only register
- Returns:
  - Output value for pins configured as outputs
  - External pin state for pins configured as inputs

Defining this behavior early prevents ambiguity and inconsistent implementations .

***

### Address Offset Decoding Strategy

The GPIO IP must determine **which register is being accessed** using the address bus .

Planning decisions:
- Base address decoding will be handled at the SoC level
- Inside the GPIO IP, only **offset decoding** is required
- Lower address bits (e.g. `bus_addr[3:2]`) will be used to select registers

Planned mapping:
- `00` → GPIO_DATA
- `01` → GPIO_DIR
- `10` → GPIO_READ

No decoding logic is written yet, but the mapping is clearly defined .

***

### Internal Signal Planning

Before coding, internal signals and registers were planned to keep the RTL clean and understandable .

Planned internal elements:
- `gpio_data` register for output values
- `gpio_dir` register for direction control
- `gpio_out` signal for driven outputs
- `gpio_in` signal for external inputs
- `gpio_read` signal for readback logic

Each signal has:
- A single responsibility
- Clear ownership (register vs combinational logic)

This prevents latch inference and unintended behavior later .

***

### Reset Behavior Planning

Reset behavior was defined at this stage to ensure safe defaults .

On reset:
- GPIO_DATA → `0`
- GPIO_DIR → `0` (all pins input)
- GPIO outputs → inactive

This ensures:
- No pins are driven unintentionally at reset
- Software must explicitly configure GPIOs before use

***

### Software Perspective Considerations

This step also required thinking from the **software point of view** .

Key questions considered:
- What happens if software writes GPIO_DATA before GPIO_DIR?
- What value does GPIO_READ return immediately after reset?
- Are register reads deterministic?

By answering these questions early, the GPIO IP becomes:
- Easier to use from software
- Easier to debug
- Closer to real industry peripherals

***

### Outcome of Step 1

At the end of Step 1:
- No RTL code was written
- The full GPIO register map was defined
- Register behavior was clearly specified
- Address decoding strategy was planned
- Internal signals were identified
- Reset behavior was decided

This step establishes a **solid design foundation** .  
Only after completing this planning step does it make sense to move to RTL implementation in Step 2 .

## Step 2: Implement Multi-Register GPIO RTL (Mandatory)

### Purpose of This Step

Step 2 is where the **actual RTL implementation** of the GPIO Control IP begins .  
Using the design decisions made in Step 1, the goal here is to convert the planned register map and behavior into **clean, synthesizable Verilog RTL** .

This step focuses only on the **GPIO IP itself** .  
No SoC-level integration, no firmware, and no hardware programming are done here .

***

### File Modified in This Step

Only one file is modified in this step:

- `gpio_ctrl_ip.v`

This separation is intentional and important:
- The GPIO IP should remain **self-contained**
- It should not depend on SoC internals
- It should be reusable in other SoCs

***

### Register Implementation Strategy

Based on Step 1 planning, the GPIO IP must implement three registers :

| Offset | Register | Type |
|------:|----------|------|
| 0x00  | GPIO_DATA | Read / Write |
| 0x04  | GPIO_DIR  | Read / Write |
| 0x08  | GPIO_READ | Read Only |

Each register is implemented either as:
- A **storage register** (`reg`)
- Or a **derived combinational signal**

***

### Internal Registers and Signals

The following internal elements are declared inside `gpio_ctrl_ip.v` :

#### Registers
- `gpio_data`  
  Stores the value written by software to `GPIO_DATA`
- `gpio_dir`  
  Stores direction bits written to `GPIO_DIR`

#### Wires
- `gpio_read`  
  Represents the computed GPIO readback value
- `reg_sel`  
  Used for address offset decoding

This clear separation ensures:
- No unintended latches
- Predictable synthesis results
- Easy debugging

***

### Address Offset Decoding

To support multiple registers, address offset decoding is implemented using lower address bits :

```verilog
wire [1:0] reg_sel = bus_addr[3:2];
```

Why this works:
- Registers are 32-bit aligned
- Address offsets increment by 4 bytes
- `bus_addr[3:2]` uniquely selects each register

Mapping:
- `2'b00` → GPIO_DATA (0x00)
- `2'b01` → GPIO_DIR (0x04)
- `2'b10` → GPIO_READ (0x08)

This decoding logic is simple, synthesizable, and commonly used in real designs .

***

### Write Logic Implementation

All write operations are handled synchronously on the rising edge of the clock .

Key design rules followed:
- No combinational writes
- Explicit reset behavior
- Writes only occur when the IP is selected

#### Write Logic Code

```verilog
always @(posedge clk) begin
    if (!rst_n) begin
        gpio_data <= 32'b0;
        gpio_dir  <= 32'b0;
    end else if (bus_valid && bus_we) begin
        case (reg_sel)
            2'b00: gpio_data <= bus_wdata; // GPIO_DATA
            2'b01: gpio_dir  <= bus_wdata; // GPIO_DIR
            default: ;
        endcase
    end
end
```

#### Behavior Explanation

On reset:
- All GPIOs default to input mode
- Output data is cleared

Writes only affect:
- GPIO_DATA or GPIO_DIR
- Writes to GPIO_READ are ignored

This guarantees predictable and safe behavior .

***

### Read Logic Implementation

Read logic is implemented using pure combinational logic .

This ensures:
- Immediate response to CPU reads
- No clock dependency
- No state changes during reads

#### Read Logic Code

```verilog
always @(*) begin
    case (reg_sel)
        2'b00: bus_rdata = gpio_data; // GPIO_DATA
        2'b01: bus_rdata = gpio_dir;  // GPIO_DIR
        2'b10: bus_rdata = gpio_read; // GPIO_READ
        default: bus_rdata = 32'b0;
    endcase
end
```

***

### GPIO Readback Logic (Key Functional Block)

The most critical part of the GPIO IP is the GPIO_READ behavior .

Requirement:
- Output pins → reflect driven output value
- Input pins → reflect external pin state

#### Implementation

```verilog
assign gpio_read = (gpio_dir & gpio_data) | (~gpio_dir & gpio_in);
```

Explanation:
- `gpio_dir` acts as a mask
- Output pins select `gpio_data`
- Input pins select `gpio_in`
- Supports mixed input/output operation per pin

This exactly matches real-world GPIO peripherals .

***

### Driving GPIO Outputs

GPIO outputs must only drive pins configured as outputs .

```verilog
assign gpio_out = gpio_data & gpio_dir;
```

This prevents:
- Driving pins configured as inputs
- Electrical contention
- Undefined hardware behavior

***

### Reset Behavior

Reset behavior is explicit and safe :

- GPIO_DATA → 0
- GPIO_DIR → 0
- All pins default to input mode

This ensures the system powers up in a safe state .

***

### What Was NOT Done in Step 2

To avoid confusion, note that in this step :

❌ No SoC integration was done  
❌ No address base decoding was added  
❌ No C firmware was written  
❌ No simulation or hardware validation was performed

This step is strictly RTL IP implementation .

***

### Evidence to Include for Step 2

![GPIO Control IP RTL](snapshots/gpio_ctrl_ip.png)

`gpio_ctrl_ip.v` showing:
- Register declarations
- Address decoding logic
- Write always block
- Read combinational block
- GPIO readback logic

These screenshots prove correct RTL design, not functionality .

***

### Outcome of Step 2

At the end of Step 2:
- A clean, multi-register GPIO IP is implemented
- Register map behavior matches the specification
- RTL is synthesizable and reusable
- The IP is ready for SoC integration in Step 3

This step transforms the GPIO from a simple block into a realistic peripheral .

## Step 3: Integrate the Multi Register GPIO IP into the SoC (Mandatory)

### Purpose of This Step

Step 3 focuses on **integrating the multi-register GPIO IP into the existing RISC-V SoC** .  
At this stage, the GPIO IP is already complete and correct in isolation (from Step 2) . The goal now is to make it **accessible to software** by connecting it to the SoC bus and assigning it a **memory-mapped address range** .

This step is critical because:
- Even a perfectly designed IP will not work if it is integrated incorrectly
- Most real-world bugs appear during SoC integration, not IP design
- Address decoding and bus routing must be exact

***

### Files Modified in This Step

Only **one file** is modified in this step :

- `riscv.v` (specifically the `SOC` module)

The following files are **not modified** :
- `gpio_ctrl_ip.v`
- Firmware files
- Build scripts

This strict separation ensures:
- IP remains reusable
- Integration logic remains SoC-specific

***

### Understanding the SoC Bus Interface

The RISC-V processor communicates with peripherals using a simple memory-mapped bus consisting of :

- `mem_addr`  → Address issued by CPU
- `mem_rdata` → Read data returned to CPU
- `mem_rstrb` → Read strobe
- `mem_wdata` → Write data
- `mem_wmask` → Write mask (byte enables)

Every peripheral (RAM, GPIO, UART) must:
- Respond only to its own address range
- Drive `mem_rdata` only when selected
- Ignore accesses meant for other devices

***

### Assigning the GPIO Base Address

A fixed base address is assigned to the GPIO IP inside the `SOC` module :

```verilog
localparam GPIO_BASE = 32'h2000_0000;
```

Why this matters:
- Software uses this base address to access GPIO registers
- Address must not overlap with RAM or UART
- Clean separation simplifies debugging and future expansion

***

### Address Map of the SoC

After integration, the SoC address space looks like this :

| Address Range | Device |
|---------------|--------|
| 0x0000_0000 – … | RAM |
| 0x2000_0000 – … | GPIO |
| 0x4000_0000 – … | UART |

Only the upper address bits are used for decoding .

***

### Address Decoding Logic

Inside the SOC module, address decoding is implemented as :

```verilog
wire is_uart = (mem_addr[31:12] == UART_BASE[31:12]);
wire is_gpio = (mem_addr[31:12] == GPIO_BASE[31:12]);
wire is_ram  = ~(is_uart | is_gpio);
```

Why this logic is important:
- Ensures only one device responds to a given access
- Prevents bus contention
- Fixes instruction fetch issues (X-PC problem)

⚠️ The `is_ram` signal is especially important .  
Without it, the CPU may try to fetch instructions from GPIO or UART space .

***

### Instantiating the GPIO IP

The GPIO IP is instantiated and connected to the SoC bus :

```verilog
gpio_ctrl_ip GPIO (
    .clk        (clk),
    .rst_n      (resetn),

    .bus_valid  (is_gpio & (mem_rstrb | |mem_wmask)),
    .bus_we     (|mem_wmask),
    .bus_addr   (mem_addr),
    .bus_wdata  (mem_wdata),
    .bus_rdata  (gpio_rdata),

    .gpio_in    (32'b0),
    .gpio_out   (gpio_out)
);
```

Signal explanation :

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

***

### Connecting GPIO Outputs to LEDs

To make GPIO activity visible, GPIO outputs are connected to board LEDs :

```verilog
always @(posedge clk)
    LEDS <= resetn ? gpio_out[4:0] : 5'b0;
```

This allows:
- Immediate visual validation
- Debugging without tools
- Hardware validation in Step 5

***

### Read Data Multiplexing (Critical Section)

Only one device must drive `mem_rdata` at any time .

```verilog
assign mem_rdata =
    is_ram  ? ram_rdata  :
    is_gpio ? gpio_rdata :
    (mem_addr == UART_CTL) ? {22'b0, !uart_ready, 9'b0} :
    32'b0;
```

Why this is critical:
- Prevents multiple drivers on `mem_rdata`
- Guarantees correct readback
- Avoids undefined CPU behavior

This is one of the most common integration mistakes, and fixing it is essential .

***

### Optional Simulation Debug Output

For simulation-only debugging, GPIO writes are logged :

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
- Confirms address offsets
- Confirms write values
- Helps validate Step 4 software behavior

These statements are ignored during synthesis .

***

### What Was NOT Done in Step 3

To avoid confusion :

❌ No GPIO RTL changes  
❌ No firmware compilation  
❌ No HEX generation  
❌ No FPGA programming

This step is strictly SoC-level integration .

***

### Evidence to Include for Step 3

![GPIO Integration in SoC](snapshots/gpio_integration.png)

SOC module showing:
- GPIO base address definition
- Address decoding logic
- GPIO IP instantiation
- `mem_rdata` multiplexer
- LED connection logic

These screenshots prove correct integration, not functionality .

***

### Outcome of Step 3

At the end of Step 3:
- GPIO IP is memory-mapped into the SoC
- CPU can safely access GPIO registers
- Address decoding is correct and conflict-free
- The design is ready for software validation in Step 4

This step completes the hardware-side plumbing required for end-to-end validation .

## Step 4: Software Validation (Mandatory)

### Purpose of This Step

Step 4 validates the **entire GPIO IP through software**, proving that the hardware behaves correctly when accessed by a real program running on the RISC-V processor .

Up to Step 3:
- The GPIO IP is correctly designed
- It is integrated into the SoC
- Address decoding is correct

However, **none of that matters unless software can use it correctly** .

This step establishes the complete **software → bus → GPIO IP → register → signal** flow and confirms that the register-level contract defined in Step 1 is actually honored .

Simulation-based validation is **mandatory** for this step .

***

## What Is Being Validated

The following behaviors must be proven using software :

1. GPIO direction control works (`GPIO_DIR`)
2. GPIO output values update correctly (`GPIO_DATA`)
3. GPIO readback reflects correct pin state (`GPIO_READ`)
4. Address offsets decode correctly
5. Writes and reads occur at the correct time
6. UART output confirms execution flow

***

## Files Used in This Step

### Firmware Side
- `Firmware/gpio_test.c` – main test program
- `Firmware/start.S` – startup code
- `Firmware/print.c` – UART print support
- `Firmware/bram.ld` – linker script

### RTL Side
- `RTL/riscv.v`
- `RTL/firmware.hex` (generated in this step)

No RTL logic is modified during Step 4 .

***

## Step 4.1: Writing the GPIO Test Program

A C program is written to explicitly exercise all GPIO registers .

### Key Actions Performed by the Firmware

1. Define GPIO base address
2. Write to `GPIO_DIR` to configure pins as outputs
3. Write a known pattern to `GPIO_DATA`
4. Read back `GPIO_READ`
5. Print values over UART for verification

### Example Register Definitions in C

```c
#define GPIO_BASE  0x20000000
#define GPIO_DATA  (*(volatile unsigned int *)(GPIO_BASE + 0x00))
#define GPIO_DIR   (*(volatile unsigned int *)(GPIO_BASE + 0x04))
#define GPIO_READ  (*(volatile unsigned int *)(GPIO_BASE + 0x08))
```

### Example Test Sequence

```c
GPIO_DIR  = 0x0F;      // Set lower 4 GPIOs as outputs
GPIO_DATA = 0x05;      // Drive pattern 0101
unsigned int val = GPIO_READ;
print_hex(val);
```

This sequence directly maps to the register behavior defined in Step 1 .

📸 **Screenshot to include**
- `gpio_test.c` showing GPIO register access and writes

***

## Step 4.2: Cleaning Previous Firmware Builds

Before compiling new firmware, all old artifacts are removed to avoid stale binaries .

Commands used:
```bash
cd Firmware
make clean || true
rm -f *.hex *.elf
```

This ensures:
- No outdated firmware is accidentally used
- The generated HEX matches the current C code

## Step 4.3: Compiling the Firmware (ELF Generation)

The firmware is compiled and linked for execution from BRAM .

Command used:
```bash
make gpio_test.bram.elf
```

This step performs:
- C compilation using `riscv32-unknown-elf-gcc`
- Assembly of startup code
- Linking using `bram.ld`

**Output File**
- `gpio_test.bram.elf`


## Step 4.4: Generating the BRAM HEX File (CRITICAL)

This is the most commonly missed step and is absolutely required .

The ELF file must be converted into a word-addressable HEX file that the RTL memory model can load .

Command used:
```bash
make gpio_test.bram.hex
```

Internally, this runs a tool similar to:
```bash
./firmware_words gpio_test.bram.elf -ram 6144 -max_addr 6144 -out gpio_test.bram.hex
```

### Why This Step Matters

- `$readmemh()` in RTL loads HEX, not ELF
- Memory size must match RTL BRAM
- Without this step, the CPU executes garbage

**Output File**
- `gpio_test.bram.hex`

![Firmware HEX Generation](snapshots/firmware_hex_output.png)

- Terminal output showing HEX generation

***

## Step 4.5: Copying the HEX File to RTL Directory

The generated HEX file must be placed where the RTL expects it .

Commands used:
```bash
cp gpio_test.bram.hex ../RTL/firmware.hex
```

This file is loaded in RTL using :
```verilog
initial begin
    $readmemh("firmware.hex", MEM);
end
```

📸 **Screenshot to include**
- `ls -l firmware.hex` confirming file exists in RTL directory

***

## Step 4.6: Running RTL Simulation

Now the full SoC is simulated with the firmware executing on the RISC-V core .

Commands used:
```bash
cd ../RTL
iverilog -g2012 -DBENCH -o soc_sim riscv.v
./soc_sim
```

During simulation:
- CPU fetches instructions from BRAM
- Firmware runs automatically
- GPIO registers are accessed by software
- RTL debug messages appear

***

## Step 4.7: Observing Simulation Output

Typical simulation output includes :

```
RAM words: 0 to 1535
[GPIO WRITE] addr=1 data=0x0000000f time=14725
[GPIO WRITE] addr=0 data=0x00000005 time=14855
```

Interpretation:
- `addr=1` → GPIO_DIR (offset 0x04)
- `addr=0` → GPIO_DATA (offset 0x00)
- Data values match firmware writes

This confirms:
- Correct offset decoding
- Correct write handling
- Correct register updates

![Simulation Output](snapshots/terminal_output.png)

- Terminal output showing GPIO write logs
- Optional waveform screenshot

***

## Step 4.8: Handling Output Differences

If your simulation output differs slightly :

- Timing values may differ → expected
- Order of prints may differ → acceptable
- Data values must match firmware intent

As long as:
- `GPIO_DIR` is written first
- `GPIO_DATA` follows
- `GPIO_READ` returns correct value

The validation is considered successful .

***

## Outcome of Step 4

By the end of this step:
- GPIO IP is proven usable by software
- Register map behaves exactly as defined
- Address decoding is correct
- Software–hardware contract is validated
- Design is ready for FPGA programming (Step 5)

This step completes functional validation of the GPIO IP .


## Step 5: Hardware Validation (Optional)

### Purpose of This Step

Step 5 validates the **entire GPIO subsystem on real FPGA hardware**, confirming that everything developed and verified in simulation (Steps 1–4) works correctly when deployed on the VSDSquadron FPGA board.

While this step is optional, it is extremely valuable because it demonstrates:
- End-to-end correctness (software → CPU → bus → GPIO IP → physical pins)
- Real-world timing behavior
- Practical FPGA bring-up and debugging skills

This step mirrors how IPs are validated in industry after simulation sign-off.

---

## What Is Being Validated on Hardware

By running this step, we validate:

1. The synthesized SoC fits and routes correctly on the FPGA
2. The RISC-V core boots and executes firmware from BRAM
3. The GPIO IP responds to memory-mapped accesses
4. GPIO direction control works physically
5. GPIO output values drive real LEDs
6. The same firmware used in simulation runs unmodified on hardware

---

## Prerequisites Before Starting Step 5

Before attempting hardware validation, ensure:

- Step 2: GPIO IP RTL is complete and correct
- Step 3: GPIO IP is integrated into the SoC
- Step 4: Firmware runs successfully in simulation
- `firmware.hex` is present in the `RTL/` directory
- `make build` completes without errors
- FPGA board and USB cable are available

---

## Step 5.1: Building the FPGA Bitstream

From the `RTL` directory, generate the FPGA bitstream:

```bash
make clean
make build
````

### What Happens Internally

This command performs the full FPGA flow:

1. **Yosys**

   * Synthesizes Verilog RTL
   * Generates a technology-mapped netlist (`SOC.json`)

2. **nextpnr-ice40**

   * Places and routes the design
   * Applies pin constraints from `.pcf`
   * Optimizes timing

3. **icetime**

   * Performs static timing analysis

4. **icepack**

   * Produces the final bitstream (`SOC.bin`)

### Expected Outcome

* No synthesis or P&R errors
* `SOC.bin` file generated successfully

![FPGA Build Output](snapshots/make_build.png)

* Terminal output showing successful completion of `make build`

---

## Step 5.2: Connecting the FPGA Board

Connect the VSDSquadron FPGA board to the system using a USB cable.

Verify that the board is detected:

```bash
lsusb
```

You should see an FTDI device similar to:

```
Future Technology Devices International, Ltd FT232H
```

Verify FTDI drivers are loaded:

```bash
lsmod | grep ftdi
```

This confirms the system can communicate with the board.

---

## Step 5.3: Flashing the Bitstream to the FPGA

Program the FPGA with the generated bitstream:

```bash
sudo iceprog SOC.bin
```

### Expected Output

Typical successful output looks like:

```
flash ID: 0xEF 0x40 0x16 0x00
programming..
VERIFY OK
cdone: high
```

This confirms:

* Flash memory is detected
* Bitstream is written correctly
* FPGA configuration is successful

![FPGA Flash Output](snapshots/flash_terminal.png)

* Terminal output showing successful `iceprog`

---

## Step 5.4: Firmware Execution on Hardware

After flashing:

* The FPGA automatically comes out of reset
* The RISC-V CPU starts executing the firmware from BRAM
* No additional action is required

The firmware performs the same sequence as in simulation:

1. Writes to `GPIO_DIR`
2. Writes to `GPIO_DATA`
3. Optionally reads `GPIO_READ`

---

## Step 5.5: Observing GPIO Behavior on LEDs

GPIO outputs are connected to physical LEDs on the board.

Expected behavior:

* LEDs corresponding to output-enabled GPIO pins turn ON/OFF
* LED pattern matches the value written in firmware
* Pins configured as inputs do not drive LEDs

This confirms:

* Direction control is working
* Output data is correctly driven
* Software-to-hardware path is functional

---

## Step 5.6: Optional UART Output Validation

If UART output is enabled in the firmware:

Open serial terminal:

```bash
picocom -b 9600 /dev/ttyUSBx
```

If the firmware prints messages, they should appear in the terminal, confirming UART and CPU execution on hardware.

---

## Common Issues and Debug Notes

* **No LEDs ON**

  * Check `GPIO_DIR` is set correctly in firmware
  * Verify LED pin mapping in `.pcf`

* **Board not detected**

  * Try a different USB port
  * Check FTDI drivers
  * Use `sudo` for flashing

* **UART not opening**

  * Confirm correct `/dev/ttyUSBx`
  * Ensure baud rate matches firmware

These are normal bring-up issues and part of real hardware validation.

---

## Outcome of Step 5

By completing Step 5, you have demonstrated:

* A fully working RISC-V SoC on FPGA
* A production-style GPIO IP with direction and data registers
* Successful software control of hardware peripherals
* End-to-end validation beyond simulation

This confirms that the GPIO IP is **hardware-ready** and suitable as a foundation for more advanced peripherals.

---

## Final Note

Although optional, completing this step provides strong evidence of:

* Practical FPGA skills
* Real SoC integration experience
* Industry-relevant validation workflow

This concludes Task-3 successfully.
