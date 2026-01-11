#  Task-4: Real Peripheral IP Development – Timer IP

##  Overview

**Task-4** focuses on designing, integrating, and validating a **real memory-mapped Timer IP** inside a RISC-V based SoC.
Unlike previous tasks, the Timer IP operates **autonomously in hardware** after being configured by software, demonstrating a realistic peripheral design workflow used in industry-grade SoCs.

The timer is programmed by the CPU through memory-mapped registers and generates a periodic timeout signal, which is used to toggle LEDs on the FPGA board.

---

##  Objectives

* Design a **custom Timer IP** with control, load, value, and status registers
* Integrate the Timer IP into the SoC address map
* Control the Timer IP using C firmware running on a RISC-V core
* Use the Timer IP output to drive **real hardware (LEDs)**
* Verify functionality through **simulation and FPGA implementation**

---

##  What This Task Demonstrates

* Memory-mapped peripheral design
* Hardware–software interaction
* Address decoding and bus integration
* Register-based IP control
* Difference between **software-driven** and **hardware-driven** behavior
* End-to-end SoC bring-up (RTL → Firmware → FPGA)

---

## 🗂️ Project Structure

```
basicRISCV/
├── RTL/
│   ├── riscv.v              # SoC top module (Timer integrated)
│   ├── timer_ip.v           # Custom Timer IP (Task-4)
│   ├── gpio_ctrl_ip.v
│   ├── VSDSquadronFM.pcf
│   └── Makefile
│
├── Firmware/
│   ├── timer_test.c         # Firmware to configure Timer IP
│   ├── start.S
│   ├── firmware.hex
│   └── Makefile
│
└── README.md
```

---

## 🔗 Direct Access to Task-4 Sections

* [1. Timer IP Design](#1-timer-ip-design)
* [2. Memory Map & Register Definition](#2-memory-map--register-definition)
* [3. SoC Integration](#3-soc-integration)
* [4. Firmware Control](#4-firmware-control)
* [5. Simulation & Verification](#5-simulation--verification)
* [6. FPGA Hardware Demonstration](#6-fpga-hardware-demonstration)
* [7. Results & Observations](#7-results--observations)

---


Perfect. Below is **Step-1 written cleanly for your README**, in the same professional tone, **plus a clear answer on screenshots**.

You can paste this **directly under the Step-1 heading** in `README.md`.

---

## 1. Timer IP Design

### Description

In this step, a **custom Timer IP** was designed as a **memory-mapped peripheral**.
The Timer IP is responsible for generating a periodic timeout signal after being configured by software running on the RISC-V processor.

The design follows a **register-based architecture**, similar to real SoC peripherals, allowing the CPU to control the timer using standard load/store instructions.

---

### Timer IP Features

The Timer IP provides the following functionality:

* Configurable countdown timer
* Programmable load value
* Enable and mode control (one-shot / periodic)
* Hardware-generated timeout pulse
* Software-visible status register

Once enabled, the timer operates **autonomously in hardware**, without continuous CPU intervention.

---

### Register Map

The Timer IP exposes four 32-bit memory-mapped registers:

| Offset | Register | Description                                                |
| ------ | -------- | ---------------------------------------------------------- |
| `0x00` | CTRL     | Control register (`bit0 = enable`, `bit1 = periodic mode`) |
| `0x04` | LOAD     | Load value for the timer                                   |
| `0x08` | VALUE    | Current countdown value (read-only)                        |
| `0x0C` | STAT     | Status register (`bit0 = timeout`)                         |

---

### Internal Operation

* When the **CTRL register** is written with `enable = 1`, the timer starts counting down.
* The **VALUE register** decrements every clock cycle.
* When the counter reaches zero:

  * A **timeout pulse** is generated
  * In periodic mode, the timer automatically reloads from the LOAD register
* The timeout signal is asserted for **one full clock cycle**, making it safe for hardware use and simulation visibility.

---

### Design Considerations

* The timeout signal is **registered**, not combinational, ensuring reliable detection.
* The timer supports **periodic operation**, required for continuous LED blinking.
* The design is **synchronous and reset-safe**, compatible with FPGA synthesis.
* The implementation avoids free-running hardware blink logic, ensuring compliance with Task-4 requirements.

---

### Files Involved

* `RTL/timer_ip.v` — Timer IP implementation
  
---

### Outcome of Step-1

A fully functional, memory-mapped Timer IP was designed, ready to be integrated into the SoC and controlled by software.

---

## 2. Memory Map & Register Definition

### Description

In this step, the **Timer IP was assigned a dedicated address range** in the SoC memory map, and its internal registers were exposed to the CPU using **fixed offsets**.
This allows the RISC-V processor to configure and control the Timer IP using standard load and store instructions.

Defining a clear memory map ensures **predictable hardware–software interaction** and avoids address conflicts with other peripherals.

---

### Timer Base Address

The Timer IP is mapped at the following base address:

```text
TIMER_BASE = 0x2000_1000
```

This address lies in the peripheral address space and does not overlap with:

* GPIO IP
* UART
* Internal RAM

---

### Register Address Map

All Timer IP registers are accessed relative to the base address:

| Address Offset | Absolute Address | Register | Access | Description                                           |
| -------------- | ---------------- | -------- | ------ | ----------------------------------------------------- |
| `0x00`         | `0x20001000`     | CTRL     | R/W    | Control register (`bit0 = enable`, `bit1 = periodic`) |
| `0x04`         | `0x20001004`     | LOAD     | R/W    | Load value for timer                                  |
| `0x08`         | `0x20001008`     | VALUE    | R      | Current countdown value                               |
| `0x0C`         | `0x2000100C`     | STAT     | R/W    | Status register (`bit0 = timeout`)                    |

---

### Software Access

The Timer IP registers are accessed from C firmware using **volatile memory-mapped pointers**:

```c
#define TIMER_BASE   0x20001000

#define TIMER_CTRL  (*(volatile unsigned int *)(TIMER_BASE + 0x00))
#define TIMER_LOAD  (*(volatile unsigned int *)(TIMER_BASE + 0x04))
#define TIMER_VALUE (*(volatile unsigned int *)(TIMER_BASE + 0x08))
#define TIMER_STAT  (*(volatile unsigned int *)(TIMER_BASE + 0x0C))
```

Using `volatile` ensures that:

* The compiler does not optimize away register accesses
* Every read/write reaches the hardware

---

### Address Decoding in SoC

Inside the SoC, address decoding logic identifies accesses to the Timer IP using the upper address bits:

```verilog
localparam TIMER_BASE = 32'h2000_1000;

wire is_timer = (mem_addr[31:12] == TIMER_BASE[31:12]);
```

When `is_timer` is asserted:

* CPU writes are routed to the Timer IP
* Timer IP read data is returned to the CPU

This mechanism cleanly integrates the Timer IP into the existing bus structure.

---

### Design Considerations

* Register offsets are **word-aligned** for safe 32-bit access
* Timer IP address range is isolated from other peripherals
* Address decoding is minimal and efficient
* The design follows standard SoC memory-mapped peripheral conventions

---

### Files Involved

* `RTL/riscv.v` — Timer address decoding and bus routing
* `Firmware/timer_test.c` — Timer register access from software

---

### Outcome of Step-2

The Timer IP was successfully assigned a unique memory-mapped address range with a well-defined register interface, enabling reliable software control.

---

## 3. SoC Integration

### Description

In this step, the custom Timer IP was **integrated into the RISC-V SoC** by connecting it to the system bus, implementing address decoding, and routing read/write transactions between the CPU and the Timer IP.

This step ensures that the Timer IP behaves like a **real SoC peripheral**, accessible through standard memory operations.

---

### Integration into SoC Architecture

The Timer IP was instantiated inside the SoC top module and connected to:

* System clock and reset
* CPU memory address bus
* CPU write data and write mask
* CPU read data return path

This allows the processor to configure and control the Timer IP using normal load/store instructions.

---

### Address Decode Logic

A dedicated address decode signal is generated for the Timer IP:

```verilog
localparam TIMER_BASE = 32'h2000_1000;

wire is_timer = (mem_addr[31:12] == TIMER_BASE[31:12]);
```

This signal ensures that only accesses within the Timer IP address range are forwarded to the peripheral.

---

### Timer IP Instantiation

The Timer IP is instantiated in the SoC and connected to the bus as shown below:

```verilog
timer_ip TIMER (
    .clk     (clk),
    .resetn  (resetn),
    .sel     (is_timer),
    .we      (is_timer & |mem_wmask),
    .addr    (mem_addr),
    .wdata   (mem_wdata),
    .rdata   (timer_rdata),
    .timeout (timer_timeout)
);
```

* `sel` enables the Timer IP for valid accesses
* `we` enables register writes
* `rdata` returns register values to the CPU
* `timeout` generates the hardware event

---

### Read Data Multiplexing

The Timer IP output is added to the SoC read-data multiplexer:

```verilog
assign mem_rdata =
    is_ram   ? ram_rdata   :
    is_gpio  ? gpio_rdata  :
    is_timer ? timer_rdata :
    32'b0;
```

This allows the CPU to correctly read Timer IP registers such as `VALUE` and `STAT`.

---

### Hardware Event Routing

The `timeout` output from the Timer IP is routed to SoC logic and used to toggle LED outputs:

```verilog
always @(posedge clk or negedge resetn) begin
    if (!resetn)
        led_toggle <= 1'b0;
    else if (timer_timeout)
        led_toggle <= ~led_toggle;
end
```

This demonstrates how a peripheral-generated hardware event can directly influence system behavior.

---

### Design Considerations

* Timer IP integration does not disturb existing peripherals
* Address decode logic prevents unintended bus conflicts
* Read and write paths are clearly separated
* Timeout signal is treated as a synchronous hardware event

---

### Files Involved

* `RTL/riscv.v` — SoC integration and address decoding
* `RTL/timer_ip.v` — Timer peripheral
* `RTL/gpio_ctrl_ip.v` — Existing GPIO IP (unchanged)

---

### Outcome of Step-3

The Timer IP was successfully integrated into the SoC, enabling seamless communication between the RISC-V processor and the hardware timer peripheral.

---

## 4. Firmware Control

### Description

In this step, firmware running on the RISC-V processor was developed to **configure and control the Timer IP** through its memory-mapped registers.
The firmware demonstrates how software can initialize the timer, enable periodic operation, and allow the hardware timer to operate autonomously.

This step establishes the **hardware–software contract** required for a real peripheral IP.

---

### Firmware Objective

The firmware performs the following actions:

1. Programs the timer load value
2. Enables the timer in periodic mode
3. Waits for timeout events
4. Allows the hardware timer to drive LED behavior

Once configured, the CPU does **not** manually toggle LEDs, ensuring that blinking is **hardware-driven**, not software-driven.

---

### Timer Register Access

The Timer IP registers are accessed using volatile pointers:

```c
#define TIMER_BASE   0x20001000

#define TIMER_CTRL  (*(volatile unsigned int *)(TIMER_BASE + 0x00))
#define TIMER_LOAD  (*(volatile unsigned int *)(TIMER_BASE + 0x04))
#define TIMER_VALUE (*(volatile unsigned int *)(TIMER_BASE + 0x08))
#define TIMER_STAT  (*(volatile unsigned int *)(TIMER_BASE + 0x0C))
```

Using `volatile` ensures that all reads and writes directly access the hardware registers.

---

### Firmware Flow

The firmware follows a simple and deterministic sequence:

```c
int main(void)
{
    TIMER_LOAD = HEARTBEAT_INTERVAL;   // Load timer value
    TIMER_CTRL = 0x3;                  // Enable + periodic mode

    while (1) {
        while ((TIMER_STAT & 1) == 0)
            ;                           // Wait for timeout

        TIMER_STAT = 1;                // Clear timeout flag
    }
}
```

* The timer is configured **once**
* Periodic mode ensures automatic reload
* The CPU only waits for status updates
* LED toggling is handled in hardware

---

### Software–Hardware Interaction

* Software **initiates** the timer
* Hardware **runs independently**
* Timeout events are generated without CPU intervention
* This mirrors real-world SoC peripheral usage

---

### Design Considerations

* Firmware avoids busy-loop LED toggling
* No delay loops are used to create blinking
* Timer configuration is clean and minimal
* The design cleanly separates software control from hardware behavior

---

### Files Involved

* `Firmware/timer_test.c` — Timer configuration firmware
* `Firmware/start.S` — Startup code
* `Firmware/Makefile` — Firmware build flow

---

### Outcome of Step-4

Firmware successfully configures the Timer IP, enabling periodic timeout generation and allowing the hardware timer to control system behavior without continuous CPU involvement.

---

## 5. Simulation & Verification

### Description

In this step, the complete Timer IP integration was **verified using RTL simulation**.
Simulation was used to confirm correct interaction between the **RISC-V CPU, system bus, Timer IP, and LED control logic** before programming the FPGA.

GTKWave was used to observe internal signals and validate end-to-end functionality.

---

### Simulation Setup

* Simulator: **Icarus Verilog**
* Waveform viewer: **GTKWave**
* Firmware: `timer_test.c` (simulation interval configured for fast timeout)
* Top module: `SOC`

Simulation was run using:

```bash
make sim
gtkwave soc.vcd
```

---

### Signals Observed

The following signals were monitored during simulation:

#### From SoC / Bus

* `mem_addr`
* `mem_wmask`
* `is_timer`
* `clk`
* `resetn`

#### From Timer IP

* `en`
* `load_reg`
* `value_reg`
* `timeout`

#### From LED Logic

* `led_toggle`
* `LEDS`
* `LED_EXT`

---

### Verification Steps

#### 1. Timer Register Programming

* CPU writes to `TIMER_LOAD` and `TIMER_CTRL`
* `mem_addr` matches `0x20001000`
* `is_timer` asserts during access

✔ Confirms correct **address decoding and bus routing**

---

#### 2. Timer Countdown Operation

* `value_reg` decrements on every clock cycle
* Reloads automatically in periodic mode

✔ Confirms correct **timer core operation**

---

#### 3. Timeout Generation

* `timeout` asserts for one clock cycle when the counter reaches zero
* Timeout pulse is clearly visible in waveform

✔ Confirms correct **timeout logic**

---

#### 4. LED Response

* `led_toggle` toggles on every timeout
* `LEDS` (active-LOW) and `LED_EXT` (active-HIGH) toggle accordingly

✔ Confirms **hardware reaction to timer events**

---

### Simulation Screenshots

The following screenshots were captured for submission:

1. CPU writing to Timer registers
   ![CPU to Timer Write](./snapshots/cpu_to_timer.png)

2. Timer countdown (`value_reg` changing)
   ![Timer Countdown](./snapshots/timer_countdown.png)

3. Timeout pulse assertion
   ![Timeout Pulse](./snapshots/timeout_pulse.png)

4. LED toggle driven by timeout
   ![LED Toggle](./snapshots/led_toggle.png)

5. Combined view showing full data path
    ![End-to-End View](./snapshots/end_view.png)


### Design Considerations

* Simulation timeout interval was reduced to allow faster observation
* Timeout signal was implemented as a registered pulse for visibility
* No free-running hardware blink logic was used

---

### Files Involved

* `RTL/timer_ip.v`
* `RTL/riscv.v`
* `Firmware/timer_test.c`
* `tb_soc.v` (if applicable)

---

### Outcome of Step-5

Simulation successfully verified that the Timer IP is correctly programmed by software, generates periodic timeout events, and drives hardware LED behavior as intended.

---

## 6. FPGA Hardware Demonstration

### Description

In this step, the verified Timer IP design was **synthesized, placed, routed, and programmed onto the FPGA board**.
The objective was to demonstrate that the Timer IP operates correctly not only in simulation but also on **real hardware**, driving physical LEDs through hardware-generated timeout events.

---

### Hardware Setup

* FPGA Board: **VSD Squadron FPGA**
* Clock Frequency: **12 MHz**
* On-board LED:

  * Pin: **39**
  * Polarity: **Active-LOW**
* External LED:

  * Pin: **46**
  * Polarity: **Active-HIGH**
* External LED connected via series resistor to GPIO pin

---

### FPGA Build & Programming

The design was built and programmed using the following flow:

```bash
make clean
make
sudo make flash
```

This process includes:

* RTL synthesis using Yosys
* Place and route using nextpnr
* Bitstream generation
* Programming the FPGA using `iceprog`

---

### Firmware Configuration

For hardware testing, the timer load value was configured for a visible blink rate:

```c
#define HEARTBEAT_INTERVAL 6000000   // ~0.5 seconds @ 12 MHz
```

The timer was enabled in **periodic mode**, allowing continuous operation without CPU intervention.

---

### Observed Hardware Behavior

After programming the FPGA:

* The **on-board LED** (pin 39) blinks at a steady rate
* The **external LED** (pin 46) blinks in synchronization
* Blinking continues autonomously once the timer is configured
* No software delay loops are used for LED control

This confirms that the Timer IP is generating timeout events and directly driving hardware behavior.

---

### Validation Criteria

The hardware demonstration validates:

* Correct Timer IP operation on FPGA
* Proper clock and reset handling
* Accurate memory-mapped control by software
* Hardware-driven LED toggling
* End-to-end functionality from firmware to physical outputs

---

### 📸 Screenshot / Evidence Requirement for Step-6

📷 **Hardware evidence is required for this step**

Recommended evidence:

* Photo or short video of FPGA showing:

  * On-board LED blinking
  * External LED blinking simultaneously

Optional (but strong):

* Screenshot of terminal output (if UART used)
* Board photo with LED connections visible

---

### Files Involved

* `RTL/riscv.v`
* `RTL/timer_ip.v`
* `Firmware/timer_test.c`
* `VSDSquadronFM.pcf`

---

### Outcome of Step-6

The Timer IP was successfully demonstrated on real FPGA hardware, confirming that the design functions correctly beyond simulation and meets all Task-4 requirements.

---
Here is the **final step (Step-7)**, written cleanly and ready to close Task-4 professionally.

You can paste this **directly after Step-6** in your README.

---

## 7. Results & Observations

### Summary of Results

The Task-4 objective of developing a **real memory-mapped Timer IP** was successfully achieved.
The Timer IP was designed, integrated, and validated through both simulation and FPGA hardware demonstration.

Key results include:

* The Timer IP can be configured entirely through software
* The timer operates autonomously in hardware once enabled
* Periodic timeout events are generated reliably
* Hardware outputs (LEDs) are driven directly by the Timer IP
* The design behaves consistently in simulation and on FPGA

---

### Observations

* **Hardware-driven behavior differs fundamentally from software-driven control**
  Unlike GPIO blinking via software loops (Task-3), the Timer IP demonstrates true hardware autonomy.

* **Address decoding is critical**
  Correct memory mapping and bus routing are essential for peripheral functionality.

* **Registered timeout pulses improve reliability**
  Implementing the timeout as a registered one-cycle pulse ensures visibility in simulation and safe hardware usage.

* **Simulation alone is not sufficient**
  FPGA testing validated clocking, reset, and real-world timing behavior.

---

### Challenges Encountered

* Initial timeout pulses were not visible due to incorrect pulse handling
* Address decode omissions prevented the timer from receiving CPU writes
* PCF mismatches caused unconstrained IO errors
* Simulation time scaling required adjustment for efficient verification

All issues were resolved through systematic debugging and validation.

---

### Learning Outcomes

This task provided hands-on experience with:

* Custom peripheral IP design
* Memory-mapped register interfaces
* Hardware-software co-design
* SoC integration and bus architecture
* FPGA bring-up and hardware validation

These concepts are directly applicable to real-world SoC and embedded system development.

---

### Final Status

✔ Timer IP designed
✔ Memory-mapped integration completed
✔ Firmware control verified
✔ Simulation validation complete
✔ FPGA hardware demonstration successful

---

### Conclusion

Task-4 successfully demonstrates the complete workflow of designing and integrating a real peripheral IP into a RISC-V SoC.
The Timer IP functions correctly as an autonomous hardware component, controlled through software and validated on physical hardware.

---

