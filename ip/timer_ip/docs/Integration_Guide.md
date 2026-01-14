
# Timer IP – Integration Guide

## 1. Purpose
This document describes how to integrate the **Timer IP** into a
VSDSquadron-based SoC. It covers RTL inclusion, address mapping,
bus connections, and an example of hardware usage.

This guide is intended for **SoC designers and FPGA system integrators**.

---

## 2. Prerequisites
Before integration, ensure the following are available:

- VSDSquadron SoC with a RISC-V CPU
- System clock (`clk`)
- Active-low reset (`resetn`)
- Memory-mapped bus signals:
  - `mem_addr`
  - `mem_wdata`
  - `mem_wmask`
  - `mem_rdata`

---

## 3. RTL File Inclusion
Include the Timer IP RTL file in the build system:

```
ip/timer_ip/rtl/timer_ip.v
````

Ensure this file is included in both synthesis and simulation sources.

---

## 4. Address Mapping
Assign a free address range to the Timer IP in the SoC memory map.

### Recommended Base Address
```verilog
localparam TIMER_BASE = 32'h2000_1000;
````

### Address Decode

```verilog
wire is_timer = (mem_addr[31:12] == TIMER_BASE[31:12]);
```

This allocates a 4 KB address window for the Timer IP.

---

## 5. Timer IP Instantiation

Instantiate the Timer IP in the SoC top-level module:

```verilog
wire [31:0] timer_rdata;
wire        timer_timeout;

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

---

## 6. Read Data Multiplexing

Add the Timer IP to the SoC read-data multiplexer:

```verilog
assign mem_rdata =
    is_ram   ? ram_rdata   :
    is_gpio  ? gpio_rdata  :
    is_timer ? timer_rdata :
    32'b0;
```

This ensures the CPU reads data from the correct peripheral.

---

## 7. Example Hardware Usage

The Timer IP asserts the `timeout` signal when the countdown expires.
This signal can be used to drive hardware logic.

### Example: LED Toggle Logic

```verilog
reg led_toggle;

always @(posedge clk or negedge resetn) begin
    if (!resetn)
        led_toggle <= 1'b0;
    else if (timer_timeout)
        led_toggle <= ~led_toggle;
end

assign LED_EXT = led_toggle;        // External LED (active-HIGH)
assign LEDS    = {5{~led_toggle}};  // On-board LEDs (active-LOW)
```

This demonstrates **hardware-driven behavior**, independent of software delay loops.

---

## 8. Reset Behavior

On reset:

* Timer is disabled
* Counter value is cleared
* Timeout flag is deasserted

Ensure reset polarity matches the system reset convention.

---

## 9. Timing and Clocking Notes

* Timer operates on the system clock
* Countdown rate is one decrement per clock cycle
* Timer resolution depends on clock frequency

Example:

* 12 MHz clock → 12 million timer ticks per second

---

## 10. Verification After Integration

After integration:

* Run RTL simulation
* Verify register read/write access
* Confirm timeout assertion
* Validate hardware behavior
* Program FPGA and observe expected output

---

## 11. Common Integration Errors

| Issue                 | Possible Cause            |
| --------------------- | ------------------------- |
| Timer not responding  | Incorrect address decode  |
| Timeout never asserts | Timer not enabled         |
| LED stuck ON/OFF      | Missing toggle logic      |
| Read returns zero     | Read-data mux not updated |

---

## 12. Related Documents

- [IP User Guide](IP_User_Guide.md)  
- [Register Map](Register_Map.md)  
- [Example Usage](Example_Usage.md)  

