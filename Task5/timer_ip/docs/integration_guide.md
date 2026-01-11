# Timer IP – Integration Guide

## 1. Purpose

This document describes how to **integrate the Timer IP into a VSDSquadron-based SoC**.
It covers RTL instantiation, address mapping, bus connections, and LED hookup.

This guide is intended for **SoC and FPGA system integrators**.

---

## 2. Prerequisites

Before integration, ensure the following are available:

* VSDSquadron SoC with RISC-V CPU
* Memory-mapped bus signals:

  * `mem_addr`
  * `mem_wdata`
  * `mem_wmask`
  * `mem_rdata`
* System clock (`clk`)
* Active-low reset (`resetn`)

---

## 3. RTL File Inclusion

Add the Timer IP RTL file to your build system:

```
ip/timer_ip/rtl/timer_ip.v
```

Ensure it is included in synthesis and simulation sources.

---

## 4. Address Mapping

Choose a free address range for the Timer IP.

### Recommended Mapping

```verilog
localparam TIMER_BASE = 32'h2000_1000;
```

Address decode condition:

```verilog
wire is_timer = (mem_addr[31:12] == TIMER_BASE[31:12]);
```

This provides a 4 KB address window for the Timer IP.

---

## 5. Timer IP Instantiation

Instantiate the Timer IP in the SoC top module:

```verilog
wire [31:0] timer_rdata;
wire        timer_timeout;

timer_ip TIMER (
    .clk        (clk),
    .resetn     (resetn),
    .addr       (mem_addr),
    .wdata      (mem_wdata),
    .we         (is_timer & |mem_wmask),
    .sel        (is_timer),
    .rdata      (timer_rdata),
    .timeout    (timer_timeout)
);
```

---

## 6. Read Data Multiplexing

Add the Timer IP to the SoC read-data mux:

```verilog
assign mem_rdata =
    is_ram   ? ram_rdata   :
    is_gpio  ? gpio_rdata  :
    is_timer ? timer_rdata :
    32'b0;
```

This ensures CPU reads from the correct peripheral.

---

## 7. LED Connection Example

The Timer IP generates a `timeout` pulse that can drive hardware logic.

### Example: LED Toggle Logic

```verilog
reg led_toggle;

always @(posedge clk or negedge resetn) begin
    if (!resetn)
        led_toggle <= 1'b0;
    else if (timer_timeout)
        led_toggle <= ~led_toggle;
end

assign LED_EXT = led_toggle;        // external LED (active-HIGH)
assign LEDS    = {5{~led_toggle}};  // onboard LEDs (active-LOW)
```

This logic demonstrates **hardware-driven behavior**, independent of CPU loops.

---

## 8. Reset Behavior

On reset:

* Timer is disabled
* Counter is cleared
* Timeout signal is deasserted
* LEDs are turned OFF

Ensure reset polarity matches system conventions.

---

## 9. Timing & Clocking Notes

* Timer operates on system clock
* Countdown rate = 1 decrement per clock cycle
* Timer resolution depends on clock frequency

Example:

* 12 MHz clock → 12 million ticks per second

---

## 10. Verification After Integration

After integration:

* Run RTL simulation
* Verify register access
* Confirm timeout generation
* Validate LED toggling
* Program FPGA and observe hardware behavior

---

## 11. Common Integration Errors

| Issue                 | Cause                    |
| --------------------- | ------------------------ |
| Timer not responding  | Incorrect address decode |
| Timeout never asserts | Timer not enabled        |
| LED stuck             | Missing toggle logic     |
| Read returns zero     | Read mux not updated     |

---

## 12. Related Documents

* **Register_Map.md**
* **IP_User_Guide.md**
* **Example_Usage.md**

---
