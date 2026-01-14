# Timer IP – User Guide

## 1. Introduction

The Timer IP is a **memory-mapped hardware timer peripheral** designed for the VSDSquadron FPGA platform.
It allows software running on the on-board RISC-V processor to configure and control time-based operations without relying on software delay loops.

Once configured, the Timer IP operates **autonomously in hardware**, generating timeout events that can be used to drive LEDs or other logic.

---

## 2. Key Features

* 32-bit programmable countdown timer
* Memory-mapped register interface
* Software-controlled enable and mode selection
* Periodic (auto-reload) operation
* Hardware-generated timeout pulse
* Suitable for delays, polling, and heartbeat indicators
* Verified via simulation and FPGA hardware

---

## 3. Functional Overview

The Timer IP consists of:

* A **load register** that stores the countdown value
* A **counter register** that decrements on each clock cycle
* A **control register** to start and configure the timer
* A **status register** that indicates timeout events

When enabled, the timer loads the value from the load register and begins counting down.
When the counter reaches zero, a **timeout pulse** is generated.

In periodic mode, the timer automatically reloads and continues operation.

---

## 4. Block Diagram (Conceptual)

![Timer IP Block Diagram](Block_Diagram.png)

---

## 5. Operating Modes

### 5.1 One-Shot Mode

* Timer counts down once
* Timeout is generated
* Timer stops after reaching zero

### 5.2 Periodic Mode

* Timer automatically reloads
* Timeout is generated periodically
* No software re-trigger required

---

## 6. Timeout Signal Behavior

* The `timeout` signal is asserted for **one clock cycle**
* It is cleared automatically in hardware
* Software can also observe timeout via the status register

This design ensures:

* Clean pulse generation
* Safe hardware triggering
* Easy waveform visibility during simulation

---

## 7. Clock and Reset Behavior

* Operates on the system clock (`clk`)
* Active-low reset (`resetn`)
* On reset:

  * Timer is disabled
  * Counter is cleared
  * Timeout is deasserted

---

## 8. Typical Use Cases

* LED heartbeat blinking
* Periodic polling in embedded software
* Time delays without CPU busy-wait loops
* Event triggering in hardware

---

## 9. Validation Summary

The Timer IP has been:

* Verified using RTL simulation (GTKWave)
* Tested on VSDSquadron FPGA hardware
* Demonstrated driving both on-board and external LEDs

---

## 10. Related Documents

* **Register_Map.md** – Detailed register definitions
* **Integration_Guide.md** – SoC and RTL integration steps
* **Example_Usage.md** – Example C code for timer usage

---

## 11. Conclusion

The Timer IP provides a reliable, reusable, and hardware-verified timing solution for VSDSquadron FPGA-based systems.
It enables clean separation between software control and hardware execution.

---
