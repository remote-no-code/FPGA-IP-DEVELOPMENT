Here is the **fully rewritten, corrected, and commercially compliant** version of **`IP_User_Guide.md`**, aligned with your RTL, register map, and software model.

You can **replace your entire file** with the content below.

---

# Timer IP – User Guide

## 1. Introduction

The Timer IP is a **memory-mapped hardware timer peripheral** designed for the **VSDSquadron FPGA platform**.
It allows software running on a RISC-V processor to perform accurate time-based operations **without using software delay loops**.

Once configured, the Timer IP operates **autonomously in hardware**, generating timeout events that can be observed by software or used directly by hardware logic such as LEDs or control signals.

---

## 2. Purpose of the IP

The Timer IP provides a programmable countdown timer that can be used for:

* Periodic heartbeat generation
* Time delays and polling intervals
* Hardware-driven LED blinking
* Event triggering without CPU busy-wait loops

This IP is intended to be **simple, reliable, and reusable** across VSDSquadron-based SoCs.

---

## 3. Key Features

* 32-bit programmable countdown timer
* Memory-mapped register interface
* Software-controlled enable and mode selection
* One-shot and periodic (auto-reload) modes
* Sticky timeout flag with write-one-to-clear (W1C) behavior
* Hardware timeout signal for external logic
* Verified using RTL simulation and FPGA hardware

---

## 4. Functional Overview

The Timer IP consists of the following logical components:

* **Control Register (CTRL)**
  Enables the timer and selects one-shot or periodic mode.

* **Load Register (LOAD)**
  Holds the initial countdown value programmed by software.

* **Counter Register (VALUE)**
  Decrements on every clock cycle while the timer is enabled.

* **Status Register (STATUS)**
  Indicates when a timeout event has occurred.

When the timer is enabled, the counter loads the value from the LOAD register and begins counting down.
When the counter reaches zero, a timeout event is generated.

In periodic mode, the counter automatically reloads and continues running.

---

## 5. Block Diagram

The conceptual block diagram of the Timer IP is shown below:

![Timer IP Block Diagram](Block_Diagram.png)

This diagram illustrates the relationship between the bus interface, register file, counter logic, and timeout output.

---

## 6. Operating Modes

### 6.1 One-Shot Mode

* Timer counts down once
* Timeout flag is set when the counter reaches zero
* Timer stops after timeout
* Software must re-enable the timer for another cycle

### 6.2 Periodic Mode

* Timer automatically reloads from the LOAD register
* Timeout flag is set on every expiration
* Timer continues running until disabled by software

---

## 7. Timeout and Status Behavior (IMPORTANT)

When the timer counter reaches zero:

* The internal **timeout flag is set to `1`**
* The timeout flag remains asserted (**sticky**) until cleared by software
* Software clears the timeout flag by writing `1` to the STATUS register (W1C behavior)

This design ensures:

* Reliable polling without missing timeout events
* Compatibility with memory-mapped software programming models
* Deterministic behavior in both one-shot and periodic modes

The timeout flag is also exposed internally as a `timeout` signal, allowing it to drive hardware logic such as LED toggling.

---

## 8. Clock and Reset Behavior

* The Timer IP operates on the system clock (`clk`)
* Reset is **active-low** (`resetn`)

On reset:

* Timer is disabled
* Counter value is cleared
* Timeout flag is deasserted

After reset, software must reprogram the LOAD and CTRL registers to start the timer.

---

## 9. Typical Use Cases

Common use cases for the Timer IP include:

* LED heartbeat blinking on FPGA boards
* Periodic software polling intervals
* Delay generation without CPU busy loops
* Hardware event triggering based on elapsed time

---

## 10. Validation Summary

The Timer IP has been validated using:

* RTL simulation with GTKWave
* Register-level software testing
* FPGA hardware testing on the VSDSquadron board
* LED-based visual confirmation of timeout events

Both simulation and hardware results confirm correct functionality.

---

## 11. Known Limitations & Notes

* No interrupt output is provided (polling-based operation only)
* Single timer channel only
* Timer resolution depends on system clock frequency
* Assumes synchronous memory-mapped bus access

These limitations are intentional to keep the IP lightweight and easy to integrate.

---

## 12. Related Documents

* **Register_Map.md**
* **Integration_Guide.md**
* **Example_Usage.md**

---

## 13. Conclusion

The Timer IP provides a clean, reliable, and hardware-verified timing solution for VSDSquadron-based systems.
Its simple programming model and predictable behavior make it suitable for both learning and real-world FPGA SoC designs.

---
