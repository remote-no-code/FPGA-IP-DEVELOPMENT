# Timer IP – Example Usage

## 1. Purpose

This document provides a **working software example** demonstrating how to use the Timer IP from C code running on the RISC-V processor.

The example configures the Timer IP in **periodic mode** and uses the hardware-generated timeout event to toggle LEDs on the FPGA board.

---

## 2. Assumptions

This example assumes:

* Timer IP is mapped at base address `0x20001000`
* System clock frequency is **12 MHz**
* Timer IP is already integrated into the SoC
* LEDs are connected to timer-driven hardware logic

---

## 3. Timer Register Definitions

```c
#define TIMER_BASE   0x20001000

#define TIMER_CTRL  (*(volatile unsigned int *)(TIMER_BASE + 0x00))
#define TIMER_LOAD  (*(volatile unsigned int *)(TIMER_BASE + 0x04))
#define TIMER_VALUE (*(volatile unsigned int *)(TIMER_BASE + 0x08))
#define TIMER_STAT  (*(volatile unsigned int *)(TIMER_BASE + 0x0C))
```

The `volatile` keyword ensures that all accesses reach the hardware.

---

## 4. Example: Periodic LED Blink

### Description

* The timer is configured to generate a timeout every **0.5 seconds**
* The timer runs autonomously in hardware
* LEDs toggle on each timeout event
* The CPU does not implement any delay loops

---

### Example Code

```c
#define HEARTBEAT_INTERVAL 6000000   // ~0.5 seconds @ 12 MHz

int main(void)
{
    // Program timer load value
    TIMER_LOAD = HEARTBEAT_INTERVAL;

    // Enable timer in periodic mode
    TIMER_CTRL = 0x3;   // bit0 = enable, bit1 = periodic

    while (1)
    {
        // Wait for timeout event
        while ((TIMER_STAT & 1) == 0)
            ;

        // Clear timeout flag
        TIMER_STAT = 1;
    }

    return 0;
}
```

---

## 5. Expected Behavior

After programming the FPGA and running the firmware:

* On-board LED (active-LOW) blinks periodically
* External LED (active-HIGH) blinks in sync
* Blink rate ≈ 0.5 seconds
* Blinking continues without CPU involvement

This confirms **hardware-driven operation** of the Timer IP.

---

## 6. Modifying the Timer Interval

To change the blink rate, update the load value:

```c
#define HEARTBEAT_INTERVAL 12000000   // ~1 second
```

General formula:

```
Timer interval (seconds) = LOAD_VALUE / CLOCK_FREQUENCY
```

---

## 7. Polling the Timer Value (Optional)

Software can read the current countdown value:

```c
unsigned int remaining = TIMER_VALUE;
```

This can be useful for diagnostics or time measurement.

---

## 8. Clearing the Timeout Flag

The timeout flag must be cleared by writing `1` to `TIMER_STAT`:

```c
TIMER_STAT = 1;
```

Failure to clear the flag may prevent detection of subsequent timeouts.

---

## 9. Common Issues and Debug Tips

| Issue                 | Possible Cause         |
| --------------------- | ---------------------- |
| LED does not blink    | Timer not enabled      |
| Timeout never asserts | LOAD value too large   |
| Read returns zero     | Incorrect base address |
| LED always ON/OFF     | Polarity mismatch      |

---

## 10. Related Documents

* **IP_User_Guide.md**
* **Register_Map.md**
* **Integration_Guide.md**

