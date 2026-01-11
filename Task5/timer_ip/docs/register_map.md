# Timer IP – Register Map

## 1. Overview

The Timer IP is accessed through a **memory-mapped register interface**.
All registers are **32-bit wide** and aligned on 4-byte boundaries.

The base address of the Timer IP is configurable during SoC integration.

> **Example Base Address:** `0x2000_1000`

---

## 2. Register Summary

| Offset | Register Name | Access | Description              |
| ------ | ------------- | ------ | ------------------------ |
| 0x00   | TIMER_CTRL    | R/W    | Timer control register   |
| 0x04   | TIMER_LOAD    | R/W    | Load value for countdown |
| 0x08   | TIMER_VALUE   | R      | Current counter value    |
| 0x0C   | TIMER_STAT    | R/W    | Timer status register    |

---

## 3. Register Descriptions

---

### 3.1 TIMER_CTRL (Offset: `0x00`)

**Purpose:**
Controls timer enable and operating mode.

**Access:** Read / Write
**Reset Value:** `0x00000000`

#### Bit Definition

| Bit  | Name | Description                              |
| ---- | ---- | ---------------------------------------- |
| 0    | EN   | Timer enable (1 = enable, 0 = disable)   |
| 1    | MODE | Mode select (0 = one-shot, 1 = periodic) |
| 31:2 | –    | Reserved (write as 0)                    |

#### Example

```c
// Enable timer in periodic mode
TIMER_CTRL = 0x3;
```

---

### 3.2 TIMER_LOAD (Offset: `0x04`)

**Purpose:**
Specifies the countdown start value.

**Access:** Read / Write
**Reset Value:** `0x00000000`

#### Bit Definition

| Bit  | Description          |
| ---- | -------------------- |
| 31:0 | Countdown load value |

When the timer is enabled, this value is loaded into the counter.

#### Example

```c
// Set timer interval
TIMER_LOAD = 6000000;
```

---

### 3.3 TIMER_VALUE (Offset: `0x08`)

**Purpose:**
Provides the current countdown value.

**Access:** Read-Only
**Reset Value:** `0x00000000`

#### Bit Definition

| Bit  | Description           |
| ---- | --------------------- |
| 31:0 | Current counter value |

This register decrements on each clock cycle when the timer is enabled.

#### Example

```c
unsigned int current = TIMER_VALUE;
```

---

### 3.4 TIMER_STAT (Offset: `0x0C`)

**Purpose:**
Indicates timeout status and allows software to clear the timeout flag.

**Access:** Read / Write
**Reset Value:** `0x00000000`

#### Bit Definition

| Bit  | Name    | Description               |
| ---- | ------- | ------------------------- |
| 0    | TIMEOUT | Set to 1 on timeout event |
| 31:1 | –       | Reserved                  |

* Writing `1` clears the timeout flag
* Writing `0` has no effect

#### Example

```c
// Clear timeout flag
TIMER_STAT = 1;
```

---

## 4. Register Access Notes

* All registers are **synchronous to the system clock**
* Reserved bits must be written as zero
* Reading undefined bits returns zero
* Timeout flag is also reflected via internal `timeout` signal

---

## 5. Typical Register Programming Sequence

```c
TIMER_LOAD = interval;
TIMER_CTRL = 0x3;   // enable + periodic

while (!(TIMER_STAT & 1));
// timeout occurred

TIMER_STAT = 1;     // clear flag
```

---

## 6. Address Calculation Example

If base address = `0x20001000`:

| Register    | Address      |
| ----------- | ------------ |
| TIMER_CTRL  | `0x20001000` |
| TIMER_LOAD  | `0x20001004` |
| TIMER_VALUE | `0x20001008` |
| TIMER_STAT  | `0x2000100C` |

---

## 7. Related Documents

* **IP_User_Guide.md**
* **Integration_Guide.md**
* **Example_Usage.md**

---
