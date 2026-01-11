# Timer IP – VSDSquadron FPGA

## Overview
This repository provides a **memory-mapped Timer IP** for the VSDSquadron FPGA platform.
The IP implements a programmable 32-bit countdown timer that can be controlled entirely
from software running on the on-board RISC-V processor.

The Timer IP is suitable for:
- Delays
- Periodic events
- Heartbeat LED blinking
- Time-based polling in embedded systems

---

## Features
- 32-bit programmable countdown timer
- Memory-mapped register interface
- One-shot and periodic operation
- Hardware-generated timeout pulse
- Software polling via status register
- Tested on VSDSquadron FPGA hardware

---

## Repository Structure


```
ip/timer_ip/
├── rtl/              # RTL implementation
├── software/         # Example C software
├── docs/             # Detailed documentation
└── README.md         # This file
```
---

## Quick Start
1. Instantiate `timer_ip.v` in your SoC
2. Map the Timer IP at a free address (example: `0x20001000`)
3. Use the provided C example to configure and start the timer
4. Observe LED toggling on timeout

---

## Documentation
Detailed documentation is available in the `docs/` directory:
- **IP_User_Guide.md** – Functional overview and usage
- **Register_Map.md** – Complete register definitions
- **Integration_Guide.md** – RTL and SoC integration steps
- **Example_Usage.md** – Ready-to-run software example

---

## Validation
- Verified via GTKWave simulation
- Verified on VSDSquadron FPGA hardware
- Demonstrates autonomous hardware operation

---

## License
Educational / Open-source use for VSDSquadron FPGA
```

