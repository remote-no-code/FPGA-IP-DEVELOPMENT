# =========================================================
# Project Configuration
# =========================================================
TOP            = SOC
VERILOG_FILE   = riscv.v
PCF_FILE       = VSDSquadronFM
FPGA_VARIANT   = up5k
FPGA_PACKAGE   = sg48
BOARD_FREQ     = 10
CPU_FREQ       = 12

# =========================================================
# Firmware Configuration
# =========================================================
FIRMWARE_DIR   = ../Firmware
FIRMWARE_HEX   = firmware.hex
TIMER_FIRMWARE = timer_test.bram.hex

# =========================================================
# UART (optional)
# =========================================================
PICO_DEVICE    = /dev/ttyUSB0
BAUDS          = 9600

# =========================================================
# FPGA BUILD
# =========================================================
build:
	yosys -q -DNEGATIVE_RESET \
		-p "synth_ice40 -dsp -top $(TOP) -json $(TOP).json" \
		$(VERILOG_FILE)

	nextpnr-ice40 --force \
		--json $(TOP).json \
		--pcf $(PCF_FILE).pcf \
		--asc $(TOP).asc \
		--freq $(BOARD_FREQ) \
		--$(FPGA_VARIANT) \
		--package $(FPGA_PACKAGE) \
		--opt-timing

	icetime -p $(PCF_FILE).pcf \
		-P $(FPGA_PACKAGE) \
		-r $(TOP).timings \
		-d $(FPGA_VARIANT) \
		-t $(TOP).asc

	icepack -s $(TOP).asc $(TOP).bin

flash:
	iceprog $(TOP).bin

clean:
	rm -f $(TOP).asc $(TOP).bin $(TOP).json $(TOP).timings soc_sim soc.vcd

# =========================================================
# TERMINAL
# =========================================================
terminal:
	sudo picocom -b $(BAUDS) $(PICO_DEVICE)

# =========================================================
# SIMULATION (IVERILOG) — TIMER-AWARE
# =========================================================

# Default simulation uses timer firmware
sim: sim_timer

# Timer simulation target (Task-4)
sim_timer:
	@echo ">>> Building timer firmware"
	$(MAKE) -C $(FIRMWARE_DIR) timer_test.bram.hex

	@echo ">>> Installing firmware.hex for simulation"
	cp $(FIRMWARE_DIR)/$(TIMER_FIRMWARE) $(FIRMWARE_DIR)/$(FIRMWARE_HEX)

	@echo ">>> Running Icarus Verilog simulation"
	iverilog -g2012 -DBENCH -DSIMULATION \
		-o soc_sim $(VERILOG_FILE)

	vvp soc_sim

