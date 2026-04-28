# Path variables
# Note: Ensure XILINX_VIVADO is set in your environment or defined here
# Source vivado's settings64.sh
VIVADO_PATH = $(XILINX_VIVADO)/bin/vivado
BUILD_TCL   = ./build_qsfp_test.tcl
LOG_FILE	= ./build/make_build.log

# Default rule
all:
	mkdir -p build/
	$(VIVADO_PATH) -mode batch -source $(BUILD_TCL) 2>&1 | tee $(LOG_FILE)

# Cleanup rule
clean:
	rm -rf build/ .gen/ .srcs/ .Xil/
	rm vivado*.log vivado*.jou


.PHONY: all clean