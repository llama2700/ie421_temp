# Path variables
# Note: Ensure XILINX_VIVADO is set in your environment or defined here
# Source vivado's settings64.sh
VIVADO_PATH = $(XILINX_VIVADO)/bin/vivado
BUILD_TCL   = ./build_flash_test.tcl

# Default rule
all:
	$(VIVADO_PATH) -mode batch -source $(BUILD_TCL)

# Cleanup rule
clean:
	rm -rf build/ .gen/ .srcs/ .Xil/
	rm vivado*.log vivado*.jou clockInfo.txt

.PHONY: all clean