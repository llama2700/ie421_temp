# Build instructions
- cd to tests/flashing_test
- run vivado -mode batch -source build_flash_test.tcl
- Generates top.bit and top.ltx in build/

# Flashing instructions (GUI)
- Start hw_server on ul3422 host (hft11: /tools/Xilinx/Vivado/2024.2/bin/hw_server)
- connect to remote hw_server thru gui vivado host
- Use HW manager to flash .bit and .ltx
- Open ILA viewer, hit capture/one-shot and see counter