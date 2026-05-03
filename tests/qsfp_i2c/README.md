# QSFP_I2C (taken from [Alveo-Cards](https://github.com/Xilinx/Alveo-Cards/tree/ul3422) repo)

# Changes:
- in each .xci file, Vivado version is changed from 2023.2 -> 2024.1
- Using custom non-project mode tcl build script instead of make project_setup.tcl

# Build instructions
1) Source vivado settings64.sh (eg. on hft06 `source /tools/Xilinx/Vivado/2024.2/settings64.sh`)
2) In this directory (contains this README.md and Makefile) run `make all`
3) Resulting DCPs and bitstream/debug probes file will be in generated director `build/`

# Notes:
Useful links/documentations
- [Xilinx qsfp_i2c repo HTML docs](https://xilinx.github.io/Alveo-Cards/cards/ul3422/build/html/docs/QSFP_I2C/README.html)
- QSFP Management Interface: SFF-8636
- SFP+ Management Interface: SFF-8472
- SFF Reference Codes: SFF-8024 (refered to in other two docs)

The direct attach cables and QSFP loopback modules we used show as Device Identfier 0x3 (SFP) but follow QSFP Management Inferface

### I2C Notes:
- Xilinx docs switch between as 7-bit I2C address (eg 0x20/0x21 IO expanders, 0x50 transceivers, 0x70 switch/MUX), and 8-bit address (R/W bit at the end) so 0x40/0x42, 0xA0, 0xE0. Keep this in mind, 8-bit is used in the TCL scripts, both 7/8bit is used in the HTML docs.
- The I2C state machine will stall if a bad read/write is issued, it can't handle a timeout. The documented reset register isn't actually wired to anything, so you mus reload the bitstream file to reset the I2C state machine

## How to run
- Connect to device in Vivado HW manager
- Upload bitstream
- Open TCL console and run following commands
    - `source qsfp.tcl` and `qsfp_test.tcl` (definitions of helper functions)
    - `qsfp_enable_power_only` to turn on power regulators to QSFP power planes
    - `qsfp_reset_port <0/1>` run with corresponding port you want to reset on, run 2x to reset (note this is 0 indexed while diagram of UL3422 is 1-indexed) This while also print value of QSFP SB register for that port
    - `qsfp_access_i2c <0/1>` to connect I2C switch to transceiver I2C interface of corresponding port and print contents of register 0x0 (Identifier)
    - todo add tcl proc to dump more stats, for now can query using `i2c_rd 0xA0 <register/offset #>` interesting offset is 0x94-0xA3 which is QSFP map for left aligned space padded mfg name. Note upper page select register (offset 0x7F) must be 0x0 to acces 0x94-0xA3 as mfg name. Either read this register and cofirm first or write to it.
