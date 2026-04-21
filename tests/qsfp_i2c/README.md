# This build does not work yet, use project from Alveo-Cards
# QSFP_I2C (taken from [Alveo-Cards](https://github.com/Xilinx/Alveo-Cards/tree/ul3422) repo)

# Changes:
- in each .xci file, Vivado version is changed from 2023.2 -> 2024.1
- Using tcl build script instead of make project setup.tcl


# Notes:
[HTML docs](https://xilinx.github.io/Alveo-Cards/cards/ul3422/build/html/docs/QSFP_I2C/README.html)
QSFP Management Interface: SFF-8636
SFP+ Management Interface: SFF-8472
SFF Reference Codes: SFF-8024 (refered to in other two docs)

The direct attach cables show up as SFP+ cables, use corresponding document.

### I2C Notes:
- Xilinx docs show I2C address as 7-bit (eg 0x20/0x21 IO expanders, 0x50 transceivers, 0x70 switch/MUX), but in code/scripts they must be refered to in 8-bit address (R/W bit at the end) so 0x40/0x42, 0xA0, 0xE0
- The I2C state machine will stall if a bad read/write is issued, it can't handle a timeout. Easiest way to fix this is to reload the bitstream file

To enable the xcvrs run these in order:
- Turn on corresponding power plane
- 

## How to run
- Connect to device in Vivado HW manager
- Upload bitstream
- In tcl console: source qsfp.tcl (definitions of helper functions)
- In tcl console: Call qsfp_enable_power which will disable then enable qsfp power planes, reset QSFP 0, then read simple Sideband info (see docs for more info). If QSFP 1 also needed, need to call `select_qsfp_sb` with 1 and the two sb reset commands after that