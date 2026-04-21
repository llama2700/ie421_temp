# GTF loopback test

This test achieves near-end loopback on the gtf transceivers
Please refer to Vivado gtf wizard testbench for more info: gtfwizard_0_example_gtfmac_top_tb.sv
This script follows the register write structure of the testbench example

## Necessary Files
* gtfwizard_0_gtfmac_ex.bit
* gtfwizard_0_gtfmac_ex.ltx
* init_gtf.tcl

## How to run
The *init_gtf.tcl* file flashes the bitstream and ltx files, and performs register writes via JTAG-to-AXI to complete the loopback test. Use:
```
vivado -mode batch -source init_gtf.tcl
```

Successful Run:
```
...
PASS: TX == RX == 50                                                                                 
# puts "--- DONE ---"                                                                                
--- DONE ---
```

## Debugging
For debugging purposes, a verbose version of the TCL script is available.
It's recommended that the following command is run to generate a .txt of the output:
```
vivado -mode batch -source init_gtf_verbose.tcl 2>&1 | tee init_gtf_verbose_out.txt
```
The expected verbose output is included for reference.

### To Do
Rewire ILA to see TX/RX data, and other signals (?)
