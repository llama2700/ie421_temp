### eth_ila project

This is a modified version of the RECOV_CLK UL3422 port. Port 1 is changed from a RAW mode 4 channel loopback to a MAC configured as a ethernet responder. Only channel 1 is driven. When it receives a ethernet packet, it will send out a predefined ethernet packet (see `<project repo>/tests/ila_eth/RTL/gtfwizard_mac_ex/imports/gtfwizard_mac_gtfmac_ex.v`)


### Build instructions

Vivado version: 2024.1

1) Open vivado and in TCL console, cd to <project repo>/tests/ila_eth/Vivado_Project
2) run `source setup.tcl` in Vivado TCL console 
3) Once project is made, run Reports > Report IP status to ensure all IPs up to date, update IPs if needed
4) Click generate bitstream on left hand bar, will output bitstream and probes file to <project repo>/tests/ila_eth/Vivado_Project/project_1/project_1.runs/impl_1, files will be clk_recov.bit/ltx

Run instructions:

1) Wire NIC to QSFPDD port 1 channel 1 on UL3422
2) Setup Vivado hw_server to UL3422
3) Connect to vivado hw_server, flash the bitstream/debug probes
4) In that tcl console, cd to <project repo>/tests/ila_eth/scripts/HwMgr and source runme.tcl
5) Run `setup` in TCL console, this will enable QSFP power planes and reset GTF transceivers on QSFPDD port 1 channel 1 and QSFPDD port 2 channels 1-4. Port 2 expects a QSFP Loopback adapter, this is used for the loopback test (from RECOV_CLK)