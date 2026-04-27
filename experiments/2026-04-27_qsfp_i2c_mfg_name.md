# 2026-04-27 Querying QSFP I2C for Transceiver Mfg Name

## Goal
Use Xilinx qsfp_i2c example to query manufacture name of transceiver.


## Background
Previously, we were able to power on QSFP power regions, reset transveiers, and read register 0x0. We got code 0x3 which corresponds to SFP/SFP+/SFP28 (see SFF-8024/SFF-8472, more details in `tests/qsfp_i2c/README.md`), but when we query the manufacture name fields, we get 0x0 or gibberish. We had tested on a 1 QSFP -> 4x SFP+ (10g) direct attach cable.

## Plans
Go through power enable sequence with group, retry process with new QSFP28 loopback and a QSFP28 optical transceiver. Some helpers have been written in `tests/qsfp_i2c/qsfp_test.tcl`, relies on `tests/qsfp_i2c/qsfp.tcl`

## Results
Although register 0x0 returns 0x3, the devices follow the QSFP register map, not SFP+. We were able to query the Mfg Name of all 3 devices (DAC cable, QSFP loopback, QSFP optic).

To enable a qsfp port and query follow these commands:
```TCL
# turns on qsfp to device 0/1, which is port 1/2 (0 vs 1 index)
# Should return 0xbe, meaning enable power on 4 ports, port 1 and 2 return PG (4 ports since I assume Xilinx recycled this script from UL3524)
# See qsfp_i2c project Xilinx docs table `Table: QSFP-DD Power Good/Enable Bit Allocation` for more info
enable_qsfp_power_only <device # (0/1)>

# Set I2C switch (i2c 8-bit addr 0xE0) to connect IO expander (i2c 8-bit addr 0x40, note there is 1 per port) corresponding with device
select_qsfp_sb <device # (0/1)>

# Runs reset sequence on QSFP SB IO expander (i2c 8-bit address 0x42)
# Should return 0x12 (resetn deasserted, intl deasserted, etc) or 0x10 (intl asserted) if i2c regs have not been queried.
# See qsfp_i2c project Xilinx docs table `Table: QSFP sideband signal definition` for more info
qsfp_reset_port <device # (0/1)>

# Set I2C switch to QSFP transceiver interface (i2c 8-bit addr 0xA0) and read register 0x0
# Should return 0x3
qsfp_access_i2c <device # (0/1)>

# Ensure high page select is set to 0x0
# Must return 0x0, write to this register if required
i2c_rd 0xA0 0x7F

# Read Manufacture String (0x94 - 0xA3) ascii text mfg id, left aligned and padded with spaces (ascii 0x20)
i2c_rd 0xA0 0x94
i2c_rd 0xA0 0x95
...
i2c_rd 0xA0 0xA3
```

## Setup Modifications
During testing, QSFP loopback in port 2 was replaced with QSFP optic. At the end of testing, QSFP optic was removed, QSFP loopback was not yet replaced (will be done later).

Power to the QSFP planes was turned off.

## Log
Start config:
- Port 1: DAC 1qsfpx4sfp+ , 1st sfp+ connected to hft01 solareflare x2522 port 1
- Port 2: QSFP28 loopback module
- TCL: qsfp_test.tcl and qsfp.tcl has been sourced in TCL console
```TCL
set_property PROBES.FILE {/home/bmn4/group_05_project/bitstreams/qsfp_i2c_top.ltx} [get_hw_devices xcvu2p_0]
set_property FULL_PROBES.FILE {/home/bmn4/group_05_project/bitstreams/qsfp_i2c_top.ltx} [get_hw_devices xcvu2p_0]
set_property PROGRAM.FILE {/home/bmn4/group_05_project/bitstreams/qsfp_i2c_top.bit} [get_hw_devices xcvu2p_0]
program_hw_devices [get_hw_devices xcvu2p_0]
INFO: [Labtools 27-3164] End of startup status: HIGH
program_hw_devices: Time (s): cpu = 00:00:13 ; elapsed = 00:00:09 . Memory (MB): peak = 8868.215 ; gain = 0.000 ; free physical = 105031 ; free virtual = 112316
refresh_hw_device [lindex [get_hw_devices xcvu2p_0] 0]
INFO: [Labtools 27-2302] Device xcvu2p (JTAG device index = 0) is programmed with a design that has 1 ILA core(s).
INFO: [Labtools 27-2302] Device xcvu2p (JTAG device index = 0) is programmed with a design that has 1 JTAG AXI core(s).
qsfp_enable_power_only

-- Deassert Resets to I2C I/O Expanders and Switches and I2C Controller

-- Disable and re-enable QSFP power domains...
-- Disable Power to QSFP 1-2  (set output value and output enable)
[i2c_rd] 0x42 0x0 = 0x00000000
[i2c_wr] 0x42 0x1 = 0x00
[i2c_wr] 0x42 0x3 = 0x55
[i2c_rd] 0x42 0x0 = 0x00000000
-- Enable Power to QSFP 1-2  (set output value and output enable)
[i2c_rd] 0x42 0x0 = 0x00000000
[i2c_wr] 0x42 0x1 = 0xAA
[i2c_wr] 0x42 0x3 = 0x55
[i2c_rd] 0x42 0x0 = 0x000000be
0x000000be
select_qsfp_sb 0
-- Select Gate Mux's to QSFP 0 SB
[i2c_wr] 0xE0 0x01 = 0x01
0x400000e0
i2c_rd 0x40 0x0
[i2c_rd] 0x40 0x0 = 0x00000012
0x00000012
qsfp_reset_port 0

-- Enable QSFP 0...
-- Select Gate Mux's to QSFP 0 SB
[i2c_wr] 0xE0 0x01 = 0x01
-- Asserting QSFP Reset
[i2c_wr] 0x40 0x01 = 0x00
[i2c_wr] 0x40 0x03 = 0x06
-- Deasserting QSFP Reset
[i2c_wr] 0x40 0x01 = 0x10
[i2c_wr] 0x40 0x03 = 0x06
-- Reading QSFP SB Status
[i2c_rd] 0x40 0x00 = 0x00000012
0x00000012
disable_qsfp_power
-- Disable Power to QSFP 1-2  (set output value and output enable)
[i2c_rd] 0x42 0x0 = 0x000000be
[i2c_wr] 0x42 0x1 = 0x00
[i2c_wr] 0x42 0x3 = 0x55
[i2c_rd] 0x42 0x0 = 0x00000000
0x00000000
enable_qsfp_power
-- Enable Power to QSFP 1-2  (set output value and output enable)
[i2c_rd] 0x42 0x0 = 0x00000000
[i2c_wr] 0x42 0x1 = 0xAA
[i2c_wr] 0x42 0x3 = 0x55
[i2c_rd] 0x42 0x0 = 0x000000be
0x000000be
qsfp_reset_port 0

-- Enable QSFP 0...
-- Select Gate Mux's to QSFP 0 SB
[i2c_wr] 0xE0 0x01 = 0x01
-- Asserting QSFP Reset
[i2c_wr] 0x40 0x01 = 0x00
[i2c_wr] 0x40 0x03 = 0x06
-- Deasserting QSFP Reset
[i2c_wr] 0x40 0x01 = 0x10
[i2c_wr] 0x40 0x03 = 0x06
-- Reading QSFP SB Status
[i2c_rd] 0x40 0x00 = 0x00000012
0x00000012
qsfp_access_i2c 0

Example QSFP 0 MODULE I2C Access...
-- Select Gate Mux's to QSFP 0 I2C
[i2c_wr] 0xE0 0x02 = 0x02

-- Read Module Identifier --
[i2c_rd] 0xA0 0x00 = 0x00000011
0x00000011
qsfp_reset_port 1

-- Enable QSFP 1...
-- Select Gate Mux's to QSFP 1 SB
[i2c_wr] 0xE0 0x04 = 0x04
-- Asserting QSFP Reset
[i2c_wr] 0x40 0x01 = 0x00
[i2c_wr] 0x40 0x03 = 0x06
-- Deasserting QSFP Reset
[i2c_wr] 0x40 0x01 = 0x10
[i2c_wr] 0x40 0x03 = 0x06
-- Reading QSFP SB Status
[i2c_rd] 0x40 0x00 = 0x00000012
0x00000012
qsfp_access_i2c 1

Example QSFP 1 MODULE I2C Access...
-- Select Gate Mux's to QSFP 1 I2C
[i2c_wr] 0xE0 0x08 = 0x08

-- Read Module Identifier --
[i2c_rd] 0xA0 0x00 = 0x00000011
0x00000011
i2c_rd 0xA0 0x14
[i2c_rd] 0xA0 0x14 = 0x00000000
0x00000000
i2c_rd 0xA0 0x15
[i2c_rd] 0xA0 0x15 = 0x00000000
0x00000000
i2c_rd 0xA0 0x16
[i2c_rd] 0xA0 0x16 = 0x00000000
0x00000000
i2c_rd 0xA0 0x17
[i2c_rd] 0xA0 0x17 = 0x00000000
0x00000000
i2c_rd 0xA0 0x1
[i2c_rd] 0xA0 0x1 = 0x00000007
0x00000007
i2c_rd 0xA0 0x2
[i2c_rd] 0xA0 0x2 = 0x00000006
0x00000006
i2c_rd 0xA0 0x3
[i2c_rd] 0xA0 0x3 = 0x00000000
0x00000000
i2c_rd 0xE0 0x0
[i2c_rd] 0xE0 0x0 = 0x00000008
0x00000008
i2c_rd 0xA0 0x37
[i2c_rd] 0xA0 0x37 = 0x00000008
0x00000008
reg_rd 0x0
0x800000a0
set_property PROBES.FILE {/home/bmn4/group_05_project/bitstreams/qsfp_i2c_top.ltx} [get_hw_devices xcvu2p_0]
set_property FULL_PROBES.FILE {/home/bmn4/group_05_project/bitstreams/qsfp_i2c_top.ltx} [get_hw_devices xcvu2p_0]
set_property PROGRAM.FILE {/home/bmn4/group_05_project/bitstreams/qsfp_i2c_top.bit} [get_hw_devices xcvu2p_0]
program_hw_devices [get_hw_devices xcvu2p_0]
INFO: [Labtools 27-3164] End of startup status: HIGH
program_hw_devices: Time (s): cpu = 00:00:13 ; elapsed = 00:00:09 . Memory (MB): peak = 8977.508 ; gain = 0.000 ; free physical = 104787 ; free virtual = 112072
refresh_hw_device [lindex [get_hw_devices xcvu2p_0] 0]
INFO: [Labtools 27-2302] Device xcvu2p (JTAG device index = 0) is programmed with a design that has 1 ILA core(s).
INFO: [Labtools 27-2302] Device xcvu2p (JTAG device index = 0) is programmed with a design that has 1 JTAG AXI core(s).
qsfp_reset_port 1

-- Enable QSFP 1...
-- Select Gate Mux's to QSFP 1 SB
[i2c_wr] 0xE0 0x04 = 0x04
-- Asserting QSFP Reset
[i2c_wr] 0x40 0x01 = 0x00
[i2c_wr] 0x40 0x03 = 0x06
-- Deasserting QSFP Reset
[i2c_wr] 0x40 0x01 = 0x10
[i2c_wr] 0x40 0x03 = 0x06
-- Reading QSFP SB Status
[i2c_rd] 0x40 0x00 = 0x00000012
0x00000012
qsfp_access_i2c 0

Example QSFP 0 MODULE I2C Access...
-- Select Gate Mux's to QSFP 0 I2C
[i2c_wr] 0xE0 0x02 = 0x02

-- Read Module Identifier --
[i2c_rd] 0xA0 0x00 = 0x00000011
0x00000011
i2c_rd 0xA0 0x7F
[i2c_rd] 0xA0 0x7F = 0x00000000
0x00000000
i2c_rd 0xA0 0x94
[i2c_rd] 0xA0 0x94 = 0x0000004f
0x0000004f
i2c_rd 0xA0 0x95
[i2c_rd] 0xA0 0x95 = 0x00000045
0x00000045
i2c_rd 0xA0 0x96
[i2c_rd] 0xA0 0x96 = 0x0000004d
0x0000004d
i2c_rd 0xA0 0x97
[i2c_rd] 0xA0 0x97 = 0x00000020
0x00000020
i2c_rd 0xA0 0x98
[i2c_rd] 0xA0 0x98 = 0x00000020
0x00000020
i2c_rd 0xA0 0x99
[i2c_rd] 0xA0 0x99 = 0x00000020
0x00000020
i2c_rd 0xA0 0x9a
[i2c_rd] 0xA0 0x9a = 0x00000020
0x00000020
i2c_rd 0xA0 0x9b
[i2c_rd] 0xA0 0x9b = 0x00000020
0x00000020
i2c_rd 0xA0 0x94
[i2c_rd] 0xA0 0x94 = 0x0000004f
0x0000004f
i2c_rd 0xA0 0x9c
[i2c_rd] 0xA0 0x9c = 0x00000020
0x00000020
i2c_rd 0xA0 0x9d
[i2c_rd] 0xA0 0x9d = 0x00000020
0x00000020
i2c_rd 0xA0 0x9e
[i2c_rd] 0xA0 0x9e = 0x00000020
0x00000020
i2c_rd 0xA0 0x9f
[i2c_rd] 0xA0 0x9f = 0x00000020
0x00000020
i2c_rd 0xA0 0xA0
[i2c_rd] 0xA0 0xA0 = 0x00000020
0x00000020
i2c_rd 0xA0 0xA1
[i2c_rd] 0xA0 0xA1 = 0x00000020
0x00000020
i2c_rd 0xA0 0xA2
[i2c_rd] 0xA0 0xA2 = 0x00000020
0x00000020
i2c_rd 0xA0 0xA3
[i2c_rd] 0xA0 0xA3 = 0x00000020
0x00000020
i2c_rd 0xA0 0xA4
[i2c_rd] 0xA0 0xA4 = 0x00000000
0x00000000
qsfp_reset_port 0

-- Enable QSFP 0...
-- Select Gate Mux's to QSFP 0 SB
[i2c_wr] 0xE0 0x01 = 0x01
-- Asserting QSFP Reset
[i2c_wr] 0x40 0x01 = 0x00
[i2c_wr] 0x40 0x03 = 0x06
-- Deasserting QSFP Reset
[i2c_wr] 0x40 0x01 = 0x10
[i2c_wr] 0x40 0x03 = 0x06
-- Reading QSFP SB Status
[i2c_rd] 0x40 0x00 = 0x00000012
0x00000012
qsfp_access_i2c
wrong # args: should be "qsfp_access_i2c port"
qsfp_access_i2c 0

Example QSFP 0 MODULE I2C Access...
-- Select Gate Mux's to QSFP 0 I2C
[i2c_wr] 0xE0 0x02 = 0x02

-- Read Module Identifier --
[i2c_rd] 0xA0 0x00 = 0x00000011
0x00000011
i2c_rd 0xA0 0x7F
[i2c_rd] 0xA0 0x7F = 0x00000000
0x00000000
i2c_rd 0xA0 0x94
[i2c_rd] 0xA0 0x94 = 0x0000004f
0x0000004f
i2c_rd 0xA0 0x95
[i2c_rd] 0xA0 0x95 = 0x00000045
0x00000045
i2c_rd 0xA0 0x96
[i2c_rd] 0xA0 0x96 = 0x0000004d
0x0000004d
i2c_rd 0xA0 0x97
[i2c_rd] 0xA0 0x97 = 0x00000020
0x00000020
i2c_rd 0xA0 0x98
[i2c_rd] 0xA0 0x98 = 0x00000020
0x00000020
i2c_rd 0xA0 0x99
[i2c_rd] 0xA0 0x99 = 0x00000020
0x00000020
i2c_rd 0xA0 0x9a
[i2c_rd] 0xA0 0x9a = 0x00000020
0x00000020
i2c_rd 0xA0 0x9b
[i2c_rd] 0xA0 0x9b = 0x00000020
0x00000020
i2c_rd 0xA0 0x9c
[i2c_rd] 0xA0 0x9c = 0x00000020
0x00000020
i2c_rd 0xA0 0x9d
[i2c_rd] 0xA0 0x9d = 0x00000020
0x00000020
i2c_rd 0xA0 0x9e
[i2c_rd] 0xA0 0x9e = 0x00000020
0x00000020
i2c_rd 0xA0 0x9f
[i2c_rd] 0xA0 0x9f = 0x00000020
0x00000020
i2c_rd 0xA0 0xa0
[i2c_rd] 0xA0 0xa0 = 0x00000020
0x00000020
i2c_rd 0xA0 0xa1
[i2c_rd] 0xA0 0xa1 = 0x00000020
0x00000020
i2c_rd 0xA0 0xa2
[i2c_rd] 0xA0 0xa2 = 0x00000020
0x00000020
i2c_rd 0xA0 0xa3
[i2c_rd] 0xA0 0xa3 = 0x00000020
0x00000020
i2c_rd 0xA0 0xa4
[i2c_rd] 0xA0 0xa4 = 0x00000000
0x00000000
i2c_rd 0xE0 0x0
[i2c_rd] 0xE0 0x0 = 0x00000002
0x00000002
i2c_rd 0xA0 0x80
[i2c_rd] 0xA0 0x80 = 0x00000002
0x00000002
i2c_rd 0xA0 0x81
[i2c_rd] 0xA0 0x81 = 0x00000002
0x00000002
i2c_rd 0xA0 0x82
[i2c_rd] 0xA0 0x82 = 0x00000002
0x00000002
reg_rd 0x0
0x800000a0
set_property PROBES.FILE {/home/bmn4/group_05_project/bitstreams/qsfp_i2c_top.ltx} [get_hw_devices xcvu2p_0]
set_property FULL_PROBES.FILE {/home/bmn4/group_05_project/bitstreams/qsfp_i2c_top.ltx} [get_hw_devices xcvu2p_0]
set_property PROGRAM.FILE {/home/bmn4/group_05_project/bitstreams/qsfp_i2c_top.bit} [get_hw_devices xcvu2p_0]
program_hw_devices [get_hw_devices xcvu2p_0]
INFO: [Labtools 27-3164] End of startup status: HIGH
program_hw_devices: Time (s): cpu = 00:00:10 ; elapsed = 00:00:09 . Memory (MB): peak = 8977.508 ; gain = 0.000 ; free physical = 104779 ; free virtual = 112065
refresh_hw_device [lindex [get_hw_devices xcvu2p_0] 0]
INFO: [Labtools 27-2302] Device xcvu2p (JTAG device index = 0) is programmed with a design that has 1 ILA core(s).
INFO: [Labtools 27-2302] Device xcvu2p (JTAG device index = 0) is programmed with a design that has 1 JTAG AXI core(s).
i2c_rd 0x42 0x0
[i2c_rd] 0x42 0x0 = 0x000000be
0x000000be
qsfp_disable_power
invalid command name "qsfp_disable_power"
disable_qsfp_power
-- Disable Power to QSFP 1-2  (set output value and output enable)
[i2c_rd] 0x42 0x0 = 0x000000be
[i2c_wr] 0x42 0x1 = 0x00
[i2c_wr] 0x42 0x3 = 0x55
[i2c_rd] 0x42 0x0 = 0x00000000
0x00000000
puts "plugging in qsfp28 optics to port2 (device 1), swapping out loopback"
plugging in qsfp28 optics to port2 (device 1), swapping out loopback
qsfp_enable_power_only

-- Deassert Resets to I2C I/O Expanders and Switches and I2C Controller

-- Disable and re-enable QSFP power domains...
-- Disable Power to QSFP 1-2  (set output value and output enable)
[i2c_rd] 0x42 0x0 = 0x00000000
[i2c_wr] 0x42 0x1 = 0x00
[i2c_wr] 0x42 0x3 = 0x55
[i2c_rd] 0x42 0x0 = 0x00000000
-- Enable Power to QSFP 1-2  (set output value and output enable)
[i2c_rd] 0x42 0x0 = 0x00000000
[i2c_wr] 0x42 0x1 = 0xAA
[i2c_wr] 0x42 0x3 = 0x55
[i2c_rd] 0x42 0x0 = 0x000000be
0x000000be
qsfp_reset_port 1

-- Enable QSFP 1...
-- Select Gate Mux's to QSFP 1 SB
[i2c_wr] 0xE0 0x04 = 0x04
-- Asserting QSFP Reset
[i2c_wr] 0x40 0x01 = 0x00
[i2c_wr] 0x40 0x03 = 0x06
-- Deasserting QSFP Reset
[i2c_wr] 0x40 0x01 = 0x10
[i2c_wr] 0x40 0x03 = 0x06
-- Reading QSFP SB Status
[i2c_rd] 0x40 0x00 = 0x00000010
0x00000010
qsfp_reset_port 1

-- Enable QSFP 1...
-- Select Gate Mux's to QSFP 1 SB
[i2c_wr] 0xE0 0x04 = 0x04
-- Asserting QSFP Reset
[i2c_wr] 0x40 0x01 = 0x00
[i2c_wr] 0x40 0x03 = 0x06
-- Deasserting QSFP Reset
[i2c_wr] 0x40 0x01 = 0x10
[i2c_wr] 0x40 0x03 = 0x06
-- Reading QSFP SB Status
[i2c_rd] 0x40 0x00 = 0x00000010
0x00000010
qsfp_access_i2c 1

Example QSFP 1 MODULE I2C Access...
-- Select Gate Mux's to QSFP 1 I2C
[i2c_wr] 0xE0 0x08 = 0x08

-- Read Module Identifier --
[i2c_rd] 0xA0 0x00 = 0x00000011
0x00000011
i2c_rd 0xE0 0x0
[i2c_rd] 0xE0 0x0 = 0x00000018
0x00000018
i2c_rd 0xA0 0x7F
[i2c_rd] 0xA0 0x7F = 0x00000018
0x00000018
reg_rd 0
0x00000000
reg_rd 0x0
0x800000a0
set_property PROBES.FILE {/home/bmn4/group_05_project/bitstreams/qsfp_i2c_top.ltx} [get_hw_devices xcvu2p_0]
set_property FULL_PROBES.FILE {/home/bmn4/group_05_project/bitstreams/qsfp_i2c_top.ltx} [get_hw_devices xcvu2p_0]
set_property PROGRAM.FILE {/home/bmn4/group_05_project/bitstreams/qsfp_i2c_top.bit} [get_hw_devices xcvu2p_0]
program_hw_devices [get_hw_devices xcvu2p_0]
INFO: [Labtools 27-3164] End of startup status: HIGH
program_hw_devices: Time (s): cpu = 00:00:10 ; elapsed = 00:00:09 . Memory (MB): peak = 8977.508 ; gain = 0.000 ; free physical = 104769 ; free virtual = 112057
refresh_hw_device [lindex [get_hw_devices xcvu2p_0] 0]
INFO: [Labtools 27-2302] Device xcvu2p (JTAG device index = 0) is programmed with a design that has 1 ILA core(s).
INFO: [Labtools 27-2302] Device xcvu2p (JTAG device index = 0) is programmed with a design that has 1 JTAG AXI core(s).
qsfp_access_i2c 1

Example QSFP 1 MODULE I2C Access...
-- Select Gate Mux's to QSFP 1 I2C
[i2c_wr] 0xE0 0x08 = 0x08

-- Read Module Identifier --
[i2c_rd] 0xA0 0x00 = 0x00000011
0x00000011
reg_rd 0x0
0xc00000a0
i2c_rd 0xA0 0x00
[i2c_rd] 0xA0 0x00 = 0x00000011
0x00000011
i2c_rd 0xA0 0x7F
[i2c_rd] 0xA0 0x7F = 0x00000000
0x00000000
i2c_rd 0xA0 0x94
[i2c_rd] 0xA0 0x94 = 0x00000043
0x00000043
i2c_rd 0xA0 0x95
[i2c_rd] 0xA0 0x95 = 0x0000006f
0x0000006f
i2c_rd 0xA0 0x96
[i2c_rd] 0xA0 0x96 = 0x0000006c
0x0000006c
i2c_rd 0xA0 0x97
[i2c_rd] 0xA0 0x97 = 0x0000006f
0x0000006f
i2c_rd 0xA0 0x98
[i2c_rd] 0xA0 0x98 = 0x00000072
0x00000072
i2c_rd 0xA0 0x999
[i2c_rd] 0xA0 0x999 = 0x00000043
0x00000043
i2c_rd 0xA0 0x99
[i2c_rd] 0xA0 0x99 = 0x00000043
0x00000043
i2c_rd 0xA0 0x9a
[i2c_rd] 0xA0 0x9a = 0x00000068
0x00000068
i2c_rd 0xA0 0x9b
[i2c_rd] 0xA0 0x9b = 0x00000069
0x00000069
i2c_rd 0xA0 0x9c
[i2c_rd] 0xA0 0x9c = 0x00000070
0x00000070
i2c_rd 0xA0 0x9d
[i2c_rd] 0xA0 0x9d = 0x00000020
0x00000020
i2c_rd 0xA0 0x9e
[i2c_rd] 0xA0 0x9e = 0x0000006c
0x0000006c
i2c_rd 0xA0 0x9f
[i2c_rd] 0xA0 0x9f = 0x00000074
0x00000074
i2c_rd 0xA0 0xa0
[i2c_rd] 0xA0 0xa0 = 0x00000064
0x00000064
i2c_rd 0xA0 0xa1
[i2c_rd] 0xA0 0xa1 = 0x00000020
0x00000020
i2c_rd 0xA0 0xa2
[i2c_rd] 0xA0 0xa2 = 0x00000020
0x00000020
i2c_rd 0xA0 0xa3
[i2c_rd] 0xA0 0xa3 = 0x00000020
0x00000020
disable_qsfp_power
-- Disable Power to QSFP 1-2  (set output value and output enable)
[i2c_rd] 0x42 0x0 = 0x000000be
[i2c_wr] 0x42 0x1 = 0x00
[i2c_wr] 0x42 0x3 = 0x55
[i2c_rd] 0x42 0x0 = 0x00000000
0x00000000
```
End config:
- Port 1: DAC 1qsfpx4sfp+ , 1st sfp+ connected to hft01 solareflare x2522 port 1
- Port 2: Disconnected (will replace QSFP28 loopback)