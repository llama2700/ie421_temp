# 2026-04-27 Test bitstreams from hft06

## Goal
Test bitstreams from hft06. hft06 is running 2024.2, I have been runnign 2024.1 locally. Need to check bitstreams still work with new compile system + hft06 setup.

## Background
See goal

## Plans
Flash hft06_top.bit/ltx (flashing_test) and see if ILA can see counter.
Flash hft06_qsfp_i2c_top.bit/ltx (xilnx i2c test) and see if we can still cycle power + read mfg id
Not sure what image is currently on FPGA right now. Will leave hft06_qsfp_i2c_top at end.

## Results
both flashing_test and qsfp_i2c_top works. However, during qsfp_i2c_top test, reseting qsft port 1 returned 0x16 instead of 0x12. This corresponds to MODPRSL being deasserted, meaning no QSFP module is there. I misread the bit map and tried to query it anyways, stalling the I2C controller. QSFP loopback is likely not plugged in. qsfp_i2c_top was reflashed and qsfp power turned off.

## Log
```TCL
set_property PROBES.FILE {/home/bmn4/group_05_project/bitstreams/hft06_top.ltx} [get_hw_devices xcvu2p_0]
set_property FULL_PROBES.FILE {/home/bmn4/group_05_project/bitstreams/hft06_top.ltx} [get_hw_devices xcvu2p_0]
set_property PROGRAM.FILE {/home/bmn4/group_05_project/bitstreams/hft06_top.bit} [get_hw_devices xcvu2p_0]
program_hw_devices [get_hw_devices xcvu2p_0]
INFO: [Labtools 27-3164] End of startup status: HIGH
program_hw_devices: Time (s): cpu = 00:00:17 ; elapsed = 00:00:08 . Memory (MB): peak = 8977.508 ; gain = 0.000 ; free physical = 104506 ; free virtual = 111835
refresh_hw_device [lindex [get_hw_devices xcvu2p_0] 0]
INFO: [Labtools 27-2302] Device xcvu2p (JTAG device index = 0) is programmed with a design that has 1 ILA core(s).
add_wave -into {hw_ila_data_1.wcfg} -radix hex { {counter} }
run_hw_ila [get_hw_ilas -of_objects [get_hw_devices xcvu2p_0] -filter {CELL_NAME=~"il0"}] -trigger_now
INFO: [Labtools 27-1964] The ILA core 'hw_ila_1' trigger was armed at 2026-Apr-28 18:49:40
wait_on_hw_ila [get_hw_ilas -of_objects [get_hw_devices xcvu2p_0] -filter {CELL_NAME=~"il0"}]
display_hw_ila_data [upload_hw_ila_data [get_hw_ilas -of_objects [get_hw_devices xcvu2p_0] -filter {CELL_NAME=~"il0"}]]
INFO: [Labtools 27-1966] The ILA core 'hw_ila_1' triggered at 2026-Apr-28 18:49:40
INFO: [Labtools 27-3304] ILA Waveform data saved to file /home/bmn4/group_05_project/.Xil/Vivado-1616472-xrdptest/backup/hw_ila_data_1.ila. Use Tcl command 'read_hw_ila_data' or Vivado File->Import->Import ILA Data menu item to import the previously saved data.
set_property PROBES.FILE {/home/bmn4/group_05_project/bitstreams/hft06_qsfp_i2c_top.ltx} [get_hw_devices xcvu2p_0]
set_property FULL_PROBES.FILE {/home/bmn4/group_05_project/bitstreams/hft06_qsfp_i2c_top.ltx} [get_hw_devices xcvu2p_0]
set_property PROGRAM.FILE {/home/bmn4/group_05_project/bitstreams/hft06_qsfp_i2c_top.bit} [get_hw_devices xcvu2p_0]
program_hw_devices [get_hw_devices xcvu2p_0]
INFO: [Labtools 27-3164] End of startup status: HIGH
program_hw_devices: Time (s): cpu = 00:00:15 ; elapsed = 00:00:09 . Memory (MB): peak = 8991.707 ; gain = 0.000 ; free physical = 104501 ; free virtual = 111826
refresh_hw_device [lindex [get_hw_devices xcvu2p_0] 0]
INFO: [Labtools 27-2302] Device xcvu2p (JTAG device index = 0) is programmed with a design that has 1 ILA core(s).
INFO: [Labtools 27-2302] Device xcvu2p (JTAG device index = 0) is programmed with a design that has 1 JTAG AXI core(s).
add_wave -into {hw_ila_data_1.wcfg} -radix hex { {IO_ADDR_ADDR} {IO_CONTROL_CMPLT} {IO_CONTROL_ID} {IO_CONTROL_PULSE} {scl_i} {scl_o} {scl_t} {sda_i} {sda_o} {sda_t} }
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
[i2c_rd] 0x40 0x00 = 0x00000016
0x00000016
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
[i2c_rd] 0x40 0x00 = 0x00000016
0x00000016
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
qsfp_access_i2c 1

Example QSFP 1 MODULE I2C Access...
-- Select Gate Mux's to QSFP 1 I2C
[i2c_wr] 0xE0 0x08 = 0x08

-- Read Module Identifier --
[i2c_rd] 0xA0 0x00 = 0x00000020
0x00000020
i2c_rd 0xA0 0x7F
[i2c_rd] 0xA0 0x7F = 0x00000020
0x00000020
reg_rd 0x0
0x800000a0
set_property PROBES.FILE {/home/bmn4/group_05_project/bitstreams/hft06_qsfp_i2c_top.ltx} [get_hw_devices xcvu2p_0]
set_property FULL_PROBES.FILE {/home/bmn4/group_05_project/bitstreams/hft06_qsfp_i2c_top.ltx} [get_hw_devices xcvu2p_0]
set_property PROGRAM.FILE {/home/bmn4/group_05_project/bitstreams/hft06_qsfp_i2c_top.bit} [get_hw_devices xcvu2p_0]
program_hw_devices [get_hw_devices xcvu2p_0]
INFO: [Labtools 27-3164] End of startup status: HIGH
program_hw_devices: Time (s): cpu = 00:00:14 ; elapsed = 00:00:09 . Memory (MB): peak = 8991.707 ; gain = 0.000 ; free physical = 104398 ; free virtual = 111725
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
[i2c_rd] 0x40 0x00 = 0x00000016
0x00000016
disable_qsfp_power
-- Disable Power to QSFP 1-2  (set output value and output enable)
[i2c_rd] 0x42 0x0 = 0x000000be
[i2c_wr] 0x42 0x1 = 0x00
[i2c_wr] 0x42 0x3 = 0x55
[i2c_rd] 0x42 0x0 = 0x00000000
0x00000000
```