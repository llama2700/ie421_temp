#
# Copyright (c) 2023, Advanced Micro Devices, Inc. All rights reserved.
# SPDX-License-Identifier: MIT
#

# Typical usage: source ./setup.tcl
#
# Pre-requisite: the GTF wizard example design sources must already be present
# under ../RTL/gtfwizard_mac_ex/ and ../RTL/gtfwizard_raw_ex/. See
# ../RTL/gtfwizard_mac_ex/README.md for the regeneration steps.

set _script_dir_ [eval pwd]
set _origin_dir_ [file dirname ${_script_dir_}]

set _proj_name_ project_1
set _part_name_ xcvu2p-fsvj2104-3-e
set _proj_path_ ${_script_dir_}/${_proj_name_}

if { [file exist ${_origin_dir_}/Vivado_project/${_proj_name_}] == 1 } {
    file delete ${_origin_dir_}/Vivado_project/${_proj_name_}
}

create_project -force ${_proj_name_} ${_proj_path_} -part ${_part_name_}

# -- Design Files
add_files -fileset sources_1 ${_origin_dir_}/RTL/sync/syncer_level.sv
add_files -fileset sources_1 ${_origin_dir_}/RTL/sync/syncer_pulse.sv
add_files -fileset sources_1 ${_origin_dir_}/RTL/sync/syncer_bus.v
add_files -fileset sources_1 ${_origin_dir_}/RTL/sync/syncer_reset.sv

add_files -fileset sources_1 ${_origin_dir_}/RTL/freq_counter/freq_counter_regs.v
add_files -fileset sources_1 ${_origin_dir_}/RTL/freq_counter/freq_counter.v
add_files -fileset sources_1 ${_origin_dir_}/RTL/freq_counter/freq_counter_top.v

add_files -fileset sources_1 ${_origin_dir_}/RTL/renesas_gpio/renesas_gpio_regs.v
add_files -fileset sources_1 ${_origin_dir_}/RTL/renesas_gpio/renesas_gpio.v

add_files -fileset sources_1 ${_origin_dir_}/RTL/renesas_i2c/renesas_bram.v
add_files -fileset sources_1 ${_origin_dir_}/RTL/renesas_i2c/renesas_i2c_regs.v
add_files -fileset sources_1 ${_origin_dir_}/RTL/renesas_i2c/renesas_i2c_sequencer.sv
add_files -fileset sources_1 ${_origin_dir_}/RTL/renesas_i2c/renesas_i2c_axi_master.v
add_files -fileset sources_1 ${_origin_dir_}/RTL/renesas_i2c/renesas_i2c_top.v

add_files -fileset sources_1 ${_origin_dir_}/RTL/qsfp_i2c/RTL/state_machine/state_machine_pwr.v
add_files -fileset sources_1 ${_origin_dir_}/RTL/qsfp_i2c/RTL/state_machine/state_machine_top.v
add_files -fileset sources_1 ${_origin_dir_}/RTL/qsfp_i2c/RTL/state_machine/state_machine_sb.v
add_files -fileset sources_1 ${_origin_dir_}/RTL/qsfp_i2c/RTL/qsfp_axi_master.v
add_files -fileset sources_1 ${_origin_dir_}/RTL/qsfp_i2c/RTL/qsfp_i2c_axi_sequencer.v
add_files -fileset sources_1 ${_origin_dir_}/RTL/qsfp_i2c/RTL/qsfp_i2c_regs.v
add_files -fileset sources_1 ${_origin_dir_}/RTL/qsfp_i2c/RTL/qsfp_i2c_top.v

add_files -fileset sources_1 ${_origin_dir_}/RTL/system/system_regs.v
add_files -fileset sources_1 ${_origin_dir_}/RTL/system/system_gtf_clk_buffer.v
add_files -fileset sources_1 ${_origin_dir_}/RTL/system/sys_if_switch.v
add_files -fileset sources_1 ${_origin_dir_}/RTL/system/reg_axi_slave.v
add_files -fileset sources_1 ${_origin_dir_}/RTL/system/clk_reset.v
add_files -fileset sources_1 ${_origin_dir_}/RTL/clk_recov.v

add_files -fileset sources_1 ${_origin_dir_}/RTL/gtf/gtf_top_0.v
add_files -fileset sources_1 ${_origin_dir_}/RTL/gtf/gtf_top_1.v
add_files -fileset sources_1 ${_origin_dir_}/RTL/gtf/gtf_top_2.v

# GTF wizard example design sources -- regenerate per RTL/gtfwizard_*_ex/README.md
foreach _f [glob -nocomplain ${_origin_dir_}/RTL/gtfwizard_mac_ex/imports/*.{v,sv,vh}] {
    add_files -fileset sources_1 $_f
}
foreach _f [glob -nocomplain ${_origin_dir_}/RTL/gtfwizard_mac_ex/gtfwizard_mac/*.{v,sv,vh}] {
    add_files -fileset sources_1 $_f
}
foreach _f [glob -nocomplain ${_origin_dir_}/RTL/gtfwizard_raw_ex/imports/*.{v,sv,vh}] {
    add_files -fileset sources_1 $_f
}

if { [file exist ${_origin_dir_}/RTL/gtfwizard_raw_ex/ip/gtfwizard_raw/gtfwizard_raw.xci] } {
    import_files -norecurse ${_origin_dir_}/RTL/gtfwizard_raw_ex/ip/gtfwizard_raw/gtfwizard_raw.xci
}
if { [file exist ${_origin_dir_}/RTL/gtfwizard_mac_ex/ip/gtfwizard_mac_axi_crossbar_inst/gtfwizard_mac_axi_crossbar_inst.xci] } {
    import_files -norecurse ${_origin_dir_}/RTL/gtfwizard_mac_ex/ip/gtfwizard_mac_axi_crossbar_inst/gtfwizard_mac_axi_crossbar_inst.xci
}
if { [file exist ${_origin_dir_}/RTL/gtfwizard_mac_ex/ip/gtfwizard_mac_example_axil_ctrl.tcl] } {
    source ${_origin_dir_}/RTL/gtfwizard_mac_ex/ip/gtfwizard_mac_example_axil_ctrl.tcl
}
if { [file exist ${_origin_dir_}/RTL/gtfwizard_mac_ex/ip/gtfwizard_mac_example_axil_ctrl.tcl] } {
    source ${_origin_dir_}/RTL/gtfwizard_mac_ex/ip/gtfwizard_mac_example_axil_ctrl.tcl
}
if { [file exist ${_origin_dir_}/RTL/gtfwizard_mac_ex/ip/fifo_generator_0_ip.tcl] } {
    source ${_origin_dir_}/RTL/gtfwizard_mac_ex/ip/fifo_generator_0_ip.tcl
}
if { [file exist ${_origin_dir_}/RTL/gtfwizard_mac_ex/ip/mac_ila_0_ip.tcl] } {
    source ${_origin_dir_}/RTL/gtfwizard_mac_ex/ip/mac_ila_0_ip.tcl
}

update_compile_order -fileset sources_1


# -- Simulation Files
add_files -fileset sim_1 ${_origin_dir_}/sim/RTL/qsfp_i2c/pca9545a.v
add_files -fileset sim_1 ${_origin_dir_}/sim/RTL/qsfp_i2c/tca6406a.v
add_files -fileset sim_1 ${_origin_dir_}/sim/RTL/qsfp_i2c/qsfp_i2c.vh

add_files -fileset sim_1 ${_origin_dir_}/sim/RTL/renesas_i2c/RC38612A002GN2.v
add_files -fileset sim_1 ${_origin_dir_}/sim/RTL/renesas_i2c/renesas_i2c.vh

add_files -fileset sim_1 ${_origin_dir_}/sim/RTL/system/sim_i2c_slave_if.v
add_files -fileset sim_1 ${_origin_dir_}/sim/RTL/system/sim_axi_master_tasks.vh
add_files -fileset sim_1 ${_origin_dir_}/sim/RTL/system/sim_tb_addr.v
add_files -fileset sim_1 ${_origin_dir_}/sim/RTL/system/sim_axi_master.v
add_files -fileset sim_1 ${_origin_dir_}/sim/RTL/system/sim_axi_monitor.v

add_files -fileset sim_1 ${_origin_dir_}/sim/RTL/sim_tb.v
add_files -fileset sim_1 ${_origin_dir_}/sim/RTL/qsfp_i2c/i2c_slave_if.v

add_files -fileset sim_1 -norecurse ${_origin_dir_}/sim/sim_tb_behav.wcfg
set_property xsim.view ${_origin_dir_}/sim/sim_tb_behav.wcfg [get_filesets sim_1]

set_property file_type SystemVerilog [get_files ${_origin_dir_}/sim/RTL/sim_tb.v]


# -- Constraint Files
add_files -fileset constrs_1 -norecurse ${_origin_dir_}/XDC/constraint.xdc
add_files -fileset constrs_1 -norecurse ${_origin_dir_}/XDC/const_timing.xdc


# Generate AXI Subsystem
source ${_origin_dir_}/RTL/system/axi_subsys.tcl


# -- IP cores

# JTAG to AXI Master
create_ip -name jtag_axi -vendor xilinx.com -library ip -version 1.2 -module_name jtag_axi_0
set_property CONFIG.PROTOCOL {2} [get_ips jtag_axi_0]

# System clock from clk_ddr_lvds_300 (300 MHz LVDS in -> 100/50 MHz)
create_ip -name clk_wiz -vendor xilinx.com -library ip -version 6.0 -module_name clk_wiz_0
set_property -dict [list \
  CONFIG.CLKIN1_JITTER_PS {33.330000000000005} \
  CONFIG.CLKOUT1_JITTER {101.475} \
  CONFIG.CLKOUT1_PHASE_ERROR {77.836} \
  CONFIG.CLKOUT2_JITTER {116.415} \
  CONFIG.CLKOUT2_PHASE_ERROR {77.836} \
  CONFIG.CLKOUT2_REQUESTED_OUT_FREQ {50} \
  CONFIG.CLKOUT2_USED {true} \
  CONFIG.MMCM_CLKFBOUT_MULT_F {4.000} \
  CONFIG.MMCM_CLKOUT1_DIVIDE {24} \
  CONFIG.MMCM_CLKIN1_PERIOD {3.333} \
  CONFIG.MMCM_CLKIN2_PERIOD {10.0} \
  CONFIG.PRIM_IN_FREQ {300.000} \
  CONFIG.USE_RESET {false} \
  CONFIG.NUM_OUT_CLKS {2} \
] [get_ips clk_wiz_0]
set_property CONFIG.PRIM_SOURCE {Differential_clock_capable_pin} [get_ips clk_wiz_0]

# GTF freerun clock from clk_sys_lvds_300 (300 MHz -> 200/425 MHz)
create_ip -name clk_wiz -vendor xilinx.com -library ip -version 6.0 -module_name gtfwizard_0_example_clk_wiz
set_property -dict [list \
  CONFIG.CLKIN1_JITTER_PS {33.330000000000005} \
  CONFIG.CLKOUT1_DRIVES {Buffer} \
  CONFIG.CLKOUT1_JITTER {87.024} \
  CONFIG.CLKOUT1_PHASE_ERROR {75.422} \
  CONFIG.CLKOUT1_REQUESTED_OUT_FREQ {200.000} \
  CONFIG.CLKOUT2_DRIVES {Buffer} \
  CONFIG.CLKOUT2_JITTER {75.082} \
  CONFIG.CLKOUT2_PHASE_ERROR {75.422} \
  CONFIG.CLKOUT2_REQUESTED_OUT_FREQ {425.000} \
  CONFIG.CLKOUT2_USED {true} \
  CONFIG.CLKOUT3_DRIVES {Buffer} \
  CONFIG.CLKOUT4_DRIVES {Buffer} \
  CONFIG.CLKOUT5_DRIVES {Buffer} \
  CONFIG.CLKOUT6_DRIVES {Buffer} \
  CONFIG.CLKOUT7_DRIVES {Buffer} \
  CONFIG.MMCM_CLKFBOUT_MULT_F {4.250} \
  CONFIG.MMCM_CLKIN1_PERIOD {3.333} \
  CONFIG.MMCM_CLKIN2_PERIOD {10.0} \
  CONFIG.MMCM_CLKOUT0_DIVIDE_F {6.375} \
  CONFIG.MMCM_CLKOUT1_DIVIDE {3} \
  CONFIG.NUM_OUT_CLKS {2} \
  CONFIG.PRIM_IN_FREQ {300.000} \
  CONFIG.PRIM_SOURCE {Global_buffer} \
  CONFIG.SECONDARY_SOURCE {Single_ended_clock_capable_pin} \
  CONFIG.USE_PHASE_ALIGNMENT {true} \
] [get_ips gtfwizard_0_example_clk_wiz]

# Dummy secondary 33 MHz clock used by the wizard example
create_ip -name clk_wiz -vendor xilinx.com -library ip -version 6.0 -module_name clk_wiz_1
set_property -dict [list \
  CONFIG.CLKOUT1_JITTER {252.007} \
  CONFIG.CLKOUT1_PHASE_ERROR {354.739} \
  CONFIG.CLKOUT1_REQUESTED_OUT_FREQ {33.000} \
  CONFIG.MMCM_CLKFBOUT_MULT_F {119.625} \
  CONFIG.MMCM_CLKOUT0_DIVIDE_F {36.250} \
  CONFIG.MMCM_DIVCLK_DIVIDE {10} \
  CONFIG.PRIM_SOURCE {No_buffer} \
  CONFIG.USE_RESET {true} \
] [get_ips clk_wiz_1]

# Dual port BRAM for jitter cleaner I2C sequence storage
create_ip -name blk_mem_gen -vendor xilinx.com -library ip -version 8.4 -module_name blk_mem_gen_0
set_property -dict [list \
  CONFIG.Enable_A {Always_Enabled} \
  CONFIG.Enable_B {Always_Enabled} \
  CONFIG.Memory_Type {True_Dual_Port_RAM} \
  CONFIG.Write_Depth_A {16384} \
  CONFIG.Write_Width_A {32} \
  CONFIG.Write_Width_B {16} \
] [get_ips blk_mem_gen_0]

# I2C controllers
create_ip -name axi_iic -vendor xilinx.com -library ip -version 2.1 -module_name axi_iic_0
set_property -dict [list CONFIG.AXI_ACLK_FREQ_MHZ {50}  ] [get_ips axi_iic_0]

create_ip -name axi_iic -vendor xilinx.com -library ip -version 2.1 -module_name axi_iic_qsfp
set_property -dict [list CONFIG.AXI_ACLK_FREQ_MHZ {100} ] [get_ips axi_iic_qsfp]

# RAW GTF loopback FIFO
create_ip -name fifo_generator -vendor xilinx.com -library ip -version 13.2 -module_name fifo_raw_loopback
set_property -dict [list \
  CONFIG.Fifo_Implementation {Independent_Clocks_Block_RAM} \
  CONFIG.Input_Data_Width {90} \
  CONFIG.Input_Depth {256} \
  CONFIG.Overflow_Flag {true} \
  CONFIG.Underflow_Flag {true} \
  CONFIG.Use_Embedded_Registers {false} \
  CONFIG.Valid_Flag {true} \
  CONFIG.Write_Acknowledge_Flag {true} \
] [get_ips fifo_raw_loopback]

create_ip -name fifo_generator -vendor xilinx.com -library ip -version 13.2 -module_name fifo_mac_data_sync
set_property -dict [list \
  CONFIG.Fifo_Implementation {Independent_Clocks_Block_RAM} \
  CONFIG.Input_Data_Width {70} \
  CONFIG.Input_Depth {4096} \
  CONFIG.Use_Embedded_Registers {false} \
] [get_ips fifo_mac_data_sync]

# ILA for MAC data compare debug
create_ip -name ila -vendor xilinx.com -library ip -version 6.2 -module_name ila_mac_fifo
set_property -dict [list \
  CONFIG.C_DATA_DEPTH {16384} \
  CONFIG.C_INPUT_PIPE_STAGES {2} \
  CONFIG.C_NUM_OF_PROBES {13} \
  CONFIG.C_PROBE11_TYPE {0} \
  CONFIG.C_PROBE11_WIDTH {3} \
  CONFIG.C_PROBE12_TYPE {1} \
  CONFIG.C_PROBE12_WIDTH {64} \
  CONFIG.C_PROBE2_WIDTH {32} \
  CONFIG.C_PROBE7_WIDTH {3} \
  CONFIG.C_PROBE8_TYPE {1} \
  CONFIG.C_PROBE8_WIDTH {64} \
  CONFIG.Component_Name {ila_mac_fifo} \
] [get_ips ila_mac_fifo]


# Set top
set_property top clk_recov [current_fileset]
set_property top sim_tb    [get_filesets sim_1]

update_compile_order -fileset sources_1
update_compile_order -fileset sim_1

set_property STEPS.PLACE_DESIGN.ARGS.DIRECTIVE ExtraNetDelay_low [get_runs impl_1]

puts {Setup complete...}
puts {NOTE: regenerate the GTF wizard example designs (gtfwizard_mac, gtfwizard_raw)}
puts {      per RTL/gtfwizard_mac_ex/README.md before launching synthesis.}
