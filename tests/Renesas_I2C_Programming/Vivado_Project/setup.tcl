# 
# Copyright (C) 2024, Advanced Micro Devices, Inc. All rights reserved.
# SPDX-License-Identifier: X11
# 
#

# Typical usage: source ./setup.tcl

# ------------------------------------------------------
#
# Setup Environment/Project
#
# ------------------------------------------------------

# Create the project and directory structure
set _script_dir_ [eval pwd]

# Set the reference directory to where repo is
set _origin_dir_ [file dirname ${_script_dir_}]

set _proj_name_ project_2
set _part_name_ xcvu2p-fsvj2104-3-e
set _proj_path_ ${_script_dir_}/${_proj_name_}

# Remove project folder if already exists...
if { [file exist ${_origin_dir_}/Vivado_project/${_proj_name_}] == 1 } {
    file delete ${_origin_dir_}/Vivado_project/${_proj_name_}
}

create_project -force ${_proj_name_} ${_proj_path_} -part ${_part_name_}

# ------------------------------------------------------
#
# Add Source Files...
#
# ------------------------------------------------------

# add_files -norecurse -scan_for_includes ${_origin_dir_}/RTL/sequence/seq_dbg.coe
add_files -norecurse -scan_for_includes ${_origin_dir_}/Scripts/example_fixed/fixed.coe
add_files -norecurse -scan_for_includes ${_origin_dir_}/RTL/axi_master.v
add_files -norecurse -scan_for_includes ${_origin_dir_}/RTL/i2c_sequencer.sv
add_files -norecurse -scan_for_includes ${_origin_dir_}/RTL/i2c_axi_sequencer.v
add_files -norecurse -scan_for_includes ${_origin_dir_}/RTL/reg_i2c_user_logic.v
add_files -norecurse -scan_for_includes ${_origin_dir_}/RTL/renesas_i2c_top.v
add_files -norecurse -scan_for_includes ${_origin_dir_}/RTL/freq_counter/syncer_reset.sv
add_files -norecurse -scan_for_includes ${_origin_dir_}/RTL/freq_counter/syncer_level.sv
add_files -norecurse -scan_for_includes ${_origin_dir_}/RTL/freq_counter/freq_counter.v
add_files -norecurse -scan_for_includes ${_origin_dir_}/RTL/freq_counter/freq_counter_regs.v
add_files -norecurse -scan_for_includes ${_origin_dir_}/RTL/freq_counter/freq_counter_top.v
add_files -norecurse -scan_for_includes ${_origin_dir_}/RTL/system/system_gtf_clk_buffer.v
add_files -norecurse -scan_for_includes ${_origin_dir_}/RTL/system/reg_axi_slave.v

# ------------------------------------------------------
#
# Add Simulation Files...
#
# ------------------------------------------------------
#add_files -fileset sim_1 ${_origin_dir_}/Sim/

set_property SOURCE_SET sources_1 [get_filesets sim_1]
add_files -fileset sim_1 -norecurse ${_origin_dir_}/Sim/i2c_slave_if.v
add_files -fileset sim_1 -norecurse ${_origin_dir_}/Sim/RC38612A002GN2.v
add_files -fileset sim_1 -norecurse ${_origin_dir_}/Sim/sim_tb.sv
add_files -fileset sim_1 -norecurse ${_origin_dir_}/Sim/sim_top_behav.wcfg

update_compile_order -fileset sim_1

# ------------------------------------------------------
#
# Add Constraint Files...
#
# ------------------------------------------------------
#add_files -fileset constrs_1 -norecurse ${_origin_dir_}/XDC/const_timing.xdc
add_files -fileset constrs_1 -norecurse ${_origin_dir_}/XDC/ul3422.xdc
#add_files -fileset constrs_1 -norecurse ${_origin_dir_}/XDC/ul3524.xdc

# ------------------------------------------------------
#
# Add IP Sources...
#
# ------------------------------------------------------

source ${_origin_dir_}/IP/axi_iic_0_ip.tcl
source ${_origin_dir_}/IP/blk_mem_gen_0_ip.tcl
source ${_origin_dir_}/IP/clk_wiz_0_ip.tcl
source ${_origin_dir_}/IP/ila_0_ip.tcl
source ${_origin_dir_}/IP/vio_0_ip.tcl

# JTAG AXI IP for register access
create_ip -name jtag_axi -vendor xilinx.com -library ip -version 1.2 -module_name jtag_axi_0
set_property CONFIG.PROTOCOL {2} [get_ips jtag_axi_0]


# ------------------------------------------------------
#
# Misc. Project Settings....
#
# ------------------------------------------------------

#
# Project uses the full UL3422 XDC.  Unused constraints will generate Critical Warnings.
# These command will suppress the following messages...

# -- 'set_property' expects at least one object.
set_msg_config -suppress -id {Common 17-55}

# -- command failed: can't read "variable": no such variable (reported during synthesis)
set_msg_config -suppress -id {Common 17-1548}

#
# Finish Environment Setup...
#

#set_property STEPS.PLACE_DESIGN.ARGS.DIRECTIVE ExtraNetDelay_low [get_runs impl_1]

set_property top  renesas_i2c_top  [get_filesets sources_1]
set_property top  sim_tb_top       [get_filesets sim_1]
update_compile_order -fileset sources_1
update_compile_order -fileset sim_1

puts {Setup complete...}

