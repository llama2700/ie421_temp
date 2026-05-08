#
# 1_flash_bitstream.tcl
#
# Programs the FPGA with the Renesas I2C + Freq Counter bitstream.
# Run from Vivado Tcl console after opening Hardware Manager.
#

# Connect to hardware
open_hw_manager
connect_hw_server
current_hw_target [get_hw_targets *JWA]
open_hw_target
refresh_hw_target

# Find the device
set device [get_hw_devices xcvu2p_0]
current_hw_device $device
refresh_hw_device $device

# Set the bitstream path (adjust if your project path differs)
set bit_file [glob $::env(HOME)/group_05_project/tests/Renesas_I2C_Programming/Vivado_Project/project_3/project_3.runs/impl_1/*.bit]

if {$bit_file eq ""} {
    puts "ERROR: No .bit file found in impl_1 directory."
    puts "       Check your project path or build status."
    return
}

set ltx_file [glob $::env(HOME)/group_05_project/tests/Renesas_I2C_Programming/Vivado_Project/project_3/project_3.runs/impl_1/*.ltx]

set_property PROGRAM.FILE $bit_file $device
set_property PROBES.FILE $ltx_file $device
program_hw_devices $device
refresh_hw_device $device

puts ""
puts "Bitstream programmed successfully: $bit_file"
puts "Next: source 2_read_freq_counters.tcl"
