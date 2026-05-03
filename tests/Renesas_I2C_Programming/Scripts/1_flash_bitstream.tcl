#
# 1_flash_bitstream.tcl
#
# Programs the FPGA with the Renesas I2C + Freq Counter bitstream.
# Run from Vivado Tcl console after opening Hardware Manager.
#

# Connect to hardware
open_hw_manager
connect_hw_server -allow_non_jtag
open_hw_target

# Find the device
set device [lindex [get_hw_devices] 0]
current_hw_device $device

# Set the bitstream path (adjust if your project path differs)
set bit_file [glob -nocomplain X:/Renesas_I2C_Programming/Vivado_Project/project_1/project_1.runs/impl_1/*.bit]

if {$bit_file eq ""} {
    puts "ERROR: No .bit file found in impl_1 directory."
    puts "       Check your project path or build status."
    return
}

set_property PROGRAM.FILE $bit_file $device
program_hw_devices $device
refresh_hw_device $device

puts ""
puts "Bitstream programmed successfully: $bit_file"
puts "Next: source 2_read_freq_counters.tcl"
