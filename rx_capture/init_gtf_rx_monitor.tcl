open_hw_manager
connect_hw_server
current_hw_target [get_hw_targets *JWA]
open_hw_target
refresh_hw_target
set device [get_hw_devices xcvu2p_0]
current_hw_device $device
refresh_hw_device $device

# Flash FPGA
set_property PROGRAM.FILE "$::env(HOME)/group_05_project/loopback_example/gtfwizard_0_gtfmac_ex.bit" $device
set_property PROBES.FILE "$::env(HOME)/group_05_project/loopback_example/gtfwizard_0_gtfmac_ex.ltx" $device
program_hw_devices $device
puts "Device programmed."

# Re-refresh after programming to pick up JTAG-AXI + ILA cores
refresh_hw_device $device
set jaxi [get_hw_axis]

# Helper procs
proc axi_read {jaxi addr} {
    create_hw_axi_txn rd_txn [lindex $jaxi 0] -type READ -address [format "%08x" $addr] -force
    run_hw_axi rd_txn
    set raw [get_property DATA [get_hw_axi_txns rd_txn]]
    scan $raw %x val
    return $val
}

proc axi_write {jaxi addr data} {
    create_hw_axi_txn wr_txn [lindex $jaxi 0] -type WRITE -address [format "%08x" $addr] -data [format "%08x" $data] -force
    run_hw_axi wr_txn
}

proc axi_poll {jaxi addr mask expected {max_attempts 100000} {delay_ms 1}} {
    for {set i 0} {$i < $max_attempts} {incr i} {
        set val [axi_read $jaxi $addr]
        if {($val & $mask) == $expected} { return [list $val $i] }
        after $delay_ms
    }
    error "axi_poll timeout: addr=[format 0x%08x $addr] last=[format 0x%08x $val]"
}

# === Step 1: Wait for DUT alive ===
axi_poll $jaxi 0x00000 0x3 0x0
puts "DUT alive."

# === Step 2: Report userrdy ===
axi_write $jaxi 0x0000C 0x00000003

# === Step 3: Disable loopback (bits[6:4] = 000) ===
set lb [axi_read $jaxi 0x10408]
axi_write $jaxi 0x10408 [expr {$lb & 0xFFFFFF8F}]
puts "Loopback disabled."

# === Step 4: RX reset cycle ===
axi_write $jaxi 0x10400 0x00000002
after 100
axi_write $jaxi 0x10400 0x00000000
after 500

# === Step 5: GTFMAC data rate (10G) ===
axi_write $jaxi 0x10000 0x00000000

# === Step 6: Allow bitslip ===
axi_write $jaxi 0x000A4 0x00000000

# === Step 7: Wait for block lock ===
puts "Waiting for block lock..."
axi_poll $jaxi 0x000A0 0x00010000 0x00010000
puts "Block lock acquired."

# === Step 8: Trigger bitslip correction ===
axi_write $jaxi 0x000A4 0x00000001

# === Step 9: Wait for bitslip done ===
axi_poll $jaxi 0x000A0 0x00040000 0x00040000 100 10

# === Step 10: Wait for block lock re-acquire ===
axi_poll $jaxi 0x000A0 0x00010000 0x00010000
puts "Block lock re-acquired after bitslip."

# === Step 11: Wait for RX buffer bypass ===
after 5000

# === Step 12: Wait for link up ===
puts "Waiting for link up..."
axi_poll $jaxi 0x00000 0x00000F00 0x00000000 10000 1
puts "Link is up."

# === Step 13: RX config ===
set rxcfg [axi_read $jaxi 0x10008]
set rxcfg [expr {$rxcfg & 0xFFFFFF9B}]
axi_write $jaxi 0x10008 $rxcfg

# === Step 14: Arm ILA RX trigger on rx_axis_tvalid ===
puts "Arming ILA RX trigger on rx_axis_tvalid..."
set ila_rx [get_hw_ilas -filter {CELL_NAME=~"*ila_rx*"}]
set_property CONTROL.TRIGGER_POSITION 0 $ila_rx
set_property TRIGGER_COMPARE_VALUE eq1'b1 [get_hw_probes */probe1 -of $ila_rx]
run_hw_ila $ila_rx
puts "ILA armed. Waiting for trigger..."
puts ">>> Run udp_broadcast.sh on the V80 now <<<"

# === Step 15: Wait for ILA to trigger ===
wait_on_hw_ila $ila_rx
puts "ILA triggered! Data captured."

# === Step 16: Upload and export captured data ===
set ila_data [upload_hw_ila_data $ila_rx]
display_hw_ila_data $ila_data

# === Step 17: Write VCD ===
write_hw_ila_data -vcd_file "$::env(HOME)/group_05_project/rx_capture/ila_rx_capture.vcd" $ila_data
puts "VCD written to ila_rx_capture.vcd"
puts "--- DONE ---"
