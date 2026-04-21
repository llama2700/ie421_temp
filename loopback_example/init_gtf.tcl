open_hw_manager
connect_hw_server
current_hw_target [get_hw_targets *JWA]
open_hw_target
refresh_hw_target
set device [get_hw_devices xcvu2p_0]
current_hw_device $device
refresh_hw_device $device

# Flash FPGA
set_property PROGRAM.FILE {/home/akshim2/gtfwizard_0_gtfmac_ex.bit} $device
set_property PROBES.FILE {/home/akshim2/gtfwizard_0_gtfmac_ex.ltx} $device
program_hw_devices $device
puts "Device programmed."

# Re-refresh after programming to pick up JTAG-AXI core
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

# === Step 2: Report userrdy ===
axi_write $jaxi 0x0000C 0x00000003

# === Step 3: Near-end loopback (bits[6:4] = 010) ===
set lb [axi_read $jaxi 0x10408]
axi_write $jaxi 0x10408 [expr {($lb & 0xFFFFFF8F) | 0x00000020}]

# === Step 4-5: RX reset cycle ===
axi_write $jaxi 0x10400 0x00000002
after 100
axi_write $jaxi 0x10400 0x00000000
after 500

# === Step 6: GTFMAC data rate (10G) ===
axi_write $jaxi 0x10000 0x00000000

# === Step 7: HWCHK data rate ===
set hwcfg [axi_read $jaxi 0x00010]
set hwcfg [expr {$hwcfg | (0 << 0) | (0 << 16)}]
axi_write $jaxi 0x00010 $hwcfg

# === Step 8: Allow bitslip ===
axi_write $jaxi 0x000A4 0x00000000

# === Step 9: Wait for block lock ===
axi_poll $jaxi 0x000A0 0x00010000 0x00010000

# === Step 10: Trigger bitslip correction ===
axi_write $jaxi 0x000A4 0x00000001

# === Step 11: Wait for bitslip done ===
axi_poll $jaxi 0x000A0 0x00040000 0x00040000 100 10

# === Step 12: Wait for block lock re-acquire ===
axi_poll $jaxi 0x000A0 0x00010000 0x00010000

# === Step 12b: Wait for RX buffer bypass ===
after 5000

# === Step 13: Wait for link up (status bits[11:8] == 0) ===
axi_poll $jaxi 0x00000 0x00000F00 0x00000000 10000 1

# === Step 14: TX config ===
set txcfg [axi_read $jaxi 0x10004]
set txcfg [expr {($txcfg & 0xFFFFE0F1) | (1 << 1) | (8 << 8)}]
axi_write $jaxi 0x10004 $txcfg

# === Step 15: RX config ===
set rxcfg [axi_read $jaxi 0x10008]
set rxcfg [expr {$rxcfg & 0xFFFFFF9B}]
axi_write $jaxi 0x10008 $rxcfg

# === Step 16-17: MAC min/max packet len ===
axi_write $jaxi 0x1000C 0x00000040
axi_write $jaxi 0x10010 0x000005DC

# === Step 18: HWCHK config (fcs_ins_enable) ===
set hwcfg [axi_read $jaxi 0x00010]
set hwcfg [expr {$hwcfg | (1 << 4)}]
axi_write $jaxi 0x00010 $hwcfg

# === Step 19-20: No error/poison injection ===
axi_write $jaxi 0x00040 0x00000000
axi_write $jaxi 0x00098 0x00000000

# === Step 21-22: HWCHK min/max frame len ===
axi_write $jaxi 0x00028 0x00000040
axi_write $jaxi 0x00024 0x000005DC

# === Step 23: Gen mode (random) ===
axi_write $jaxi 0x00014 0x00000000

# === Step 24: Frames to send ===
axi_write $jaxi 0x0002C 0x00000032

# === Step 25-26: Init stats ===
axi_write $jaxi 0x00090 0x00000001
axi_write $jaxi 0x1040C 0x00000001
after 100

# === Step 27: START generator + monitor ===
axi_write $jaxi 0x00020 0x00000011
after 5000

# === Step 28: STOP generator ===
axi_write $jaxi 0x00020 0x00000000
after 1000

# === Step 29-30: Tick stats for collection ===
axi_write $jaxi 0x00090 0x00000001
axi_write $jaxi 0x1040C 0x00000001
after 500

# === Results ===
set tx_lo [axi_read $jaxi 0x10700]
set rx_lo [axi_read $jaxi 0x10808]
puts ""
puts "TX packets: $tx_lo"
puts "RX packets: $rx_lo"
if {$tx_lo == $rx_lo && $tx_lo == 50} {
    puts "PASS: TX == RX == 50"
} else {
    puts "FAIL: TX=$tx_lo RX=$rx_lo (expected 50)"
}
puts "--- DONE ---"
