open_hw_manager
connect_hw_server
current_hw_target [get_hw_targets *JWA]
open_hw_target
refresh_hw_target
set device [get_hw_devices xcvu2p_0]
current_hw_device $device
refresh_hw_device $device
set jaxi [get_hw_axis]
puts "JTAG-AXI: $jaxi"

# Flash FPGA
set_property PROGRAM.FILE "$::env(HOME)/group_05_project/tests/loopback_example/gtfwizard_0_gtfmac_ex.bit" $device
set_property PROBES.FILE "$::env(HOME)/group_05_project/tests/loopback_example/gtfwizard_0_gtfmac_ex.ltx" $device
program_hw_devices $device
puts "Device programmed."

# Re-refresh after programming to pick up JTAG-AXI core
refresh_hw_device $device
set jaxi [get_hw_axis]

# Helper: read a 32-bit register and return its integer value
proc axi_read {jaxi addr} {
    set addr_hex [format "%08x" $addr]
    create_hw_axi_txn rd_txn [lindex $jaxi 0] -type READ -address $addr_hex -force
    run_hw_axi rd_txn
    set raw [get_property DATA [get_hw_axi_txns rd_txn]]
    scan $raw %x val
    return $val
}

# Helper: write a 32-bit register, then read back and verify
proc axi_write {jaxi addr data} {
    set addr_hex [format "%08x" $addr]
    set data_hex [format "%08x" $data]
    create_hw_axi_txn wr_txn [lindex $jaxi 0] -type WRITE -address $addr_hex -data $data_hex -force
    run_hw_axi wr_txn
}

# Helper: write + read-back + print verification
proc axi_write_verify {jaxi addr data label} {
    axi_write $jaxi $addr $data
    set rb [axi_read $jaxi $addr]
    if {$rb != $data} {
        puts "  WARNING: $label readback mismatch! wrote=[format 0x%08x $data] read=[format 0x%08x $rb]"
    } else {
        puts "  OK: $label verified [format 0x%08x $rb]"
    }
    return $rb
}

# Helper: poll a register until (read & mask) == expected, with timeout
proc axi_poll {jaxi addr mask expected {max_attempts 100000} {delay_ms 1}} {
    for {set i 0} {$i < $max_attempts} {incr i} {
        set val [axi_read $jaxi $addr]
        if {(($val & $mask) == $expected)} {
            return [list $val $i]
        }
        after $delay_ms
    }
    error "axi_poll timeout: addr=[format 0x%08x $addr] mask=[format 0x%08x $mask] expected=[format 0x%08x $expected] last=[format 0x%08x $val]"
}

# Helper: dump a register with label
proc axi_dump {jaxi addr label} {
    set val [axi_read $jaxi $addr]
    puts [format "  %-40s \[0x%05x\] = 0x%08x" $label $addr $val]
    return $val
}

puts ""
puts "============================================================"
puts " DIAGNOSTIC REGISTER DUMP - PRE-INIT"
puts "============================================================"
axi_dump $jaxi 0x00000 "HWCHK status"
axi_dump $jaxi 0x00010 "HWCHK config"
axi_dump $jaxi 0x00020 "HWCHK gen/mon enable"
axi_dump $jaxi 0x0002C "HWCHK frames_to_send"
axi_dump $jaxi 0x000A0 "HWCHK block_lock/bitslip"
axi_dump $jaxi 0x000A4 "HWCHK bitslip_ctrl"
axi_dump $jaxi 0x10000 "GTFMAC data_rate"
axi_dump $jaxi 0x10004 "GTFMAC TX config"
axi_dump $jaxi 0x10008 "GTFMAC RX config"
axi_dump $jaxi 0x10400 "GTFMAC GT reset"
axi_dump $jaxi 0x10408 "GTFMAC GT loopback"
puts ""

# ==========================================================================
# 0. RESET - Clear stale state from prior runs
#    Note: Only RX reset (bit 1) is safe via AXI. Bit 0 (TX reset) kills the
#    AXI bus. The full system reset (hb_gtwiz_reset_all_in) is a top-level pin
#    and not AXI-accessible.
# ==========================================================================
puts "=== Step 0: Reset ==="

# Disable any active generator/monitor from a prior run
axi_write $jaxi 0x00020 0x00000000
puts "  gen/mon disabled: [format 0x%08x [axi_read $jaxi 0x00020]]"

# Assert RX reset only (bit 1) - same as testbench step
axi_write $jaxi 0x10400 0x00000002
puts "  GT RX reset asserted: [format 0x%08x [axi_read $jaxi 0x10400]]"
after 500

# Deassert reset
axi_write $jaxi 0x10400 0x00000000
puts "  GT reset deasserted: [format 0x%08x [axi_read $jaxi 0x10400]]"

# Wait for GT RX to re-lock
puts "  Waiting 3 seconds for GT RX to re-lock..."
after 3000

puts ""
puts "============================================================"
puts " DIAGNOSTIC REGISTER DUMP - POST-RESET"
puts "============================================================"
axi_dump $jaxi 0x00000 "HWCHK status"
axi_dump $jaxi 0x00020 "HWCHK gen/mon enable"
axi_dump $jaxi 0x000A0 "HWCHK block_lock/bitslip"
axi_dump $jaxi 0x10400 "GTFMAC GT reset"
axi_dump $jaxi 0x10408 "GTFMAC GT loopback"
puts ""

# ==========================================================================
# 1. Wait for DUT to come alive: poll status reg until bits[1:0] == 0
# ==========================================================================
puts "=== Step 1: Wait for DUT alive ==="
set result [axi_poll $jaxi 0x00000 0x3 0x0]
puts "DUT alive after [lindex $result 1] attempts (status=[format 0x%08x [lindex $result 0]])"

# ==========================================================================
# 2. Report userrdy to the GTF
# ==========================================================================
puts "=== Step 2: userrdy ==="
axi_write_verify $jaxi 0x0000C 0x00000003 "userrdy"

# ==========================================================================
# 3. Configure near-end loopback: bits[6:4] = 3'b010 = 0x20
# ==========================================================================
puts "=== Step 3: Loopback ==="
set lb_val [axi_read $jaxi 0x10408]
puts "  loopback reg before: [format 0x%08x $lb_val]"
set lb_val [expr {($lb_val & 0xFFFFFF8F) | 0x00000020}]
axi_write $jaxi 0x10408 $lb_val
set lb_rb [axi_read $jaxi 0x10408]
puts "  loopback reg after:  [format 0x%08x $lb_rb]"

# ==========================================================================
# 4-5. RX reset assert / deassert
# ==========================================================================
puts "=== Step 4-5: RX reset ==="
axi_write $jaxi 0x10400 0x00000002
puts "  RX reset asserted"
after 100
axi_write $jaxi 0x10400 0x00000000
puts "  RX reset deasserted"
after 500

# ==========================================================================
# 6. Set GTFMAC data rate
# ==========================================================================
puts "=== Step 6: GTFMAC data rate ==="
axi_write_verify $jaxi 0x10000 0x00000000 "GTFMAC data_rate"

# ==========================================================================
# 7. HWCHK data rate config
# ==========================================================================
puts "=== Step 7: HWCHK data rate config ==="
set hwcfg [axi_read $jaxi 0x00010]
puts "  HWCHK config before: [format 0x%08x $hwcfg]"
set hwcfg [expr {$hwcfg | (0 << 0)}]
set hwcfg [expr {$hwcfg | (0 << 16)}]
axi_write $jaxi 0x00010 $hwcfg
puts "  HWCHK config after:  [format 0x%08x [axi_read $jaxi 0x00010]]"

# ==========================================================================
# 8. Allow bitslip
# ==========================================================================
puts "=== Step 8: Bitslip allow ==="
axi_write $jaxi 0x000A4 0x00000000
puts "  bitslip_ctrl = [format 0x%08x [axi_read $jaxi 0x000A4]]"

# ==========================================================================
# 9. Wait for block lock
# ==========================================================================
puts "=== Step 9: Block lock ==="
set result [axi_poll $jaxi 0x000A0 0x00010000 0x00010000]
puts "  Block lock after [lindex $result 1] attempts, reg=[format 0x%08x [lindex $result 0]]"

# ==========================================================================
# 10. Trigger bitslip correction
# ==========================================================================
puts "=== Step 10: Bitslip correct ==="
axi_write $jaxi 0x000A4 0x00000001
puts "  bitslip_ctrl = [format 0x%08x [axi_read $jaxi 0x000A4]]"

# ==========================================================================
# 11. Wait for bitslip done (bit 18)
# ==========================================================================
puts "=== Step 11: Bitslip done ==="
set result [axi_poll $jaxi 0x000A0 0x00040000 0x00040000 100 10]
puts "  Bitslip done after [lindex $result 1] attempts, reg=[format 0x%08x [lindex $result 0]]"

# ==========================================================================
# 12. Wait for block lock after bitslip
# ==========================================================================
puts "=== Step 12: Block lock re-acquire ==="
set result [axi_poll $jaxi 0x000A0 0x00010000 0x00010000]
puts "  Block lock after [lindex $result 1] attempts, reg=[format 0x%08x [lindex $result 0]]"

# ==========================================================================
# 12b. Wait for RX buffer bypass to complete
#      The testbench waits for gtwiz_buffbypass_rx_done_out_i before
#      proceeding. There is no direct AXI register for this, so we use a
#      generous delay to allow the GT RX buffer bypass alignment to finish.
# ==========================================================================
puts "=== Step 12b: Wait for RX buffer bypass ==="
puts "  Waiting 5 seconds for RX buffer bypass completion..."
after 5000
axi_dump $jaxi 0x00000 "HWCHK status (post-bypass-wait)"
axi_dump $jaxi 0x000A0 "HWCHK block_lock (post-bypass-wait)"

# ==========================================================================
# 13. Wait for link up
# ==========================================================================
puts "=== Step 13: Link up ==="
set result [axi_poll $jaxi 0x00000 0x00000F00 0x00000000 10000 1]
puts "  Link up after [lindex $result 1] attempts (status=[format 0x%08x [lindex $result 0]])"

puts ""
puts "============================================================"
puts " DIAGNOSTIC REGISTER DUMP - POST-LINK-UP / PRE-CONFIG"
puts "============================================================"
axi_dump $jaxi 0x00000 "HWCHK status"
axi_dump $jaxi 0x00004 "HWCHK reg 0x004"
axi_dump $jaxi 0x00008 "HWCHK reg 0x008"
axi_dump $jaxi 0x0000C "HWCHK userrdy"
axi_dump $jaxi 0x00010 "HWCHK config"
axi_dump $jaxi 0x00014 "HWCHK gen_mode"
axi_dump $jaxi 0x00018 "HWCHK reg 0x018"
axi_dump $jaxi 0x0001C "HWCHK reg 0x01C"
axi_dump $jaxi 0x00020 "HWCHK gen/mon enable"
axi_dump $jaxi 0x00024 "HWCHK max_frame_len"
axi_dump $jaxi 0x00028 "HWCHK min_frame_len"
axi_dump $jaxi 0x0002C "HWCHK frames_to_send"
axi_dump $jaxi 0x000A0 "HWCHK block_lock/bitslip"
axi_dump $jaxi 0x10000 "GTFMAC data_rate"
axi_dump $jaxi 0x10004 "GTFMAC TX config"
axi_dump $jaxi 0x10008 "GTFMAC RX config"
axi_dump $jaxi 0x1000C "GTFMAC RX min pkt"
axi_dump $jaxi 0x10010 "GTFMAC RX max pkt"
puts ""

# ==========================================================================
# 14. TX config (0x10004) — read-modify-write
# ==========================================================================
puts "=== Step 14: TX config ==="
set txcfg [axi_read $jaxi 0x10004]
puts "  TX config before: [format 0x%08x $txcfg] (binary: [format %032b $txcfg])"
set txcfg [expr {$txcfg & 0xFFFFE0F1}]
set txcfg [expr {$txcfg | (1 << 1)}]
set txcfg [expr {$txcfg | (0 << 2)}]
set txcfg [expr {$txcfg | (0 << 3)}]
set txcfg [expr {$txcfg | (8 << 8)}]
set txcfg [expr {$txcfg | (0 << 12)}]
axi_write $jaxi 0x10004 $txcfg
set txcfg_rb [axi_read $jaxi 0x10004]
puts "  TX config after:  [format 0x%08x $txcfg_rb] (binary: [format %032b $txcfg_rb])"

# ==========================================================================
# 15. RX config (0x10008) — read-modify-write
# ==========================================================================
puts "=== Step 15: RX config ==="
set rxcfg [axi_read $jaxi 0x10008]
puts "  RX config before: [format 0x%08x $rxcfg] (binary: [format %032b $rxcfg])"
set rxcfg [expr {$rxcfg & 0xFFFFFF9B}]
set rxcfg [expr {$rxcfg | (0 << 2)}]
set rxcfg [expr {$rxcfg | (0 << 5)}]
set rxcfg [expr {$rxcfg | (0 << 6)}]
axi_write $jaxi 0x10008 $rxcfg
set rxcfg_rb [axi_read $jaxi 0x10008]
puts "  RX config after:  [format 0x%08x $rxcfg_rb] (binary: [format %032b $rxcfg_rb])"

# ==========================================================================
# 16-17. Min/Max packet len
# ==========================================================================
puts "=== Step 16-17: Packet lengths ==="
axi_write_verify $jaxi 0x1000C 0x00000040 "min_pkt_len"
axi_write_verify $jaxi 0x10010 0x000005DC "max_pkt_len"

# ==========================================================================
# 18. HWCHK config — second pass (fcs_ins, preamble, etc.)
# ==========================================================================
puts "=== Step 18: HWCHK config (fcs/preamble) ==="
set hwcfg [axi_read $jaxi 0x00010]
puts "  HWCHK config before: [format 0x%08x $hwcfg]"
set hwcfg [expr {$hwcfg | (1 << 4)}]
set hwcfg [expr {$hwcfg | (0 << 8)}]
set hwcfg [expr {$hwcfg | (0 << 12)}]
set hwcfg [expr {$hwcfg | (0 << 24)}]
axi_write $jaxi 0x00010 $hwcfg
puts "  HWCHK config after:  [format 0x%08x [axi_read $jaxi 0x00010]]"

# ==========================================================================
# 19-20. Error / Poison injection
# ==========================================================================
puts "=== Step 19-20: Injection ==="
axi_write_verify $jaxi 0x00040 0x00000000 "err_inj"
axi_write_verify $jaxi 0x00098 0x00000000 "poison_inj"

# ==========================================================================
# 21-22. Min/Max frame len (HWCHK generator)
# ==========================================================================
puts "=== Step 21-22: Frame lengths ==="
axi_write_verify $jaxi 0x00028 0x00000040 "hwchk_min_frame"
axi_write_verify $jaxi 0x00024 0x000005DC "hwchk_max_frame"

# ==========================================================================
# 23. Frame gen mode
# ==========================================================================
puts "=== Step 23: Gen mode ==="
axi_write_verify $jaxi 0x00014 0x00000000 "gen_mode"

# ==========================================================================
# 24. Frames to send
# ==========================================================================
puts "=== Step 24: Frames to send ==="
axi_write_verify $jaxi 0x0002C 0x00000032 "frames_to_send"

# ==========================================================================
# 25-26. Init stats
# ==========================================================================
puts "=== Step 25-26: Init stats ==="
axi_write $jaxi 0x00090 0x00000001
puts "  HWCHK stats tick (init)"
axi_write $jaxi 0x1040C 0x00000001
puts "  GTFMAC stats tick (init)"

after 100

puts ""
puts "============================================================"
puts " DIAGNOSTIC REGISTER DUMP - PRE-TRAFFIC"
puts "============================================================"
axi_dump $jaxi 0x00000 "HWCHK status"
axi_dump $jaxi 0x00010 "HWCHK config"
axi_dump $jaxi 0x00014 "HWCHK gen_mode"
axi_dump $jaxi 0x00020 "HWCHK gen/mon enable"
axi_dump $jaxi 0x00024 "HWCHK max_frame_len"
axi_dump $jaxi 0x00028 "HWCHK min_frame_len"
axi_dump $jaxi 0x0002C "HWCHK frames_to_send"
axi_dump $jaxi 0x00040 "HWCHK err_inj"
axi_dump $jaxi 0x00098 "HWCHK poison_inj"
axi_dump $jaxi 0x000A0 "HWCHK block_lock/bitslip"
axi_dump $jaxi 0x10004 "GTFMAC TX config"
axi_dump $jaxi 0x10008 "GTFMAC RX config"
axi_dump $jaxi 0x1000C "GTFMAC RX min pkt"
axi_dump $jaxi 0x10010 "GTFMAC RX max pkt"
puts ""

# =============================================================================
# ILA SETUP — arm all 3 ILAs before starting traffic
# =============================================================================
puts "Setting up ILA captures..."

# Discover ILA cores — names depend on what Vivado assigned
set ila_list [get_hw_ilas]
puts "Found ILAs: $ila_list"

# --- ILA 0 (hw_ila_1 / ila_inst — status/reset on freerun_clk) ---
# Trigger: immediate (capture steady-state status)
set ila0 [get_hw_ilas hw_ila_1]
set_property CONTROL.TRIGGER_POSITION 512 $ila0
set_property CONTROL.DATA_DEPTH 1024 $ila0
run_hw_ila $ila0
puts "ILA0 (status) armed."

# --- ILA TX (hw_ila_3 / ila_tx_inst — tx data path on tx_axis_clk) ---
# Trigger: tx_axis_tvalid == 1
set ila_tx [get_hw_ilas hw_ila_3]
set_property CONTROL.TRIGGER_POSITION 512 $ila_tx
set_property CONTROL.DATA_DEPTH 1024 $ila_tx
set_property TRIGGER_COMPARE_VALUE eq1'b1 [get_hw_probes tx_axis_tvalid -of $ila_tx]
run_hw_ila $ila_tx
puts "ILA TX armed — trigger on tx_axis_tvalid."

# --- ILA RX (hw_ila_2 / ila_rx_inst — rx data path on rx_axis_clk) ---
# Trigger: rx_axis_tvalid == 1
set ila_rx [get_hw_ilas hw_ila_2]
set_property CONTROL.TRIGGER_POSITION 512 $ila_rx
set_property CONTROL.DATA_DEPTH 1024 $ila_rx
set_property TRIGGER_COMPARE_VALUE eq1'b1 [get_hw_probes rx_axis_tvalid -of $ila_rx]
run_hw_ila $ila_rx
puts "ILA RX armed — trigger on rx_axis_tvalid."

# ==========================================================================
# 27. START frame generator and monitor
# ==========================================================================
puts "=== Step 27: START generator ==="
puts "  Writing 0x11 to gen/mon enable (0x20)..."
axi_write $jaxi 0x00020 0x00000011
set gen_rb [axi_read $jaxi 0x00020]
puts "  gen/mon readback: [format 0x%08x $gen_rb]"
if {($gen_rb & 0x11) != 0x11} {
    puts "  WARNING: gen_en or mon_en did not latch!"
}

puts "  Waiting 1 second..."
after 1000

puts "  --- Mid-traffic snapshot ---"
axi_dump $jaxi 0x00000 "HWCHK status (mid-traffic)"
axi_dump $jaxi 0x00020 "HWCHK gen/mon enable (mid-traffic)"
# Read some HWCHK internal state
axi_dump $jaxi 0x00030 "HWCHK reg 0x030"
axi_dump $jaxi 0x00034 "HWCHK reg 0x034"
axi_dump $jaxi 0x00038 "HWCHK reg 0x038"
axi_dump $jaxi 0x0003C "HWCHK reg 0x03C"
axi_dump $jaxi 0x000A0 "HWCHK block_lock (mid-traffic)"

puts "  Waiting 4 more seconds..."
after 4000

puts "  --- Post-traffic snapshot (gen still on) ---"
axi_dump $jaxi 0x00000 "HWCHK status (post-wait)"
axi_dump $jaxi 0x00020 "HWCHK gen/mon enable (post-wait)"

# ==========================================================================
# 28. STOP frame generator
# ==========================================================================
puts "=== Step 28: STOP generator ==="
axi_write $jaxi 0x00020 0x00000000
puts "  gen/mon stopped: [format 0x%08x [axi_read $jaxi 0x00020]]"

after 1000

# ==========================================================================
# 29-30. Tick stats for collection
# ==========================================================================
puts "=== Step 29-30: Tick stats ==="
axi_write $jaxi 0x00090 0x00000001
puts "  HWCHK stats ticked"
axi_write $jaxi 0x1040C 0x00000001
puts "  GTFMAC stats ticked"

after 500

# ==========================================================================
# COMPREHENSIVE STAT DUMP
# ==========================================================================
puts ""
puts "============================================================"
puts " GTFMAC MAC STATISTICS"
puts "============================================================"

# Cycle counts (non-zero = MAC clocks are running)
axi_dump $jaxi 0x10500 "status_rx_cycle_count_lo"
axi_dump $jaxi 0x10504 "status_rx_cycle_count_hi"
axi_dump $jaxi 0x10508 "status_tx_cycle_count_lo"
axi_dump $jaxi 0x1050C "status_tx_cycle_count_hi"

# Error indicators
axi_dump $jaxi 0x10648 "stat_rx_framing_err_lo"
axi_dump $jaxi 0x1064C "stat_rx_framing_err_hi"
axi_dump $jaxi 0x10660 "stat_rx_bad_code_lo"
axi_dump $jaxi 0x10664 "stat_rx_bad_code_hi"
axi_dump $jaxi 0x106A0 "stat_tx_frame_error_lo"
axi_dump $jaxi 0x106A4 "stat_tx_frame_error_hi"

# TX stats
axi_dump $jaxi 0x10700 "stat_tx_total_packets_lo"
axi_dump $jaxi 0x10704 "stat_tx_total_packets_hi"
axi_dump $jaxi 0x10708 "stat_tx_total_good_packets_lo"
axi_dump $jaxi 0x1070C "stat_tx_total_good_packets_hi"
axi_dump $jaxi 0x10710 "stat_tx_total_bytes_lo"
axi_dump $jaxi 0x10714 "stat_tx_total_bytes_hi"
axi_dump $jaxi 0x10718 "stat_tx_total_good_bytes_lo"
axi_dump $jaxi 0x1071C "stat_tx_total_good_bytes_hi"

# RX stats
axi_dump $jaxi 0x10808 "stat_rx_total_packets_lo"
axi_dump $jaxi 0x1080C "stat_rx_total_packets_hi"
axi_dump $jaxi 0x10810 "stat_rx_total_good_packets_lo"
axi_dump $jaxi 0x10814 "stat_rx_total_good_packets_hi"
axi_dump $jaxi 0x10818 "stat_rx_total_bytes_lo"
axi_dump $jaxi 0x1081C "stat_rx_total_bytes_hi"
axi_dump $jaxi 0x10820 "stat_rx_total_good_bytes_lo"
axi_dump $jaxi 0x10824 "stat_rx_total_good_bytes_hi"

# RX errors
axi_dump $jaxi 0x108C0 "stat_rx_bad_fcs_lo"
axi_dump $jaxi 0x108C4 "stat_rx_bad_fcs_hi"
axi_dump $jaxi 0x108D0 "stat_rx_stomped_fcs_lo"
axi_dump $jaxi 0x108D4 "stat_rx_stomped_fcs_hi"

puts ""
puts "============================================================"
puts " HWCHK REGISTER SPACE SCAN (0x00 - 0xC0, every 4 bytes)"
puts "============================================================"
for {set a 0x00} {$a <= 0xC0} {set a [expr {$a + 4}]} {
    axi_dump $jaxi $a "HWCHK [format 0x%03x $a]"
}

puts ""
puts "============================================================"
puts " FINAL STATUS"
puts "============================================================"
axi_dump $jaxi 0x00000 "HWCHK status (final)"
axi_dump $jaxi 0x000A0 "HWCHK block_lock (final)"
axi_dump $jaxi 0x10004 "GTFMAC TX config (final)"
axi_dump $jaxi 0x10008 "GTFMAC RX config (final)"


# =============================================================================
# ILA CAPTURE — upload data and export to VCD
# =============================================================================
puts "Uploading ILA captures..."

set outdir "$::env(HOME)/group_05_project/loopback_example/ila_captures"
file mkdir $outdir

set ila0 [get_hw_ilas hw_ila_1]
wait_on_hw_ila $ila0
set ila0_data [upload_hw_ila_data $ila0]
write_hw_ila_data -force -vcd_file ${outdir}/ila_status.vcd $ila0_data
write_hw_ila_data -force -csv_file ${outdir}/ila_status.csv $ila0_data
puts "ILA0 (status) -> ila_status.vcd / .csv"

set ila_tx [get_hw_ilas hw_ila_3]
wait_on_hw_ila $ila_tx
set ilatx_data [upload_hw_ila_data $ila_tx]
write_hw_ila_data -force -vcd_file ${outdir}/ila_tx.vcd $ilatx_data
write_hw_ila_data -force -csv_file ${outdir}/ila_tx.csv $ilatx_data
puts "ILA TX -> ila_tx.vcd / .csv"

set ila_rx [get_hw_ilas hw_ila_2]
wait_on_hw_ila $ila_rx
set ilarx_data [upload_hw_ila_data $ila_rx]
write_hw_ila_data -force -vcd_file ${outdir}/ila_rx.vcd $ilarx_data
write_hw_ila_data -force -csv_file ${outdir}/ila_rx.csv $ilarx_data
puts "ILA RX -> ila_rx.vcd / .csv"

puts "ILA captures saved to $outdir"


set tx_lo [axi_read $jaxi 0x10700]
set rx_lo [axi_read $jaxi 0x10808]
puts ""
if {$tx_lo == $rx_lo && $tx_lo == 50} {
    puts "PASS: TX == RX == 50"
} elseif {$tx_lo == 0 && $rx_lo == 0} {
    puts "FAIL: Zero packets - frame generator did not produce traffic"
    puts "  Check: HWCHK gen/mon enable readback, cycle counts, block lock"
} else {
    puts "FAIL: TX=$tx_lo RX=$rx_lo (expected 50)"
}
puts "--- DONE ---"
