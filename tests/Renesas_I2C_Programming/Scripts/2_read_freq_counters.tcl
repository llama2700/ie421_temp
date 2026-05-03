#
# 2_read_freq_counters.tcl
#
# Reads the 8 SYNCE clock frequency counters via JTAG AXI.
# Run after 1_flash_bitstream.tcl. Safe to run at any time.
#
# Register map:
#   0x14 bit[0] = resetn, bit[1] = samp_start
#   0x18 = sample window width (default 0x186A0 = 100k cycles @ 50MHz = 2ms)
#   0x10 bit[0] = samp_valid
#   0x20-0x3C = counter values for channels 0-7
#
# Channel mapping:
#   [0] Bank 127   [4] Bank 227
#   [1] Bank 128   [5] Bank 228
#   [2] Bank 130   [6] Bank 229
#   [3] Bank 131   [7] Bank 230
#

set axi [lindex [get_hw_axis] 0]

# Step 1: Release reset
create_hw_axi_txn wr_rstn $axi -type write -address 00000014 -data 00000001 -force
run_hw_axi [get_hw_axi_txns wr_rstn]
puts "Reset released."

# Step 2: Trigger sample (bit[0]=1 resetn, bit[1]=1 start)
create_hw_axi_txn wr_start $axi -type write -address 00000014 -data 00000003 -force
run_hw_axi [get_hw_axi_txns wr_start]
puts "Sample triggered. Waiting..."

# Step 3: Wait for measurement to complete
after 500

# Step 4: Check status
create_hw_axi_txn rd_status $axi -type read -address 00000010 -force
run_hw_axi [get_hw_axi_txns rd_status]
set status [get_property DATA [get_hw_axi_txns rd_status]]
puts "Status (0x10) = $status  (bit0=1 means valid)"

# Step 5: Read all 8 counters
set banks [list "Bank 127" "Bank 128" "Bank 130" "Bank 131" "Bank 227" "Bank 228" "Bank 229" "Bank 230"]
set addrs [list 00000020 00000024 00000028 0000002C 00000030 00000034 00000038 0000003C]

puts ""
puts "=========================================="
puts "  SYNCE Frequency Counter Results"
puts "=========================================="

for {set i 0} {$i < 8} {incr i} {
    set addr [lindex $addrs $i]
    set bank [lindex $banks $i]
    create_hw_axi_txn rd_fc$i $axi -type read -address $addr -force
    run_hw_axi [get_hw_axi_txns rd_fc$i]
    set raw [get_property DATA [get_hw_axi_txns rd_fc$i]]

    # Convert hex to decimal and compute frequency
    scan $raw %x dec_val
    set freq_mhz [expr {$dec_val * 500.0 / 1000000.0}]

    puts [format "  \[%d\] %-8s  addr=0x%s  raw=0x%s  count=%-10d  freq=%8.4f MHz" \
        $i $bank $addr $raw $dec_val $freq_mhz]
}

puts "=========================================="
puts ""
puts "Formula: freq = count * (50MHz / 100000) = count * 500 Hz"
puts "To increase precision, write a larger sample window to 0x18."
