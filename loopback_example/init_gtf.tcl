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

# 1. Check initial status
create_hw_axi_txn rd_stat [lindex $jaxi 0] -type READ -address 00000000 -force
run_hw_axi rd_stat
puts "Initial status   (0x00000): [get_property DATA [get_hw_axi_txns rd_stat]]"

# 2. userrdy
create_hw_axi_txn wr_urdy [lindex $jaxi 0] -type WRITE -address 0000000C -data 00000003 -force
run_hw_axi wr_urdy
puts "userrdy set"

# 3. Set loopback bits[6:4] = 3'b010 = 0x20
create_hw_axi_txn wr_lb [lindex $jaxi 0] -type WRITE -address 00010408 -data 00000020 -force
run_hw_axi wr_lb
puts "Loopback set"

# 4. Assert RX reset
create_hw_axi_txn wr_rst_on [lindex $jaxi 0] -type WRITE -address 00010400 -data 00000002 -force
run_hw_axi wr_rst_on
puts "Reset asserted"
after 500

# 5. Deassert RX reset
create_hw_axi_txn wr_rst_off [lindex $jaxi 0] -type WRITE -address 00010400 -data 00000000 -force
run_hw_axi wr_rst_off
puts "Reset deasserted"

# 6. Set data rate
create_hw_axi_txn wr_rate [lindex $jaxi 0] -type WRITE -address 00010000 -data 00000000 -force
run_hw_axi wr_rate
puts "Data rate set"

# 7. Allow bitslip
create_hw_axi_txn wr_bs [lindex $jaxi 0] -type WRITE -address 000000A4 -data 00000000 -force
run_hw_axi wr_bs
puts "Bitslip allowed"
after 2000

# 8. Read block lock status
create_hw_axi_txn rd_lock [lindex $jaxi 0] -type READ -address 000000A0 -force
run_hw_axi rd_lock
puts "Block lock       (0x000A0): [get_property DATA [get_hw_axi_txns rd_lock]]"

# 9. Read loopback confirm
create_hw_axi_txn rd_lb [lindex $jaxi 0] -type READ -address 00010408 -force
run_hw_axi rd_lb
puts "Loopback reg     (0x10408): [get_property DATA [get_hw_axi_txns rd_lb]]"

# 10. Trigger bitslip correction
create_hw_axi_txn wr_bs2 [lindex $jaxi 0] -type WRITE -address 000000A4 -data 00000001 -force
run_hw_axi wr_bs2
puts "Bitslip correction triggered"
after 500

# 11. Confirm bitslip done (bit[18]) and block lock again (bit[16])
create_hw_axi_txn rd_lock2 [lindex $jaxi 0] -type READ -address 000000A0 -force
run_hw_axi rd_lock2
puts "Block lock after bitslip (0x000A0): [get_property DATA [get_hw_axi_txns rd_lock2]]"

# 12. Wait for link up - poll 0x000 bits[11:8] == 0
after 2000
create_hw_axi_txn rd_link [lindex $jaxi 0] -type READ -address 00000000 -force
run_hw_axi rd_link
puts "Link status      (0x00000): [get_property DATA [get_hw_axi_txns rd_link]]"

# 13. TX config: fcs_ins_enable=1, ipg=8 -> 0x00000802
create_hw_axi_txn wr_txcfg [lindex $jaxi 0] -type WRITE -address 00010004 -data 00000802 -force
run_hw_axi wr_txcfg
puts "TX config set"

# 14. RX config: all zeros
create_hw_axi_txn wr_rxcfg [lindex $jaxi 0] -type WRITE -address 00010008 -data 00000000 -force
run_hw_axi wr_rxcfg
puts "RX config set"

# 15. Min packet len = 64 = 0x40
create_hw_axi_txn wr_minpkt [lindex $jaxi 0] -type WRITE -address 0001000C -data 00000040 -force
run_hw_axi wr_minpkt
puts "Min packet len set"

# 16. Max packet len = 1500 = 0x5DC
create_hw_axi_txn wr_maxpkt [lindex $jaxi 0] -type WRITE -address 00010010 -data 000005DC -force
run_hw_axi wr_maxpkt
puts "Max packet len set"

# 17. HWCHK config: read-modify-write, set fcs_ins_enable bit[4]
create_hw_axi_txn rd_hwcfg [lindex $jaxi 0] -type READ -address 00000010 -force
run_hw_axi rd_hwcfg
set hwcfg [get_property DATA [get_hw_axi_txns rd_hwcfg]]
set hwcfg_val [expr {[scan $hwcfg %x tmp; set tmp] | 0x00000010}]
set hwcfg_hex [format "%08x" $hwcfg_val]
create_hw_axi_txn wr_hwcfg [lindex $jaxi 0] -type WRITE -address 00000010 -data $hwcfg_hex -force
run_hw_axi wr_hwcfg
puts "HWCHK config set: $hwcfg_hex"

# 18. Error injection = 0
create_hw_axi_txn wr_errinj [lindex $jaxi 0] -type WRITE -address 00000040 -data 00000000 -force
run_hw_axi wr_errinj
puts "Error injection set"

# 19. Poison injection = 0
create_hw_axi_txn wr_poison [lindex $jaxi 0] -type WRITE -address 00000098 -data 00000000 -force
run_hw_axi wr_poison
puts "Poison injection set"

# 20. Min frame len = 64 = 0x40
create_hw_axi_txn wr_minfrm [lindex $jaxi 0] -type WRITE -address 00000028 -data 00000040 -force
run_hw_axi wr_minfrm
puts "Min frame len set"

# 21. Max frame len = 1500 = 0x5DC
create_hw_axi_txn wr_maxfrm [lindex $jaxi 0] -type WRITE -address 00000024 -data 000005DC -force
run_hw_axi wr_maxfrm
puts "Max frame len set"

# 22. Frame gen mode = 0 (random), variable_ipg = 0
create_hw_axi_txn wr_genmode [lindex $jaxi 0] -type WRITE -address 00000014 -data 00000000 -force
run_hw_axi wr_genmode
puts "Frame gen mode set"

# 23. Frames to send = 50 = 0x32
create_hw_axi_txn wr_nfrm [lindex $jaxi 0] -type WRITE -address 0000002C -data 00000032 -force
run_hw_axi wr_nfrm
puts "Frames to send set"

# 24. Init HWCHK stats
create_hw_axi_txn wr_hwstat [lindex $jaxi 0] -type WRITE -address 00000090 -data 00000001 -force
run_hw_axi wr_hwstat
puts "HWCHK stats initialized"

# 25. Init GTFMAC stats
create_hw_axi_txn wr_macstat [lindex $jaxi 0] -type WRITE -address 0001040C -data 00000001 -force
run_hw_axi wr_macstat
puts "GTFMAC stats initialized"

# 26. Start frame generator and monitor
create_hw_axi_txn wr_gen_en [lindex $jaxi 0] -type WRITE -address 00000020 -data 00000011 -force
run_hw_axi wr_gen_en
puts "Frame generator started"

# Wait for frames to be sent (50 frames at 10G should be near-instant, wait 1s to be safe)
after 1000

# 27. Stop frame generator
create_hw_axi_txn wr_gen_off [lindex $jaxi 0] -type WRITE -address 00000020 -data 00000000 -force
run_hw_axi wr_gen_off
puts "Frame generator stopped"

# 28. Tick HWCHK stats for collection
create_hw_axi_txn wr_hwstat2 [lindex $jaxi 0] -type WRITE -address 00000090 -data 00000001 -force
run_hw_axi wr_hwstat2

# 29. Tick GTFMAC stats for collection
create_hw_axi_txn wr_macstat2 [lindex $jaxi 0] -type WRITE -address 0001040C -data 00000001 -force
run_hw_axi wr_macstat2
after 500

# 30. Read TX total packets (64-bit, two 32-bit reads)
create_hw_axi_txn rd_txpkt_lo [lindex $jaxi 0] -type READ -address 00010700 -force
run_hw_axi rd_txpkt_lo
create_hw_axi_txn rd_txpkt_hi [lindex $jaxi 0] -type READ -address 00010704 -force
run_hw_axi rd_txpkt_hi
puts "TX total packets lo (0x10700): [get_property DATA [get_hw_axi_txns rd_txpkt_lo]]"
puts "TX total packets hi (0x10704): [get_property DATA [get_hw_axi_txns rd_txpkt_hi]]"

# 31. Read RX total packets
create_hw_axi_txn rd_rxpkt_lo [lindex $jaxi 0] -type READ -address 00010808 -force
run_hw_axi rd_rxpkt_lo
create_hw_axi_txn rd_rxpkt_hi [lindex $jaxi 0] -type READ -address 0001080C -force
run_hw_axi rd_rxpkt_hi
puts "RX total packets lo (0x10808): [get_property DATA [get_hw_axi_txns rd_rxpkt_lo]]"
puts "RX total packets hi (0x1080C): [get_property DATA [get_hw_axi_txns rd_rxpkt_hi]]"

# 32. Read TX good packets
create_hw_axi_txn rd_txgood_lo [lindex $jaxi 0] -type READ -address 00010708 -force
run_hw_axi rd_txgood_lo
puts "TX good packets  lo (0x10708): [get_property DATA [get_hw_axi_txns rd_txgood_lo]]"

# 33. Read RX good packets
create_hw_axi_txn rd_rxgood_lo [lindex $jaxi 0] -type READ -address 00010810 -force
run_hw_axi rd_rxgood_lo
puts "RX good packets  lo (0x10810): [get_property DATA [get_hw_axi_txns rd_rxgood_lo]]"

# 34. Read RX bad FCS
create_hw_axi_txn rd_rxbadfcs_lo [lindex $jaxi 0] -type READ -address 000108C0 -force
run_hw_axi rd_rxbadfcs_lo
puts "RX bad FCS       lo (0x108C0): [get_property DATA [get_hw_axi_txns rd_rxbadfcs_lo]]"

puts "--- DONE ---"
puts "Expected: TX == RX == 50 packets, bad FCS == 0"
EOF