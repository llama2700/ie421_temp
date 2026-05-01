proc qsfp_enable_power_only {} {
    puts ""
    puts "-- Deassert Resets to I2C I/O Expanders and Switches and I2C Controller"
    reg_wr 0x10 0x7F
    
    
    puts ""
    puts "-- Disable and re-enable QSFP power domains..."
    disable_qsfp_power
    after 1000
    enable_qsfp_power
}

proc qsfp_reset_port {port} {
    puts ""
    puts "-- Enable QSFP $port..."
    # Select sideband routing...
    select_qsfp_sb $port
    # Toggle RESETL...
    assert_qsfp_sb_reset
    deassert_qsfp_sb_reset
    # Readback SB status...
    read_qsfp_sb_status
}

proc qsfp_access_i2c {port} {
    set DEV_ID_QSFP_I2C 0xA0

    puts ""
    puts "Example QSFP $port MODULE I2C Access..."
    
    select_qsfp_i2c $port

    puts ""
    puts "-- Read Module Identifier --"
    i2c_rd $DEV_ID_QSFP_I2C 0x00
}

proc dump_i2c_regs {dev_id start num_reg} {
    puts ""
    puts "-- Dumping I2C registers from [format 0x%02X $start] for $num_reg bytes"

    for {set i 0} {$i < $num_reg} {incr i} {
        set addr [expr {$start + $i}]
        set val [i2c_rd_quiet $dev_id $addr]

        # Extract the actual byte (matches your existing parsing style)
        set byte [string range $val 8 9]

        puts [format "0x%02X : 0x%s" $addr $byte]
    }
}