#
# 4_user_i2c.tcl
#
# User I2C read/write via reg_i2c_user_logic registers.
# Run after 1_flash_bitstream.tcl (debug cores must be discovered).
#
# Register map:
#   0x100: CONTROL - write: bit[31]=RW (1=read,0=write), [7:0]=DeviceID
#                    read:  bit[30]=CMPLT
#   0x104: ADDR    - bits[7:0] = register address
#   0x108: WDATA   - bits[7:0] = write data
#   0x10C: RDATA   - bits[7:0] = read data
#   0x120: SCRATCH - default 0x600DFEED
#

set axi [lindex [get_hw_axis] 0]

# --- Helper procs ---

proc i2c_read {device_id reg_addr} {
    global axi

    # Write register address
    set addr_hex [format %08X [expr {$reg_addr & 0xFF}]]
    create_hw_axi_txn wr_addr $axi -type write -address 00000104 -data $addr_hex -force
    run_hw_axi [get_hw_axi_txns wr_addr]

    # Write CONTROL: bit[31]=1 (read), bits[7:0]=device_id
    set ctrl [format %08X [expr {0x80000000 | ($device_id & 0xFF)}]]
    create_hw_axi_txn wr_ctrl $axi -type write -address 00000100 -data $ctrl -force
    run_hw_axi [get_hw_axi_txns wr_ctrl]

    # Poll for completion (bit[30] of CONTROL)
    set timeout 100
    for {set i 0} {$i < $timeout} {incr i} {
        after 10
        create_hw_axi_txn rd_ctrl $axi -type read -address 00000100 -force
        run_hw_axi [get_hw_axi_txns rd_ctrl]
        set status [get_property DATA [get_hw_axi_txns rd_ctrl]]
        scan $status %x status_val
        if {$status_val & 0x40000000} break
    }

    if {$i >= $timeout} {
        puts "ERROR: I2C read timed out (device=0x[format %02X $device_id] addr=0x[format %02X $reg_addr])"
        return -1
    }

    # Read RDATA
    create_hw_axi_txn rd_data $axi -type read -address 0000010C -force
    run_hw_axi [get_hw_axi_txns rd_data]
    set rdata [get_property DATA [get_hw_axi_txns rd_data]]
    scan $rdata %x rdata_val
    set result [expr {$rdata_val & 0xFF}]

    puts [format "I2C READ:  dev=0x%02X addr=0x%02X => data=0x%02X" $device_id $reg_addr $result]
    return $result
}

proc i2c_write {device_id reg_addr data} {
    global axi

    # Write register address
    set addr_hex [format %08X [expr {$reg_addr & 0xFF}]]
    create_hw_axi_txn wr_addr $axi -type write -address 00000104 -data $addr_hex -force
    run_hw_axi [get_hw_axi_txns wr_addr]

    # Write WDATA
    set wdata_hex [format %08X [expr {$data & 0xFF}]]
    create_hw_axi_txn wr_wdata $axi -type write -address 00000108 -data $wdata_hex -force
    run_hw_axi [get_hw_axi_txns wr_wdata]

    # Write CONTROL: bit[31]=0 (write), bits[7:0]=device_id
    set ctrl [format %08X [expr {$device_id & 0xFF}]]
    create_hw_axi_txn wr_ctrl $axi -type write -address 00000100 -data $ctrl -force
    run_hw_axi [get_hw_axi_txns wr_ctrl]

    # Poll for completion
    set timeout 100
    for {set i 0} {$i < $timeout} {incr i} {
        after 10
        create_hw_axi_txn rd_ctrl $axi -type read -address 00000100 -force
        run_hw_axi [get_hw_axi_txns rd_ctrl]
        set status [get_property DATA [get_hw_axi_txns rd_ctrl]]
        scan $status %x status_val
        if {$status_val & 0x40000000} break
    }

    if {$i >= $timeout} {
        puts "ERROR: I2C write timed out (device=0x[format %02X $device_id] addr=0x[format %02X $reg_addr])"
        return -1
    }

    puts [format "I2C WRITE: dev=0x%02X addr=0x%02X <= data=0x%02X" $device_id $reg_addr $data]
    return 0
}

# --- Scratch register test ---

puts ""
puts "=== Scratch Register Test ==="
create_hw_axi_txn rd_scratch $axi -type read -address 00000120 -force
run_hw_axi [get_hw_axi_txns rd_scratch]
set scratch [get_property DATA [get_hw_axi_txns rd_scratch]]
puts "SCRATCH (0x120) = 0x$scratch"

if {$scratch eq "600DFEED"} {
    puts "PASS: Scratch register matches expected value."
} else {
    puts "FAIL: Expected 0x600DFEED, got 0x$scratch"
}

puts ""
puts "=== Ready ==="
puts "Usage:"
puts "  i2c_read  <device_id> <reg_addr>"
puts "  i2c_write <device_id> <reg_addr> <data>"
puts ""
puts "Example: i2c_read 0xB0 0x00   ;# Read reg 0 from Renesas JC1"
