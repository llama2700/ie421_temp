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
#   0x108: WDATA   - write data buffer (each write pushes a byte, up to 8)
#   0x10C: RDATA   - bits[7:0] = read data
#   0x110: WCOUNT  - bits[3:0] = number of data bytes to write (default 1)
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

    # Set WCOUNT = 1 (single byte write)
    create_hw_axi_txn wr_wcnt $axi -type write -address 00000110 -data 00000001 -force
    run_hw_axi [get_hw_axi_txns wr_wcnt]

    # Write register address
    set addr_hex [format %08X [expr {$reg_addr & 0xFF}]]
    create_hw_axi_txn wr_addr $axi -type write -address 00000104 -data $addr_hex -force
    run_hw_axi [get_hw_axi_txns wr_addr]

    # Write WDATA (push 1 byte into buffer)
    set wdata_hex [format %08X [expr {$data & 0xFF}]]
    create_hw_axi_txn wr_wdata $axi -type write -address 00000108 -data $wdata_hex -force
    run_hw_axi [get_hw_axi_txns wr_wdata]

    # Write CONTROL: bit[31]=0 (write), bits[7:0]=device_id
    # (this also resets the buffer pointer)
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

# Multi-byte write: sends N data bytes in a single I2C transaction
# Usage: i2c_write_multi <device_id> <reg_addr> <byte_list>
# Example: i2c_write_multi 0xB0 0xFC {0x20 0x10 0xC0 0x00}
proc i2c_write_multi {device_id reg_addr byte_list} {
    global axi

    set count [llength $byte_list]
    if {$count < 1 || $count > 8} {
        puts "ERROR: i2c_write_multi supports 1-8 bytes, got $count"
        return -1
    }

    # Set WCOUNT
    set wcnt_hex [format %08X $count]
    create_hw_axi_txn wr_wcnt $axi -type write -address 00000110 -data $wcnt_hex -force
    run_hw_axi [get_hw_axi_txns wr_wcnt]

    # Write register address
    set addr_hex [format %08X [expr {$reg_addr & 0xFF}]]
    create_hw_axi_txn wr_addr $axi -type write -address 00000104 -data $addr_hex -force
    run_hw_axi [get_hw_axi_txns wr_addr]

    # Push bytes into WDATA buffer (each write to 0x108 pushes one byte)
    set idx 0
    foreach byte $byte_list {
        set bval [format %08X [expr {$byte & 0xFF}]]
        create_hw_axi_txn wr_buf$idx $axi -type write -address 00000108 -data $bval -force
        run_hw_axi [get_hw_axi_txns wr_buf$idx]
        incr idx
    }

    # Write CONTROL: bit[31]=0 (write), bits[7:0]=device_id
    # (this also resets the buffer pointer for next time)
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
        puts "ERROR: I2C write_multi timed out (device=0x[format %02X $device_id] addr=0x[format %02X $reg_addr])"
        return -1
    }

    set byte_str ""
    foreach byte $byte_list { append byte_str [format " 0x%02X" $byte] }
    puts [format "I2C WRITE_MULTI: dev=0x%02X addr=0x%02X <= \[%d bytes\]%s" $device_id $reg_addr $count $byte_str]
    return 0
}

# Paged read: sets page via 4-byte write to 0xFC, then reads offset
# Usage: i2c_pread <device_id> <page_addr>
# Example: i2c_pread 0xB0 0xC003  (reads page 0xC0, offset 0x03)
proc i2c_pread {device_id page_addr} {
    set page [expr {($page_addr >> 8) & 0xFF}]
    set offset [expr {$page_addr & 0xFF}]
    i2c_write_multi $device_id 0xFC [list 0x20 0x10 $page 0x00]
    return [i2c_read $device_id $offset]
}

# Paged write: sets page, then writes single byte at offset
# Usage: i2c_pwrite <device_id> <page_addr> <data>
proc i2c_pwrite {device_id page_addr data} {
    set page [expr {($page_addr >> 8) & 0xFF}]
    set offset [expr {$page_addr & 0xFF}]
    i2c_write_multi $device_id 0xFC [list 0x20 0x10 $page 0x00]
    return [i2c_write $device_id $offset $data]
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
puts "  i2c_read        <device_id> <reg_addr>"
puts "  i2c_write       <device_id> <reg_addr> <data>"
puts "  i2c_write_multi <device_id> <reg_addr> <byte_list>"
puts "  i2c_pread       <device_id> <page_addr>"
puts "  i2c_pwrite      <device_id> <page_addr> <data>"
puts ""
puts "Examples:"
puts "  i2c_read  0xB0 0x00                          ;# Read reg 0 (page 0)"
puts "  i2c_write_multi 0xB0 0xFC {0x20 0x10 0xC0 0x00}  ;# Set page 0xC0"
puts "  i2c_pread  0xB0 0xC003                       ;# Read page 0xC0, offset 0x03"
puts "  i2c_pwrite 0xB0 0xC003 0x2F                  ;# Write page 0xC0, offset 0x03"
