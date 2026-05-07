# Renesas RC38612 I2C Page Select Issue

**Date:** 2026-05-03
**Commit:** `f2ca5bc` (i2c_axi_sequencer RTL)

## Background and Goal
Working on programming Renesas RC38612 chip. To verify success of the provided Renesas_I2C_Programming example, we implemented an i2c_axi_sequencer module to enable I2C r/w to the Jitter Cleaner chip. 
Test the I2C read (pre-programming the JC), check against a few known registers (can be found in Timing Commander --> Registers tab)

## To re-create the issue:
`cd tests/Renesas_I2C_Programming/Scripts`
`vivado -mode tcl`
`Vivado% source 1_flash_bitstream.tcl`
`Vivado% source 4_user_i2c.tcl`
`Vivado% i2c_write 0xB0 0xFC 0xC0`
`Vivado% i2c_read 0xB0 0x03`
`Vivado% i2c_write 0xB0 0xFC 0xC1`
`Vivado% i2c_read 0xB0 0x03`

## Problem

The `i2c_axi_sequencer` only supports single-byte writes. The RC38612 page select register (0xFC) requires a **4-byte write** in a single I2C transaction: `[0x20] [0x10] [page] [0x00]`. Our single-byte write doesn't change the page, so all reads return page 0x00 data regardless of the page we attempt to select.

## Test Output

```
Vivado% i2c_read 0xB0 0xC003                                
INFO: [Labtoolstcl 44-481] WRITE DATA is: 00000003                                
INFO: [Labtoolstcl 44-481] WRITE DATA is: 800000B0                                
INFO: [Labtoolstcl 44-481] READ DATA is: c00000b0                                
INFO: [Labtoolstcl 44-481] READ DATA is: 000000c5                                
I2C READ:  dev=0xB0 addr=0xC003 => data=0xC5                                
197                                
Vivado% i2c_write 0xB0 0xFC 0xC0                                
INFO: [Labtoolstcl 44-481] WRITE DATA is: 000000FC                                
INFO: [Labtoolstcl 44-481] WRITE DATA is: 000000C0                                
INFO: [Labtoolstcl 44-481] WRITE DATA is: 000000B0                                
INFO: [Labtoolstcl 44-481] READ DATA is: 400000b0                                
I2C WRITE: dev=0xB0 addr=0xFC <= data=0xC0                                
0                                
Vivado% i2c_read 0xB0 0x03                                
INFO: [Labtoolstcl 44-481] WRITE DATA is: 00000003                                
INFO: [Labtoolstcl 44-481] WRITE DATA is: 800000B0                                
INFO: [Labtoolstcl 44-481] READ DATA is: c00000b0                                
INFO: [Labtoolstcl 44-481] READ DATA is: 000000c5                                
I2C READ:  dev=0xB0 addr=0x03 => data=0xC5                                
197                                
Vivado% i2c_write 0xB0 0xFC 0xC1                                
INFO: [Labtoolstcl 44-481] WRITE DATA is: 000000FC                                
INFO: [Labtoolstcl 44-481] WRITE DATA is: 000000C1                                
INFO: [Labtoolstcl 44-481] WRITE DATA is: 000000B0                                
INFO: [Labtoolstcl 44-481] READ DATA is: 400000b0                                
I2C WRITE: dev=0xB0 addr=0xFC <= data=0xC112,0-1All1All
0
Vivado% i2c_read 0xB0 0x03
INFO: [Labtoolstcl 44-481] WRITE DATA is: 00000003
INFO: [Labtoolstcl 44-481] WRITE DATA is: 800000B0
INFO: [Labtoolstcl 44-481] READ DATA is: c00000b0
INFO: [Labtoolstcl 44-481] READ DATA is: 000000c5
I2C READ:  dev=0xB0 addr=0x03 => data=0xC5
197
```

Expected: 0xC003 = 0x2F; 0xC103 = 0x00
          
Actual: still reading page 0x00 register 0x03 (0xC5) because page select didn't take effect.

## Root Cause

The `.coe` bulk sequencer writes to 0xFC with SIZE=4:
```
04fc, 0820, 0810, 08C0, 1000  →  I2C: [START][0xB0][0xFC][0x20][0x10][0xC0][0x00+STOP]
```

The user path (`i2c_axi_sequencer.v`) only sends 1 data byte + STOP:
```
I2C: [START][0xB0][0xFC][0xC1+STOP]
```

The chip doesn't recognize this as a valid page select command.

## Next Steps

- Modify `i2c_axi_sequencer.v` to support multi-byte writes (add WCOUNT register + byte buffer)
- Alternatively: add a dedicated page-select command path


This was fixed in commit `2b3b3422`

