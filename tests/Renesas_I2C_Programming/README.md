# Renesas I2C Programming

Refer to the [Alveo-Cards](https://github.com/Xilinx/Alveo-Cards/tree/ul3422) repo for the full reference design documentation.

## Overview

Programs the Renesas RC38612 jitter cleaner chip over I2C from the FPGA. Two programming paths exist:

1. **BRAM sequencer** — bulk programs the chip at boot from a `.coe` file baked into the bitstream
2. **User I2C** — interactive single-transaction read/write via JTAG-AXI (Vivado Tcl console)

## Changes from Reference Design

- Vivado 2024.1
- Added user I2C path (`reg_i2c_user_logic.v`, `i2c_axi_sequencer.v` multi-byte write support)
- Updated `.coe` file

## RC38612 Page Register (Offset 0xFC)

The RC38612 uses a **paged register architecture**. In 1-byte (1B) addressing mode:

- The lower 8 bits of a register address come from the **offset byte** (0x00–0xFB)
- The upper 8 bits come from the **page register**
- The page register is a special 4-byte register at offset **0xFC**
- It must be written as a **single 4-byte I2C burst** write

### Page Register Format

From the RC38612 datasheet, the 4-byte write to offset 0xFC is:

```
I2C wire:  [SlaveAddr+W] [0xFC] [0x00] [page] [0x10] [0x20]
                          ^addr  ^FC    ^FD     ^FE    ^FF
```

| Byte | Offset | Value | Purpose |
|------|--------|-------|---------|
| 0 | FC | 0x00 | Fixed |
| 1 | FD | `<page>` | Page number (e.g., 0xCB) |
| 2 | FE | 0x10 | Fixed |
| 3 | FF | 0x20 | Fixed |

### Example: Access Register 0xCBE4

```
# Set page to 0xCB
B6 FC 00 CB 10 20       (I2C address 0x5B << 1 = 0xB6 for write)

# Write 0x50 to offset 0xE4 (register 0xCBE4)
B6 E4 50

# Read offset 0x24 after setting page to 0xC0 (register 0xC024)
B6 FC 00 C0 10 20       Set page
B6 24                   Set pointer (no stop)
B7 <data>               Read with address bit set
```

## User I2C Path — Byte Flow

### Architecture

```
Vivado Tcl (hw_axi_txn)
  │
  ▼
jtag_axi_0  ──AXI-Lite──►  reg_axi_slave  ──sys_if──►  reg_i2c_user_logic
                                                              │
                                                         IO_WDATA_BUF[63:0]
                                                         IO_WCOUNT, IO_ADDR, etc.
                                                              │
                                                              ▼
                                                        i2c_axi_sequencer
                                                              │
                                                         AXI IIC TX FIFO writes
                                                              │
                                                              ▼
                                                          axi_iic_0
                                                              │
                                                          I2C bus
                                                              │
                                                              ▼
                                                        RC38612 (0x5B)
```

### Byte Buffer Ordering

When `i2c_write_multi` pushes bytes to the WDATA register (0x108), each write increments
an internal pointer (`wbuf_ptr`) that fills `IO_WDATA_BUF` from LSB to MSB:

```
Push order:     byte[0]  byte[1]  byte[2]  byte[3]  ...
Buffer bits:    [7:0]    [15:8]   [23:16]  [31:24]  ...
```

The `i2c_axi_sequencer` reads bytes back in the same order (`byte_idx` 0,1,2,3...),
writing each to the AXI IIC TX FIFO. **There is no byte reversal in hardware.**
Bytes appear on the I2C wire in the exact order they are pushed from Tcl.

### Tcl Usage

```tcl
source 4_user_i2c.tcl

# Page register write — bytes match Renesas datasheet wire order
i2c_write_multi 0xB0 0xFC {0x00 0xCF 0x10 0x20}    ;# Select page 0xCF

# Convenience wrappers (page_addr = 0xPPOO where PP=page, OO=offset)
i2c_pread  0xB0 0xCF54         ;# Read page 0xCF, offset 0x54
i2c_pwrite 0xB0 0xCF54 0x0A   ;# Write 0x0A to page 0xCF, offset 0x54
```

## BRAM Sequencer Path — Byte Order Issue

### How the .coe is generated

The Perl script `txt_to_mem.pl` converts Renesas Timing Commander register export files
(`.txt`) into Vivado BRAM `.coe` files. The register file stores multi-byte data as hex
strings:

```
Size: 0x4, Offset: FC, Data: 0x00C01020
```

**The Perl script extracts bytes from right to left** (LSB of the hex number first):

```perl
$temp = substr($data, $len1-$ii-2, 2);   # starts from rightmost byte
```

For `0x00C01020`, this produces bytes: `0x20, 0x10, 0xC0, 0x00` — which is the
**reverse** of the natural hex string reading order.

### The reversal is wrong for I2C wire order

The Renesas datasheet specifies the page register wire order as `{0x00 <page> 0x10 0x20}`.
The Perl script reversal produces `{0x20 0x10 <page> 0x00}`, which places the page byte
at FC+2 (register FE) instead of FC+1 (register FD).

This means the BRAM sequencer's page register writes are incorrect. However, this was not
caught during testing because the example `.coe` file programs **config 15**, which is
the same configuration the RC38612 loads from OTP on power-up (GPIO[3:0] = 4'b1111 on
the UL3422 board). The BRAM programming was effectively a no-op.

### Implication

The `txt_to_mem.pl` Perl script byte order needs to be fixed before the BRAM sequencer
can correctly program a non-default configuration. The data bytes in the hex string should
be sent in natural reading order (left to right), not reversed.

## OTP Configuration

The RC38612 has internal one-time programmable (OTP) memory with 4 predefined
configurations (12–15). The configuration loaded at power-up is selected by GPIO[3:0]:

| GPIO[3:0] | Configuration |
|-----------|---------------|
| 4'b1100 | Config 12 |
| 4'b1101 | Config 13 |
| 4'b1110 | Config 14 |
| 4'b1111 | Config 15 (default) |

On the UL3422 board, GPIO[3:0] are pulled high → **config 15 loads on every power-up**.

The GPIO[3:0] pins (`jitt1_gpoi0`–`jitt1_gpoi3`) and the reset pin (`jitt_resetn`) are
defined in the XDC but **not connected in the current RTL**. To change the OTP config
at runtime, these pins would need to be added as FPGA outputs, the GPIO driven to the
desired config, and `jitt_resetn` toggled (the config is latched on reset deassertion).

## Script Execution Order

```
1_flash_bitstream.tcl    — Program FPGA, connect debug cores
3_enable_renesas_i2c.tcl — Trigger BRAM sequencer (VIO reset toggle)
4_user_i2c.tcl           — Load interactive I2C procs (i2c_read, i2c_pread, etc.)
2_read_freq_counters.tcl — Read jitter cleaner output frequencies
```
