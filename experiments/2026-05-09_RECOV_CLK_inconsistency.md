# Renesas RC38612 I2C Programming (And Perl Script Bug)

**Date:** 2026-05-09
**Commit:** `0c45060`

## Background and Goal

Trying to recreate results of ported *RECOV_CLK* example. It was run on a previous day with success (FIFO Wr Count == FIFO Rd Count && FIFO ERR Count == 0). Howver, rerunning it caused FIFO Wr Count == 0.

## To Reproduce

Open Vivado from clean slate and run RECOV_CLK example
```bash
cd <path to repo>/tests/RECOV_CLK/scripts/HwMgr
/tool/Xilinx/Vivado/2024.2/bin/vivado
```
Connect to hw_server
Flash recov_clk.bin/ltx to device
```tcl
# From here on out in TCL shell
source runme.tcl

setup
# ... log for setup (reset GTF etc)

runme
#
# Running Data Stream...
#
Num Channels     = 4
Num Cycles       = 1
Num Frames/Cycle = 0 (cont.)
Cycle Delay(ms)  = 5000
Cycle 1 Channel 0, Packets = 78409653, FIFO Wr Count = 588055639, FIFO RD Count = 588055639, FIFO ERR Count = 0
Cycle 1 Channel 1, Packets = 78409653, FIFO Wr Count = 588055496, FIFO RD Count = 588055496, FIFO ERR Count = 0
Cycle 1 Channel 2, Packets = 78409653, FIFO Wr Count = 588055534, FIFO RD Count = 588055534, FIFO ERR Count = 0
Cycle 1 Channel 3, Packets = 78409653, FIFO Wr Count = 588055650, FIFO RD Count = 588055650, FIFO ERR Count = 0
#
# Complete...
#
```
Here the test works normally. To corrupt follow next steps:
- Load on QSFP test bitstream
- source qsfp.tcl
- run disable_qsfp_power
- reflash recov_clk.bit/ltx
- rerun setup + runme
```
#
# Running Data Stream...
#
Num Channels     = 4
Num Cycles       = 1
Num Frames/Cycle = 0 (cont.)
Cycle Delay(ms)  = 5000
Cycle 1 Channel 0, Packets = 0, FIFO Wr Count = 0, FIFO RD Count = 0, FIFO ERR Count = 0
Cycle 1 Channel 1, Packets = 0, FIFO Wr Count = 0, FIFO RD Count = 0, FIFO ERR Count = 0
Cycle 1 Channel 2, Packets = 0, FIFO Wr Count = 0, FIFO RD Count = 0, FIFO ERR Count = 0
Cycle 1 Channel 3, Packets = 0, FIFO Wr Count = 0, FIFO RD Count = 0, FIFO ERR Count = 0
#
# Complete...
#
```

## The Fix

When running RECOV_CLK, run from fresh vivado shell/gui. There probably is a namespace collision or stale variable. Close Vivado then rerun test (reflash device, resource runme.tcl).
