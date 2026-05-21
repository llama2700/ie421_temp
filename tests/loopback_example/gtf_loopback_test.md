# GTF loopback test

This test achieves near-end loopback on the gtf transceivers
Please refer to Vivado gtf wizard testbench for more info: gtfwizard_0_example_gtfmac_top_tb.sv
This script follows the register write structure of the testbench example

## Necessary Files
* gtfwizard_0_gtfmac_ex.bit
* gtfwizard_0_gtfmac_ex.ltx
* init_gtf.tcl

## How to run
The *init_gtf.tcl* file flashes the bitstream and ltx files, and performs register writes via JTAG-to-AXI to complete the loopback test. Use:
```
vivado -mode batch -source init_gtf.tcl
```

Successful Run:
```
...
PASS: TX == RX == 50                                                                                 
# puts "--- DONE ---"                                                                                
--- DONE ---
```

## Debugging
For debugging purposes, a verbose version of the TCL script is available.
It's recommended that the following command is run to generate a .txt of the output:
```
vivado -mode batch -source init_gtf_verbose.tcl 2>&1 | tee init_gtf_verbose_out.txt
```
The expected verbose output is included for reference.

## ILA Waveforms
Three ILA instances are included to monitor various signals: ila_0, ila_tx, and ila_rx.
The ILA instances use the following clk signals respectively: freerun_clk, tx_axis_clk, rx_axis_clk.

*ila_0 — Status/Reset signals (freerun_clk domain)*
| Probe | Signal Name | Width |
|-------|-------------|-------|
| probe0 | gtf_cm_qpll0_lock | 1 |
| probe1 | gtwiz_reset_tx_done_out[0] | 1 |
| probe2 | gtwiz_reset_rx_done_out[0] | 1 |
| probe3 | gtf_ch_txsyncdone[0] | 1 |
| probe4 | gtf_ch_rxsyncdone[0] | 1 |
| probe5 | rxbuffbypass_complete_flg[0] | 1 |
| probe6 | link_status_out[0] | 1 |
| probe7 | link_maintained[0] | 1 |
| probe8 | link_down_latched_out[0] | 1 |
| probe9 | clk_wiz_locked_out | 1 |
| probe10 | hb_gtwiz_reset_all_in | 1 |
| probe11 | stat_gtf_rx_block_lock[0] | 1 |
| probe12 | tx_axis_rst[0] | 1 |
| probe13 | rx_axis_rst[0] | 1 |

*ila_tx — TX data path (tx_axis_clk domain)*
| Probe | Signal Name | Width |
|-------|-------------|-------|
| probe0 | tx_axis_tdata[63:0] | 64 |
| probe1 | tx_axis_tvalid[0] | 1 |
| probe2 | tx_axis_tready[0] | 1 |
| probe3 | tx_axis_tlast[7:0] | 8 |
| probe4 | tx_axis_tsof[1:0] | 2 |
| probe5 | tx_axis_tterm[4:0] | 5 |
| probe6 | tx_axis_terr[0] | 1 |
| probe7 | tx_axis_tpre[7:0] | 8 |

*ila_rx — RX data path (rx_axis_clk domain)*
| Probe | Signal Name | Width |
|-------|-------------|-------|
| probe0 | rx_axis_tdata[63:0] | 64 |
| probe1 | rx_axis_tvalid[0] | 1 |
| probe2 | rx_axis_tlast[7:0] | 8 |
| probe3 | rx_axis_tsof[1:0] | 2 |
| probe4 | rx_axis_tterm[4:0] | 5 |
| probe5 | rx_axis_terr[0] | 1 |
| probe6 | rx_axis_tpre[7:0] | 8 |
| probe7 | stat_gtf_rx_block_lock[0] | 1 |

Note: the ila_0 trigger isn't set yet. This should be set to get meaningful data on ila_0.