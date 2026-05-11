<table class="sphinxhide" width="100%">
 <tr width="100%">
    <td align="center"><img src="https://raw.githubusercontent.com/Xilinx/Image-Collateral/main/xilinx-logo.png" width="30%"/><h1>UL3422 Ultra Low Latency Trading</h1>
    </td>
 </tr>
</table>

# Recovered Clock Reference Design (UL3422)

## Description

This reference design demonstrates how to set up the UL3422 QSFP-DD Renesas
RC38612 jitter cleaner and route a GTF recovered clock to the
RECOV_CLK_65 LVDS pin in Bank 65. It is the UL3422 port of the original
UL3524 RECOV_CLK example.

Highlights:

* Resetting and programming the Renesas RC38612 jitter cleaner over I2C
* Data generator and checker logic on the MAC GTF
* Loopback FIFO on the RAW GTF
* Clock frequency monitor for all GT bank reference clocks
* Routing a GTF RXOUTCLK to a board pin (Bank 65, BF20/BF19)

**Requirements:** a QSFP-DD loopback module (or cable) on QSFPDD1 to feed the
RAW path back through itself.

**Additional Documentation**

* [Simulation](./Docs/simulation.md)
* [HW Manager Support](./Docs/hw_manager_support.md)

## Architecture overview

The design instantiates two GTF modules sharing one QSFP-DD card:

* **MAC GTF in QSFPDD2 / GT Bank 130** as a data stream generator and checker.
  This GTF sources the recovered clock that is routed to RECOV_CLK_65.
* **RAW GTF in QSFPDD1 / GT Bank 127** in simple loopback mode.

The Renesas Jitter Cleaner (RC38612) feeds the GTF reference clocks
(MGTREFCLK0 of each bank). It is programmed over I2C through `clkgen_scl_r`
/ `clkgen_sda_r` (Bank 65). UL3422 has a single jitter cleaner with six GPIO
pins (`jitt1_gpoi[0:5]`) and one shared reset (`jitt_resetn`); the
`renesas_gpio` module provides the IOBUFs and AXI register window for those.

QSFP power-up and sideband bring-up is handled by the `qsfp_i2c` subsystem
through `fpga_scl_r` / `fpga_sda_r` (Bank 93). Single MUX reset
`fpga_mux_rstn` (G14) covers the I2C MUX. The `qsfp_i2c` state machine
sequences power-on for both QSFPDD1 and QSFPDD2.

The frequency monitor (`freq_counter_top`) samples the GT bank reference
clocks (Banks 127, 128, 129, 130, 131) plus the two Bank 65 fabric SYNCE
inputs. A sample function (`measure_frequency`) in the HW Manager scripts
walks the eight sample registers and prints the measured frequencies.

## GTF placement and recovered clock routing

The MAC GTF (recovered-clock source) sits in:

* Bank 130, channels 0..3 -> QSFPDD2 lanes 1..4
* GT ref clock: `synce_clk_130_lvds_p/n` (MGTREFCLK0 of Bank 130)
* GT recovered clock: routed through `BUFG_GT` (DIV=4) -> `ODDRE1` ->
  `OBUFTDS` -> `recov_clk_65_lvds_p/n` (BF20/BF19, Bank 65)

The RAW GTF sits in:

* Bank 127, channels 0..3 -> QSFPDD1 lanes 1..4
* GT ref clock: `synce_clk_127_lvds_p/n`

The recovered-clock-to-pin chain lives inside the regenerated MAC wizard
example design. See `RTL/gtfwizard_mac_ex/README.md` for the wizard
regeneration steps and the hand-patch instructions.

**NOTE:** the `BUFG_GT` for the recovered clock can land in the wrong clock
region without a pblock pinning it near the GTF. Templates for the
`pblock_bank_127` and `pblock_bank_130` constraints are provided in
`XDC/constraint.xdc` -- fill in the GTF tile X/Y coordinates after the first
synthesis run.

## GTF MAC

Wizard preset: `GTF-10G-MAC`, all other settings default. See
`RTL/gtfwizard_mac_ex/README.md` for regeneration steps targeting
`xcvu2p-fsvj2104-3-e` Bank 130.

## GTF RAW

Wizard preset: `GTF-10G-RAW`, all other settings default. See
`RTL/gtfwizard_raw_ex/README.md`.

## Additional guidance

### Reconfiguring QSFP ports

The MAC and RAW GTF placements can be moved by changing the bank wiring in
`RTL/clk_recov.v` (the `gtf_top_0` and `gtf_top_1` instances) and the
matching constraints in `XDC/constraint.xdc`. Channel count is set by the
`NUM_CHANNEL` parameter in `RTL/clk_recov.v` (default 4). Reducing it may
generate critical warnings for unused logic that can be disregarded.

### Data integrity checking

The MAC GTF wraps the GTF wizard's hwchk core. `IO_SCRATCH_VALUE[8]` enables
the frame generator and `[9]` the monitor. See the original UL3524 README
for the FIFO data-compare diagram (the FIFOs and compare logic are unchanged
on UL3422).

### Frequency measurements

`freq_counter_top` is wired to:

| `clk_samp_*` | source                  |
|:-------------|:------------------------|
| 0            | synce_clk_127 (Bank 127)|
| 1            | synce_clk_128 (Bank 128)|
| 2            | synce_clk_129 (Bank 129)|
| 3            | synce_clk_130 (Bank 130)|
| 4            | synce_clk_131 (Bank 131)|
| 5            | synce_clk_65_1          |
| 6            | synce_clk_65_2          |
| 7            | tied off                |

## Support

For additional documentation, refer to the
[UL3422 product page](https://www.xilinx.com/products/boards-and-kits/alveo/ul3422.html)
and the UL3422 Lounge.

For support, contact your FAE or refer to support resources at:
<https://support.xilinx.com>

<p class="sphinxhide" align="center"><sub>Copyright (C) 2020-2024 Advanced Micro Devices, Inc</sub></p>

<p class="sphinxhide" align="center"><sup><a href="https://www.amd.com/en/corporate/copyright">Terms and Conditions</a></sup></p>
