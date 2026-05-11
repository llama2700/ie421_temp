# GTF MAC wizard regeneration (UL3422, Bank 130 / QSFPDD2)

This directory holds the GTF wizard example design generated for the MAC GTF
(QSFPDD2, GT Bank 130 on UL3422). The shipped tree is empty -- regenerate it
in Vivado before sourcing `Vivado_Project/setup.tcl`.

## Steps

1. In Vivado, launch the GTF Wizard with target part `xcvu2p-fsvj2104-3-e`.
2. On the Basic tab set:
   - Transceiver configuration preset: `GTF-10G-MAC`
   - All other preset settings unchanged
3. On the Physical Resources tab select the channels of GT Bank 130 that map
   to QSFPDD2 lanes 1..4 (4 channels). The exact `GTF_CHANNEL_X<a>Y<c>`
   coordinates can be read directly from the Vivado IP customization GUI
   for the target part, or looked up in the FPGA's package pinout reference.
4. Set the IP module name to `gtfwizard_mac` and generate.
5. Right-click the generated IP -> Open IP Example Design. Copy the resulting
   `imports/`, `gtfwizard_mac/`, and `ip/` subtrees here under
   `RTL/gtfwizard_mac_ex/`.

## Recovered clock to pin patch

The example design routes `gtf_ch_rxoutclk[i]` through a `BUFG_GT` (DIV=4),
then to an `ODDRE1`, then to an `OBUFTDS` driving `RECOV_CLK10_LVDS_P/N`. On
ul3524 this lives at `gtfwizard_mac_ex/imports/gtfwizard_mac_fab_wrap.v`
around lines 1437..1644. The freshly regenerated wizard does not include
this -- copy the same block in:

```
localparam RECOV_CLK_SEL = 0;
assign RECOV_CLK10_INT = gtf_recov_clk[RECOV_CLK_SEL];
wire gtf_recov_clk_i;

ODDRE1 #(
    .SIM_DEVICE ("ULTRASCALE_PLUS")
) u_oddre1_gtf_recov_clk (
    .C   ( RECOV_CLK10_INT ),
    .D1  ( 1'b1            ),
    .D2  ( 1'b0            ),
    .Q   ( gtf_recov_clk_i ),
    .SR  ( 1'b0            )
);

OBUFTDS u_obuftds_gtf_recov_clk (
    .I  ( gtf_recov_clk_i    ),
    .T  ( 1'b0               ),
    .O  ( RECOV_CLK10_LVDS_P ),
    .OB ( RECOV_CLK10_LVDS_N )
);
```

The `BUFG_GT` (with `DIV=3'd3` for divide-by-4) feeding `gtf_recov_clk[i]`
should already be inside the per-channel generate block; if not, follow the
ul3524 reference. Add `RECOV_CLK10_INT`, `RECOV_CLK10_LVDS_P`,
`RECOV_CLK10_LVDS_N` as wire/output ports up the wizard hierarchy until they
reach `gtf_top_0`.

## pblock guidance

`BUFG_GT` placement defaults can land in the wrong clock region. After the
first synthesis run, capture the GTF tile coordinates from the Vivado device
view and fill in the `pblock_bank_130` constraints in
`XDC/constraint.xdc` (currently commented out).
