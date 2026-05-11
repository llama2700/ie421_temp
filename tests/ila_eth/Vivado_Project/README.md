# Vivado project

`setup.tcl` builds a Vivado project for the UL3422 RECOV_CLK reference design.
Target part: `xcvu2p-fsvj2104-3-e`.

## Build steps

1. Regenerate the GTF wizard example designs first (see
   `../RTL/gtfwizard_mac_ex/README.md` and `../RTL/gtfwizard_raw_ex/README.md`).
   Apply the recovered-clock-to-pin patch noted in the MAC README.
2. Open Vivado and `cd` to this directory, then `source ./setup.tcl`. The
   script creates `project_1`, adds RTL/sim/XDC sources, and instantiates
   every IP core the design needs.
3. After the first synthesis run, capture the GTF tile X/Y coords from the
   device view and uncomment + fill in the `pblock_bank_127` /
   `pblock_bank_130` constraints in `../XDC/constraint.xdc`.
4. Re-run synthesis and implementation.
