# GTF RAW wizard regeneration (UL3422, Bank 127 / QSFPDD1)

This directory holds the GTF wizard example design generated for the RAW GTF
(QSFPDD1, GT Bank 127 on UL3422). The shipped tree is empty -- regenerate it
in Vivado before sourcing `Vivado_Project/setup.tcl`.

## Steps

1. In Vivado, launch the GTF Wizard with target part `xcvu2p-fsvj2104-3-e`.
2. On the Basic tab set:
   - Transceiver configuration preset: `GTF-10G-RAW`
   - All other preset settings unchanged
3. On the Physical Resources tab select the channels of GT Bank 127 that map
   to QSFPDD1 lanes 1..4 (4 channels). Coordinates are visible in the Vivado
   IP customization GUI.
4. Set the IP module name to `gtfwizard_raw` and generate.
5. Right-click the generated IP -> Open IP Example Design. Copy the resulting
   `imports/` and `ip/` subtrees here under `RTL/gtfwizard_raw_ex/`.

The RAW wrapper does not export a recovered clock to a board pin, so no hand
patch is required for this wizard.

## pblock guidance

After the first synthesis run, fill in the `pblock_bank_127` constraints in
`XDC/constraint.xdc` (currently commented out) with the GTF tile coordinates
captured from Vivado.
