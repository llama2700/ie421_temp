#!/usr/bin/env python3
"""
Generate lfsr_flat.v — a drop-in replacement for Corundum's lfsr.v.

Instead of using a Verilog function with nested loops this emits 
explicit XOR assign statements for each output bit, pre-computed 
by running the same lfsr_mask algorithm in Python.

Generates all configurations used by ll_eth_mac_pcs:
  - 58-bit descrambler (FIBONACCI, FF=1, REV=1, DW=64)
  - 58-bit scrambler   (FIBONACCI, FF=0, REV=1, DW=64)
  - CRC-32 checker     (GALOIS, FF=0, REV=1, DW=8..64 in steps of 8)
"""

import sys
from collections import namedtuple

Config = namedtuple("Config", [
    "lfsr_width", "lfsr_poly", "lfsr_config",
    "lfsr_feed_forward", "reverse", "data_width"
])

# All configurations used by ll_eth_mac_pcs
CONFIGS = [
    # RX descrambler
    Config(58, 0x8000000001, "FIBONACCI", 1, 1, 64),
    # TX scrambler
    Config(58, 0x8000000001, "FIBONACCI", 0, 1, 64),
    # CRC-32 for data widths 8..64
    Config(32, 0x04c11db7, "GALOIS", 0, 1, 8),
    Config(32, 0x04c11db7, "GALOIS", 0, 1, 16),
    Config(32, 0x04c11db7, "GALOIS", 0, 1, 24),
    Config(32, 0x04c11db7, "GALOIS", 0, 1, 32),
    Config(32, 0x04c11db7, "GALOIS", 0, 1, 40),
    Config(32, 0x04c11db7, "GALOIS", 0, 1, 48),
    Config(32, 0x04c11db7, "GALOIS", 0, 1, 56),
    Config(32, 0x04c11db7, "GALOIS", 0, 1, 64),
]


def compute_masks(cfg):
    """
    Replicate the lfsr_mask function from Corundum's lfsr.v in Python.
    Returns (state_masks, data_masks) where each is a list of
    (state_part, data_part) integer bitmasks, one per output bit.
    Output bits 0..LFSR_WIDTH-1 are state_out, LFSR_WIDTH..end are data_out.
    """
    W = cfg.lfsr_width
    D = cfg.data_width
    POLY = cfg.lfsr_poly

    # Bit-mask arrays: each element is an integer bitmask
    # lms[i] = which state_in bits contribute to LFSR position i
    # lmd[i] = which data_in bits contribute to LFSR position i
    lms = [0] * W
    lmd = [0] * W
    oms = [0] * D  # output_mask_state
    omd = [0] * D  # output_mask_data

    for i in range(W):
        lms[i] = 1 << i
    for i in range(D):
        if i < W:
            oms[i] = 1 << i

    if cfg.lfsr_config == "FIBONACCI":
        # Process data bits MSB first
        for bit_idx in range(D - 1, -1, -1):
            data_mask = 1 << bit_idx

            # Feedback from last FF XOR with data bit
            sv = lms[W - 1]
            dv = lmd[W - 1] ^ data_mask

            # XOR with polynomial taps
            for j in range(1, W):
                if (POLY >> j) & 1:
                    sv ^= lms[j - 1]
                    dv ^= lmd[j - 1]

            # Shift LFSR
            for j in range(W - 1, 0, -1):
                lms[j] = lms[j - 1]
                lmd[j] = lmd[j - 1]

            # Shift output
            for j in range(D - 1, 0, -1):
                oms[j] = oms[j - 1]
                omd[j] = omd[j - 1]

            oms[0] = sv
            omd[0] = dv

            if cfg.lfsr_feed_forward:
                lms[0] = 0
                lmd[0] = data_mask
            else:
                lms[0] = sv
                lmd[0] = dv

    elif cfg.lfsr_config == "GALOIS":
        for bit_idx in range(D - 1, -1, -1):
            data_mask = 1 << bit_idx

            sv = lms[W - 1]
            dv = lmd[W - 1] ^ data_mask

            # Shift first
            for j in range(W - 1, 0, -1):
                lms[j] = lms[j - 1]
                lmd[j] = lmd[j - 1]

            for j in range(D - 1, 0, -1):
                oms[j] = oms[j - 1]
                omd[j] = omd[j - 1]

            oms[0] = sv
            omd[0] = dv

            if cfg.lfsr_feed_forward:
                lms[0] = 0
                lmd[0] = data_mask
            else:
                lms[0] = sv
                lmd[0] = dv

            # XOR at polynomial tap positions
            for j in range(1, W):
                if (POLY >> j) & 1:
                    lms[j] ^= sv
                    lmd[j] ^= dv

    # Apply REVERSE if needed
    results = []  # list of (state_mask, data_mask) per output bit
    total = W + D

    for n in range(total):
        if cfg.reverse:
            if n < W:
                src_s = lms[W - n - 1]
                src_d = lmd[W - n - 1]
            else:
                src_s = oms[D - (n - W) - 1]
                src_d = omd[D - (n - W) - 1]
            # Bit-reverse within each mask
            s = 0
            for i in range(W):
                if (src_s >> (W - 1 - i)) & 1:
                    s |= 1 << i
            d = 0
            for i in range(D):
                if (src_d >> (D - 1 - i)) & 1:
                    d |= 1 << i
        else:
            if n < W:
                s = lms[n]
                d = lmd[n]
            else:
                s = oms[n - W]
                d = omd[n - W]
        results.append((s, d))

    return results


def bits_set(val, width):
    """Return list of bit positions that are set."""
    return [i for i in range(width) if (val >> i) & 1]


def xor_expr(state_bits, data_bits):
    """Build a Verilog XOR expression from lists of bit indices."""
    terms = [f"state_in[{b}]" for b in state_bits]
    terms += [f"data_in[{b}]" for b in data_bits]
    if not terms:
        return "1'b0"
    return " ^ ".join(terms)


def generate_config_block(cfg, masks, indent="        "):
    """Generate assign statements for one configuration."""
    W = cfg.lfsr_width
    D = cfg.data_width
    lines = []

    # state_out assignments
    for n in range(W):
        s_bits = bits_set(masks[n][0], W)
        d_bits = bits_set(masks[n][1], D)
        expr = xor_expr(s_bits, d_bits)
        lines.append(f"{indent}assign state_out[{n}] = {expr};")

    # data_out assignments
    for n in range(D):
        s_bits = bits_set(masks[W + n][0], W)
        d_bits = bits_set(masks[W + n][1], D)
        expr = xor_expr(s_bits, d_bits)
        lines.append(f"{indent}assign data_out[{n}] = {expr};")

    return "\n".join(lines)


def config_match_expr(cfg):
    """Generate a Verilog expression matching this configuration's parameters."""
    poly_str = f"{cfg.lfsr_width}'h{cfg.lfsr_poly:x}"
    config_str = '"FIBONACCI"' if cfg.lfsr_config == "FIBONACCI" else '"GALOIS"'
    return (
        f"LFSR_WIDTH == {cfg.lfsr_width} && "
        f"LFSR_POLY == {poly_str} && "
        f"LFSR_CONFIG == {config_str} && "
        f"LFSR_FEED_FORWARD == {cfg.lfsr_feed_forward} && "
        f"REVERSE == {cfg.reverse} && "
        f"DATA_WIDTH == {cfg.data_width}"
    )


def generate_module():
    """Generate the complete lfsr_flat.v module."""
    header = """\
/*
 * Pre-computed parallel LFSR/CRC — drop-in replacement for lfsr.v
 *
 * Generated by gen_lfsr_flat.py.  Uses explicit XOR assign statements
 *
 * Same port interface as Corundum's lfsr.v.
 */

`resetall
`timescale 1ns / 1ps
`default_nettype none

module lfsr #(
    parameter LFSR_WIDTH = 31,
    parameter LFSR_POLY = 31'h10000001,
    parameter LFSR_CONFIG = "FIBONACCI",
    parameter LFSR_FEED_FORWARD = 0,
    parameter REVERSE = 0,
    parameter DATA_WIDTH = 8,
    parameter STYLE = "AUTO"
)(
    input  wire [DATA_WIDTH-1:0] data_in,
    input  wire [LFSR_WIDTH-1:0] state_in,
    output wire [DATA_WIDTH-1:0] data_out,
    output wire [LFSR_WIDTH-1:0] state_out
);

generate
"""

    footer = """\
    // Fallback: unsupported configuration
    else begin : gen_unsupported
        // synthesis translate_off
        initial begin
            $display("ERROR: lfsr_flat does not support this parameter combination");
            $finish;
        end
        // synthesis translate_on
        assign state_out = {LFSR_WIDTH{1'b0}};
        assign data_out = {DATA_WIDTH{1'b0}};
    end
endgenerate

endmodule

`resetall
"""

    body_parts = []
    for i, cfg in enumerate(CONFIGS):
        print(f"Computing masks for {cfg.lfsr_config} W={cfg.lfsr_width} "
              f"DW={cfg.data_width} FF={cfg.lfsr_feed_forward}...",
              file=sys.stderr)
        masks = compute_masks(cfg)
        assigns = generate_config_block(cfg, masks)

        keyword = "if" if i == 0 else "else if"
        cond = config_match_expr(cfg)
        label = (f"gen_{cfg.lfsr_config.lower()}_w{cfg.lfsr_width}"
                 f"_dw{cfg.data_width}_ff{cfg.lfsr_feed_forward}")

        body_parts.append(
            f"    {keyword} ({cond}) begin : {label}\n"
            f"{assigns}\n"
            f"    end"
        )

    body = "\n".join(body_parts)
    return header + body + "\n" + footer


if __name__ == "__main__":
    verilog = generate_module()
    outfile = "lfsr_flat.v"
    if len(sys.argv) > 1:
        outfile = sys.argv[1]
    with open(outfile, "w") as f:
        f.write(verilog)
    print(f"Generated {outfile}", file=sys.stderr)
