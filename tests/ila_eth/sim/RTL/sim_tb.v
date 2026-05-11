/*
Copyright (c) 2023, Advanced Micro Devices, Inc. All rights reserved.
SPDX-License-Identifier: MIT
*/

`timescale 1 ps / 1 ps

`define PERIOD_300MHZ_PS  1667
`define PERIOD_161MHZ_PS  3106
`define PERIOD_100MHZ_PS  5000
`define PERIOD_33MHZ_PS  15152
`define PERIOD_25MHZ_PS  20000
`define PERIOD_18MHZ_PS  27778

`define PERIOD_80MHZ_PS  6250
`define PERIOD_70MHZ_PS  7143
`define PERIOD_60MHZ_PS  8333
`define PERIOD_50MHZ_PS  10000
`define PERIOD_40MHZ_PS  12500
`define PERIOD_30MHZ_PS  16664
`define PERIOD_20MHZ_PS  25000
`define PERIOD_10MHZ_PS  50000

module sim_tb ();
localparam MODULE_NAME = "sim_tb";

// -----------------------------------------------------------
//
//    Signal Connectivity...
//
// -----------------------------------------------------------

wire jtag_m_axi_aclk   ;
wire jtag_m_axi_aresetn;
`include "sim_wires.sv"

// -----------------------------------------------------------
//
// Clocks and resets...
//
// -----------------------------------------------------------

`include "sim_clk_reset.sv"

// -----------------------------------------------------------
//
// AXI Register Definitions...
//
// -----------------------------------------------------------

//`include "system/sim_tb_addr.v"

// --------------------------------------------------------------

integer ii;
integer file_coe;
integer temp;

string         string0;
reg [8*16-1:0] string1;

function [31:0] hex_str_to_int2;
    input string temp1;
    string temp2;
    begin
        temp2 = temp1.substr(0,4);
        hex_str_to_int2 = temp2.atohex();
    end
endfunction


// -----------------------------------------------------------
//
//    AXI Tasks and Master AXI Driver
//
// -----------------------------------------------------------

`include "sim_axi_driver.sv"


// -----------------------------------------------------------
//
//    Dummy Renesas I2C Peripherals...
//
// -----------------------------------------------------------

`include "renesas_i2c/renesas_i2c.vh"


// -----------------------------------------------------------
//
//    Dummy QSFP I2C Peripherals...
//
// -----------------------------------------------------------

//`include "qsfp_i2c/qsfp_i2c.vh"


// -----------------------------------------------------------
//
//    GTF Config and Execute Sequences...
//
// -----------------------------------------------------------

`include "gtf_tb.sv"


// -----------------------------------------------------------
//
//    DUT...
//
// -----------------------------------------------------------

clk_recov  #(
    .SIMULATION("true"),
    .NUM_CHANNEL(1)
) clk_recov (
    // MAC GTF -- QSFPDD2 (Bank 130)
    .qsfpdd2_tx_n           ( qsfpdd2_tx_n        ),
    .qsfpdd2_tx_p           ( qsfpdd2_tx_p        ),
    .qsfpdd2_rx_n           ( qsfpdd1_tx_n        ),
    .qsfpdd2_rx_p           ( qsfpdd1_tx_p        ),

    // RAW GTF -- QSFPDD1 (Bank 127)
    .qsfpdd1_tx_n           ( qsfpdd1_tx_n        ),
    .qsfpdd1_tx_p           ( qsfpdd1_tx_p        ),
    .qsfpdd1_rx_n           ( qsfpdd2_tx_n        ),
    .qsfpdd1_rx_p           ( qsfpdd2_tx_p        ),

    // 300 MHz LVDS reference clocks
    .clk_ddr_lvds_300_p     ( refclk_300          ),
    .clk_ddr_lvds_300_n     ( ~refclk_300         ),
    .clk_sys_lvds_300_p     ( refclk_300          ),
    .clk_sys_lvds_300_n     ( ~refclk_300         ),

    // GTF MGTREFCLK0 inputs
    .synce_clk_127_lvds_p   ( SYNCE_CLK_LVDS_P    ),
    .synce_clk_127_lvds_n   ( ~SYNCE_CLK_LVDS_P   ),
    .synce_clk_128_lvds_p   ( SYNCE_CLK_LVDS_P    ),
    .synce_clk_128_lvds_n   ( ~SYNCE_CLK_LVDS_P   ),
    .synce_clk_129_lvds_p   ( SYNCE_CLK_LVDS_P    ),
    .synce_clk_129_lvds_n   ( ~SYNCE_CLK_LVDS_P   ),
    .synce_clk_130_lvds_p   ( SYNCE_CLK_LVDS_P    ),
    .synce_clk_130_lvds_n   ( ~SYNCE_CLK_LVDS_P   ),
    .synce_clk_131_lvds_p   ( SYNCE_CLK_LVDS_P    ),
    .synce_clk_131_lvds_n   ( ~SYNCE_CLK_LVDS_P   ),

    // Bank 65 fabric SYNCE inputs
    .synce_clk_65_1_lvds_p  ( SYNCE_CLK_LVDS_P    ),
    .synce_clk_65_1_lvds_n  ( ~SYNCE_CLK_LVDS_P   ),
    .synce_clk_65_2_lvds_p  ( SYNCE_CLK_LVDS_P    ),
    .synce_clk_65_2_lvds_n  ( ~SYNCE_CLK_LVDS_P   ),

    // Recovered clock output
    .recov_clk_65_lvds_p    (                     ),
    .recov_clk_65_lvds_n    (                     ),

    // Renesas I2C
    .clkgen_sda_r           ( clkgen_sda_r        ),
    .clkgen_scl_r           ( clkgen_scl_r        ),

    // Renesas GPIO and Resetn
    .jitt_resetn            ( jitt_resetn         ),
    .jitt1_gpoi5            ( jitt1_gpoi5         ),
    .jitt1_gpoi4            ( jitt1_gpoi4         ),
    .jitt1_gpoi3            ( jitt1_gpoi3         ),
    .jitt1_gpoi2            ( jitt1_gpoi2         ),
    .jitt1_gpoi1            ( jitt1_gpoi1         ),
    .jitt1_gpoi0            ( jitt1_gpoi0         ),

    // QSFP related signals
    .fpga_mux_rstn          ( fpga_mux_rstn       ),
    .qsfpdd1_io_reset_b     ( qsfpdd1_io_reset_b  ),
    .qsfpdd2_io_reset_b     ( qsfpdd2_io_reset_b  ),

    .fpga_sda_r             ( fpga_sda_r          ),
    .fpga_scl_r             ( fpga_scl_r          )
);

// --------------------------------------------------------------

endmodule
