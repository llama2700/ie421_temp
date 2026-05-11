/*
Copyright (c) 2023, Advanced Micro Devices, Inc. All rights reserved.
SPDX-License-Identifier: MIT
*/


// --------------------------------------------------------------
//
//    GTF Signals...
//

wire [0:0] qsfpdd2_tx_n;
wire [0:0] qsfpdd2_tx_p;
wire [0:0] qsfpdd2_rx_n;
wire [0:0] qsfpdd2_rx_p;

wire [0:0] qsfpdd1_tx_n;
wire [0:0] qsfpdd1_tx_p;
wire [0:0] qsfpdd1_rx_n;
wire [0:0] qsfpdd1_rx_p;


// --------------------------------------------------------------
//
//    Renesas Signals...
//

wire jitt_resetn ; pullup( jitt_resetn );

wire jitt1_gpoi5 ; pullup( jitt1_gpoi5 );
wire jitt1_gpoi4 ; pullup( jitt1_gpoi4 );
wire jitt1_gpoi3 ; pullup( jitt1_gpoi3 );
wire jitt1_gpoi2 ; pullup( jitt1_gpoi2 );
wire jitt1_gpoi1 ; pullup( jitt1_gpoi1 );
wire jitt1_gpoi0 ; pullup( jitt1_gpoi0 );


// --------------------------------------------------------------
//
//    QSFP Signals...
//

wire fpga_mux_rstn      ; pullup( fpga_mux_rstn      );
wire qsfpdd1_io_reset_b ; pullup( qsfpdd1_io_reset_b );
wire qsfpdd2_io_reset_b ; pullup( qsfpdd2_io_reset_b );


// --------------------------------------------------------------
//
//    I2C Signals...
//

wire clkgen_sda_r ; pullup( clkgen_sda_r );
wire clkgen_scl_r ; pullup( clkgen_scl_r );

wire fpga_sda_r ; pullup( fpga_sda_r );
wire fpga_scl_r ; pullup( fpga_scl_r );
