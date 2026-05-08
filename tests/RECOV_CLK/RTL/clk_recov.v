/*
Copyright (c) 2023, Advanced Micro Devices, Inc. All rights reserved.
SPDX-License-Identifier: MIT
*/

`timescale 1 ps / 1 ps

module clk_recov #(
    parameter SIMULATION = "false",
    parameter integer NUM_CHANNEL = 4
) (
    // QSFPDD2 GTF lanes (MAC, recovered clock source) -- Bank 130
    output wire [NUM_CHANNEL-1:0] qsfpdd2_tx_p ,
    output wire [NUM_CHANNEL-1:0] qsfpdd2_tx_n ,
    input  wire [NUM_CHANNEL-1:0] qsfpdd2_rx_p ,
    input  wire [NUM_CHANNEL-1:0] qsfpdd2_rx_n ,

    // QSFPDD1 GTF lanes (RAW, loopback) -- Bank 127
    output wire [NUM_CHANNEL-1:0] qsfpdd1_tx_p ,
    output wire [NUM_CHANNEL-1:0] qsfpdd1_tx_n ,
    input  wire [NUM_CHANNEL-1:0] qsfpdd1_rx_p ,
    input  wire [NUM_CHANNEL-1:0] qsfpdd1_rx_n ,

    // 300 MHz LVDS reference clocks
    input  wire clk_ddr_lvds_300_p ,
    input  wire clk_ddr_lvds_300_n ,
    input  wire clk_sys_lvds_300_p ,
    input  wire clk_sys_lvds_300_n ,

    // GTF MGTREFCLK0 inputs from Renesas jitter cleaner
    input  wire synce_clk_127_lvds_p ,
    input  wire synce_clk_127_lvds_n ,
    input  wire synce_clk_128_lvds_p ,
    input  wire synce_clk_128_lvds_n ,
    input  wire synce_clk_129_lvds_p ,
    input  wire synce_clk_129_lvds_n ,
    input  wire synce_clk_130_lvds_p ,
    input  wire synce_clk_130_lvds_n ,
    input  wire synce_clk_131_lvds_p ,
    input  wire synce_clk_131_lvds_n ,

    // Fabric SYNCE inputs (Bank 65, freq measurement only)
    input  wire synce_clk_65_1_lvds_p ,
    input  wire synce_clk_65_1_lvds_n ,
    input  wire synce_clk_65_2_lvds_p ,
    input  wire synce_clk_65_2_lvds_n ,

    // Recovered clock to I/O (Bank 65, BF20/BF19)
    output wire recov_clk_65_lvds_p ,
    output wire recov_clk_65_lvds_n ,

    // Jitter cleaner I2C
    inout  wire clkgen_scl_r ,
    inout  wire clkgen_sda_r ,

    // Jitter cleaner reset and GPIO (single JC on UL3422)
    inout  wire jitt_resetn  ,
    inout  wire jitt1_gpoi5  ,
    inout  wire jitt1_gpoi4  ,
    inout  wire jitt1_gpoi3  ,
    inout  wire jitt1_gpoi2  ,
    inout  wire jitt1_gpoi1  ,
    inout  wire jitt1_gpoi0  ,

    // QSFP I2C and resets
    inout  wire fpga_sda_r         ,
    inout  wire fpga_scl_r         ,
    output wire fpga_mux_rstn      ,
    output wire qsfpdd1_io_reset_b ,
    output wire qsfpdd2_io_reset_b
);


// ---------------------------------------------------------------
//
//  System Reset
//
// ---------------------------------------------------------------
wire sys_clk_100;
wire sys_rst_100;
wire sys_clk_50 ;
wire sys_rst_50 ;

clk_reset clk_reset (
    .sys_clk_300_p ( clk_ddr_lvds_300_p ),
    .sys_clk_300_n ( clk_ddr_lvds_300_n ),
    .sys_clk_100   ( sys_clk_100   ),
    .sys_rst_100   ( sys_rst_100   ),
    .sys_clk_50    ( sys_clk_50    ),
    .sys_rst_50    ( sys_rst_50    )
);

wire sys_if_clk  = sys_clk_100;
wire sys_if_rstn = ~sys_rst_100;


// -----------------------------------------------------------
//
//  GTF Freerunning Clock
//
// -----------------------------------------------------------
wire    clk_wiz_reset = 1'b0;

wire    clk_sys_lvds_300;
wire    gtf_clk_200mhz;
wire    gtf_clk_425mhz;
wire    gtf_clk_wiz_locked;

IBUFDS ibufds_clk_freerun_inst (
  .I  ( clk_sys_lvds_300_p ),
  .IB ( clk_sys_lvds_300_n ),
  .O  ( clk_sys_lvds_300   )
);

gtfwizard_0_example_clk_wiz clk_wiz_300_to_161_inst
   (
    .clk_in1        ( clk_sys_lvds_300   ),
    .clk_out1       ( gtf_clk_200mhz     ),
    .clk_out2       ( gtf_clk_425mhz     ),
    .locked         ( gtf_clk_wiz_locked )
   );

wire gtf_freerun_clk;
wire gtf_sys_clk_out;

BUFG bufg_clk_freerun_inst (
  .I ( gtf_clk_200mhz  ),
  .O ( gtf_freerun_clk )
);

BUFG bufg_clk_sys_inst (
  .I ( gtf_clk_425mhz  ),
  .O ( gtf_sys_clk_out )
);


// ---------------------------------------------------------------
//
//  JTAG/AXI Interface
//
// ---------------------------------------------------------------

wire           jtag_m_axi_aclk     = sys_if_clk  ;
wire           jtag_m_axi_aresetn  = sys_if_rstn ;
wire [31 : 0]  jtag_m_axi_awaddr   ;
wire [2 : 0]   jtag_m_axi_awprot   ;
wire           jtag_m_axi_awvalid  ;
wire           jtag_m_axi_awready  ;
wire [31 : 0]  jtag_m_axi_wdata    ;
wire [3 : 0]   jtag_m_axi_wstrb    ;
wire           jtag_m_axi_wvalid   ;
wire           jtag_m_axi_wready   ;
wire [1 : 0]   jtag_m_axi_bresp    ;
wire           jtag_m_axi_bvalid   ;
wire           jtag_m_axi_bready   ;
wire [31 : 0]  jtag_m_axi_araddr   ;
wire [2 : 0]   jtag_m_axi_arprot   ;
wire           jtag_m_axi_arvalid  ;
wire           jtag_m_axi_arready  ;
wire [31 : 0]  jtag_m_axi_rdata    ;
wire [1 : 0]   jtag_m_axi_rresp    ;
wire           jtag_m_axi_rvalid   ;
wire           jtag_m_axi_rready   ;


generate
if (SIMULATION == "false") begin
jtag_axi_0 jtag_axi_0 (
    .aclk           ( sys_if_clk         ),
    .aresetn        ( sys_if_rstn        ),

    .m_axi_awaddr   ( jtag_m_axi_awaddr  ),
    .m_axi_awprot   ( jtag_m_axi_awprot  ),
    .m_axi_awvalid  ( jtag_m_axi_awvalid ),
    .m_axi_awready  ( jtag_m_axi_awready ),
    .m_axi_wdata    ( jtag_m_axi_wdata   ),
    .m_axi_wstrb    ( jtag_m_axi_wstrb   ),
    .m_axi_wvalid   ( jtag_m_axi_wvalid  ),
    .m_axi_wready   ( jtag_m_axi_wready  ),
    .m_axi_bresp    ( jtag_m_axi_bresp   ),
    .m_axi_bvalid   ( jtag_m_axi_bvalid  ),
    .m_axi_bready   ( jtag_m_axi_bready  ),
    .m_axi_araddr   ( jtag_m_axi_araddr  ),
    .m_axi_arprot   ( jtag_m_axi_arprot  ),
    .m_axi_arvalid  ( jtag_m_axi_arvalid ),
    .m_axi_arready  ( jtag_m_axi_arready ),
    .m_axi_rdata    ( jtag_m_axi_rdata   ),
    .m_axi_rresp    ( jtag_m_axi_rresp   ),
    .m_axi_rvalid   ( jtag_m_axi_rvalid  ),
    .m_axi_rready   ( jtag_m_axi_rready  )
);
end
endgenerate


// ---------------------------------------------------------------
//
//  System AXI Interconnect
//
// ---------------------------------------------------------------

wire         M_AXI_0_aclk     ;
wire         M_AXI_0_aresetn  ;
wire [31:0]  M_AXI_0_araddr   ;
wire [2:0]   M_AXI_0_arprot   ;
wire         M_AXI_0_arready  ;
wire         M_AXI_0_arvalid  ;
wire [31:0]  M_AXI_0_awaddr   ;
wire [2:0]   M_AXI_0_awprot   ;
wire         M_AXI_0_awready  ;
wire         M_AXI_0_awvalid  ;
wire         M_AXI_0_bready   ;
wire [1:0]   M_AXI_0_bresp    ;
wire         M_AXI_0_bvalid   ;
wire [31:0]  M_AXI_0_rdata    ;
wire         M_AXI_0_rready   ;
wire [1:0]   M_AXI_0_rresp    ;
wire         M_AXI_0_rvalid   ;
wire [31:0]  M_AXI_0_wdata    ;
wire         M_AXI_0_wready   ;
wire [3:0]   M_AXI_0_wstrb    ;
wire         M_AXI_0_wvalid   ;

wire         M_AXI_1_aclk     ;
wire         M_AXI_1_aresetn  ;
wire [31:0]  M_AXI_1_araddr   ;
wire [2:0]   M_AXI_1_arprot   ;
wire         M_AXI_1_arready  ;
wire         M_AXI_1_arvalid  ;
wire [31:0]  M_AXI_1_awaddr   ;
wire [2:0]   M_AXI_1_awprot   ;
wire         M_AXI_1_awready  ;
wire         M_AXI_1_awvalid  ;
wire         M_AXI_1_bready   ;
wire [1:0]   M_AXI_1_bresp    ;
wire         M_AXI_1_bvalid   ;
wire [31:0]  M_AXI_1_rdata    ;
wire         M_AXI_1_rready   ;
wire [1:0]   M_AXI_1_rresp    ;
wire         M_AXI_1_rvalid   ;
wire [31:0]  M_AXI_1_wdata    ;
wire         M_AXI_1_wready   ;
wire [3:0]   M_AXI_1_wstrb    ;
wire         M_AXI_1_wvalid   ;

wire         M_AXI_2_aclk     ;
wire         M_AXI_2_aresetn  ;
wire [31:0]  M_AXI_2_araddr   ;
wire [2:0]   M_AXI_2_arprot   ;
wire         M_AXI_2_arready  ;
wire         M_AXI_2_arvalid  ;
wire [31:0]  M_AXI_2_awaddr   ;
wire [2:0]   M_AXI_2_awprot   ;
wire         M_AXI_2_awready  ;
wire         M_AXI_2_awvalid  ;
wire         M_AXI_2_bready   ;
wire [1:0]   M_AXI_2_bresp    ;
wire         M_AXI_2_bvalid   ;
wire [31:0]  M_AXI_2_rdata    ;
wire         M_AXI_2_rready   ;
wire [1:0]   M_AXI_2_rresp    ;
wire         M_AXI_2_rvalid   ;
wire [31:0]  M_AXI_2_wdata    ;
wire         M_AXI_2_wready   ;
wire [3:0]   M_AXI_2_wstrb    ;
wire         M_AXI_2_wvalid   ;

design_1 design_1 (
    .aclk_0           ( sys_if_clk          ),
    .aresetn_0        ( sys_if_rstn         ),

    .S_AXI_0_araddr   ( jtag_m_axi_araddr   ),
    .S_AXI_0_arprot   ( jtag_m_axi_arprot   ),
    .S_AXI_0_arready  ( jtag_m_axi_arready  ),
    .S_AXI_0_arvalid  ( jtag_m_axi_arvalid  ),
    .S_AXI_0_awaddr   ( jtag_m_axi_awaddr   ),
    .S_AXI_0_awprot   ( jtag_m_axi_awprot   ),
    .S_AXI_0_awready  ( jtag_m_axi_awready  ),
    .S_AXI_0_awvalid  ( jtag_m_axi_awvalid  ),
    .S_AXI_0_bready   ( jtag_m_axi_bready   ),
    .S_AXI_0_bresp    ( jtag_m_axi_bresp    ),
    .S_AXI_0_bvalid   ( jtag_m_axi_bvalid   ),
    .S_AXI_0_rdata    ( jtag_m_axi_rdata    ),
    .S_AXI_0_rready   ( jtag_m_axi_rready   ),
    .S_AXI_0_rresp    ( jtag_m_axi_rresp    ),
    .S_AXI_0_rvalid   ( jtag_m_axi_rvalid   ),
    .S_AXI_0_wdata    ( jtag_m_axi_wdata    ),
    .S_AXI_0_wready   ( jtag_m_axi_wready   ),
    .S_AXI_0_wstrb    ( jtag_m_axi_wstrb    ),
    .S_AXI_0_wvalid   ( jtag_m_axi_wvalid   ),

    // System Peripheral Interface
    .M_AXI_0_aclk     ( sys_if_clk          ),
    .M_AXI_0_aresetn  ( sys_if_rstn         ),
    .M_AXI_0_araddr   ( M_AXI_0_araddr      ),
    .M_AXI_0_arprot   ( M_AXI_0_arprot      ),
    .M_AXI_0_arready  ( M_AXI_0_arready     ),
    .M_AXI_0_arvalid  ( M_AXI_0_arvalid     ),
    .M_AXI_0_awaddr   ( M_AXI_0_awaddr      ),
    .M_AXI_0_awprot   ( M_AXI_0_awprot      ),
    .M_AXI_0_awready  ( M_AXI_0_awready     ),
    .M_AXI_0_awvalid  ( M_AXI_0_awvalid     ),
    .M_AXI_0_bready   ( M_AXI_0_bready      ),
    .M_AXI_0_bresp    ( M_AXI_0_bresp       ),
    .M_AXI_0_bvalid   ( M_AXI_0_bvalid      ),
    .M_AXI_0_rdata    ( M_AXI_0_rdata       ),
    .M_AXI_0_rready   ( M_AXI_0_rready      ),
    .M_AXI_0_rresp    ( M_AXI_0_rresp       ),
    .M_AXI_0_rvalid   ( M_AXI_0_rvalid      ),
    .M_AXI_0_wdata    ( M_AXI_0_wdata       ),
    .M_AXI_0_wready   ( M_AXI_0_wready      ),
    .M_AXI_0_wstrb    ( M_AXI_0_wstrb       ),
    .M_AXI_0_wvalid   ( M_AXI_0_wvalid      ),

    // GTF Interface
    .M_AXI_1_aclk     ( M_AXI_1_aclk        ),
    .M_AXI_1_aresetn  ( M_AXI_1_aresetn     ),
    .M_AXI_1_araddr   ( M_AXI_1_araddr      ),
    .M_AXI_1_arprot   ( M_AXI_1_arprot      ),
    .M_AXI_1_arready  ( M_AXI_1_arready     ),
    .M_AXI_1_arvalid  ( M_AXI_1_arvalid     ),
    .M_AXI_1_awaddr   ( M_AXI_1_awaddr      ),
    .M_AXI_1_awprot   ( M_AXI_1_awprot      ),
    .M_AXI_1_awready  ( M_AXI_1_awready     ),
    .M_AXI_1_awvalid  ( M_AXI_1_awvalid     ),
    .M_AXI_1_bready   ( M_AXI_1_bready      ),
    .M_AXI_1_bresp    ( M_AXI_1_bresp       ),
    .M_AXI_1_bvalid   ( M_AXI_1_bvalid      ),
    .M_AXI_1_rdata    ( M_AXI_1_rdata       ),
    .M_AXI_1_rready   ( M_AXI_1_rready      ),
    .M_AXI_1_rresp    ( M_AXI_1_rresp       ),
    .M_AXI_1_rvalid   ( M_AXI_1_rvalid      ),
    .M_AXI_1_wdata    ( M_AXI_1_wdata       ),
    .M_AXI_1_wready   ( M_AXI_1_wready      ),
    .M_AXI_1_wstrb    ( M_AXI_1_wstrb       ),
    .M_AXI_1_wvalid   ( M_AXI_1_wvalid      ),

    // GTF Interface
    .M_AXI_2_aclk     ( M_AXI_2_aclk        ),
    .M_AXI_2_aresetn  ( M_AXI_2_aresetn     ),
    .M_AXI_2_araddr   ( M_AXI_2_araddr      ),
    .M_AXI_2_arprot   ( M_AXI_2_arprot      ),
    .M_AXI_2_arready  ( M_AXI_2_arready     ),
    .M_AXI_2_arvalid  ( M_AXI_2_arvalid     ),
    .M_AXI_2_awaddr   ( M_AXI_2_awaddr      ),
    .M_AXI_2_awprot   ( M_AXI_2_awprot      ),
    .M_AXI_2_awready  ( M_AXI_2_awready     ),
    .M_AXI_2_awvalid  ( M_AXI_2_awvalid     ),
    .M_AXI_2_bready   ( M_AXI_2_bready      ),
    .M_AXI_2_bresp    ( M_AXI_2_bresp       ),
    .M_AXI_2_bvalid   ( M_AXI_2_bvalid      ),
    .M_AXI_2_rdata    ( M_AXI_2_rdata       ),
    .M_AXI_2_rready   ( M_AXI_2_rready      ),
    .M_AXI_2_rresp    ( M_AXI_2_rresp       ),
    .M_AXI_2_rvalid   ( M_AXI_2_rvalid      ),
    .M_AXI_2_wdata    ( M_AXI_2_wdata       ),
    .M_AXI_2_wready   ( M_AXI_2_wready      ),
    .M_AXI_2_wstrb    ( M_AXI_2_wstrb       ),
    .M_AXI_2_wvalid   ( M_AXI_2_wvalid      )
);


// ---------------------------------------------------------------
//
//  System PIF
//
// ---------------------------------------------------------------

wire        sys_if_wen   ;
wire [31:0] sys_if_addr  ;
wire [31:0] sys_if_wdata ;
wire [31:0] sys_if_rdata ;

reg_axi_slave reg_axi_slave(
    .s_axi_aclk     ( sys_if_clk        ),
    .s_axi_aresetn  ( sys_if_rstn       ),

    .s_axi_awaddr   ( M_AXI_0_awaddr    ),
    .s_axi_awvalid  ( M_AXI_0_awvalid   ),
    .s_axi_awready  ( M_AXI_0_awready   ),
    .s_axi_wdata    ( M_AXI_0_wdata     ),
    .s_axi_wstrb    ( M_AXI_0_wstrb     ),
    .s_axi_wvalid   ( M_AXI_0_wvalid    ),
    .s_axi_wready   ( M_AXI_0_wready    ),
    .s_axi_bresp    ( M_AXI_0_bresp     ),
    .s_axi_bvalid   ( M_AXI_0_bvalid    ),
    .s_axi_bready   ( M_AXI_0_bready    ),

    .s_axi_araddr   ( M_AXI_0_araddr    ),
    .s_axi_arvalid  ( M_AXI_0_arvalid   ),
    .s_axi_arready  ( M_AXI_0_arready   ),
    .s_axi_rdata    ( M_AXI_0_rdata     ),
    .s_axi_rresp    ( M_AXI_0_rresp     ),
    .s_axi_rvalid   ( M_AXI_0_rvalid    ),
    .s_axi_rready   ( M_AXI_0_rready    ),

    .wr_en          ( sys_if_wen        ),
    .addr           ( sys_if_addr       ),
    .wdata          ( sys_if_wdata      ),
    .wstrb          (                   ),
    .rdata          ( sys_if_rdata      )
);


// ---------------------------------------------------------------
//
//  Mux System Write Enable and Read Data
//
// ---------------------------------------------------------------

wire [31:0] sys_if_addr_0  = {16'h0000, sys_if_addr[15:0]};

wire        sys_if_wen_0   ;
wire [31:0] sys_if_rdata_0 ;

wire        sys_if_wen_1   ;
wire [31:0] sys_if_rdata_1 ;

wire        sys_if_wen_2   ;
wire [31:0] sys_if_rdata_2 ;

wire        sys_if_wen_3   ;
wire [31:0] sys_if_rdata_3 ;

wire        sys_if_wen_4   ;
wire [31:0] sys_if_rdata_4 ;

wire        sys_if_wen_5   ;
wire [31:0] sys_if_rdata_5 ;

wire        sys_if_wen_6   ;
wire [31:0] sys_if_rdata_6 = 'h0;

wire        sys_if_wen_7   ;
wire [31:0] sys_if_rdata_7 = 'h0;


sys_if_switch sys_if_switch (
    // System Interface
    .sys_if_wen     ( sys_if_wen     ),
    .sys_if_addr    ( sys_if_addr    ),
    .sys_if_rdata   ( sys_if_rdata   ),

    .sys_if_wen_0   ( sys_if_wen_0   ),
    .sys_if_rdata_0 ( sys_if_rdata_0 ),

    .sys_if_wen_1   ( sys_if_wen_1   ),
    .sys_if_rdata_1 ( sys_if_rdata_1 ),

    .sys_if_wen_2   ( sys_if_wen_2   ),
    .sys_if_rdata_2 ( sys_if_rdata_2 ),

    .sys_if_wen_3   ( sys_if_wen_3   ),
    .sys_if_rdata_3 ( sys_if_rdata_3 ),

    .sys_if_wen_4   ( sys_if_wen_4   ),
    .sys_if_rdata_4 ( sys_if_rdata_4 ),

    .sys_if_wen_5   ( sys_if_wen_5   ),
    .sys_if_rdata_5 ( sys_if_rdata_5 ),

    .sys_if_wen_6   ( sys_if_wen_6   ),
    .sys_if_rdata_6 ( sys_if_rdata_6 ),

    .sys_if_wen_7   ( sys_if_wen_7   ),
    .sys_if_rdata_7 ( sys_if_rdata_7 )
);


// ---------------------------------------------------------------
//
//  System Registers
//
// ---------------------------------------------------------------

// ======================================================================
//                          0123456789abcdef
localparam HEADER_STRING = "Clk Recov 1.1   ";

wire [31:0] IO_HEADER0_VALUE  = HEADER_STRING[8*16-1:8*12];
wire [31:0] IO_HEADER1_VALUE  = HEADER_STRING[8*12-1:8*8];
wire [31:0] IO_HEADER2_VALUE  = HEADER_STRING[8*8-1:8*4];
wire [31:0] IO_HEADER3_VALUE  = HEADER_STRING[8*4-1:8*0];

wire [31:0] IO_SCRATCH_VALUE;
system_regs #(
    .NUM_CHANNEL ( NUM_CHANNEL )
) system_regs (
    .sys_if_clk    ( sys_if_clk     ),
    .sys_if_rstn   ( sys_if_rstn    ),
    .sys_if_wen    ( sys_if_wen_0   ),
    .sys_if_addr   ( sys_if_addr_0  ),
    .sys_if_wdata  ( sys_if_wdata   ),
    .sys_if_rdata  ( sys_if_rdata_0 ),

    .IO_HEADER0_VALUE ( IO_HEADER0_VALUE ),
    .IO_HEADER1_VALUE ( IO_HEADER1_VALUE ),
    .IO_HEADER2_VALUE ( IO_HEADER2_VALUE ),
    .IO_HEADER3_VALUE ( IO_HEADER3_VALUE ),
    .IO_SCRATCH_VALUE ( IO_SCRATCH_VALUE )
);


// ---------------------------------------------------------------
//
//  GTF SYNCE_CLK Buffering
//
// ---------------------------------------------------------------
// synce_clk_130 and synce_clk_127 use IBUFDS in their respective GTF
// blocks; don't buffer them here

wire [2:0] SYNCE_CLK_OUT ;

system_gtf_clk_buffer #(
    .CLK_BUS_WIDTH ( 3 )
) system_gtf_clk_buffer (
    .SYNCE_CLK_LVDS_P ( { synce_clk_131_lvds_p,
                          synce_clk_129_lvds_p,
                          synce_clk_128_lvds_p } ),
    .SYNCE_CLK_LVDS_N ( { synce_clk_131_lvds_n,
                          synce_clk_129_lvds_n,
                          synce_clk_128_lvds_n } ),
    .SYNCE_CLK_OUT    ( SYNCE_CLK_OUT          )
);

wire SYNCE_CLK_128_OUT  = SYNCE_CLK_OUT[0];
wire SYNCE_CLK_129_OUT  = SYNCE_CLK_OUT[1];
wire SYNCE_CLK_131_OUT  = SYNCE_CLK_OUT[2];
wire SYNCE_CLK_65_1_OUT = 1'b0;
wire SYNCE_CLK_65_2_OUT = 1'b0;
wire SYNCE_CLK_127_OUT ;
wire SYNCE_CLK_130_OUT ;


// ---------------------------------------------------------------
//
//  Frequency Counter and Clock Source
//
// ---------------------------------------------------------------

freq_counter_top freq_counter_top (
    .sys_if_clk    ( sys_if_clk         ),
    .sys_if_rstn   ( sys_if_rstn        ),
    .sys_if_wen    ( sys_if_wen_1       ),
    .sys_if_addr   ( sys_if_addr_0      ),
    .sys_if_wdata  ( sys_if_wdata       ),
    .sys_if_rdata  ( sys_if_rdata_1     ),
    // GTF MGTREFCLK0 fabric copies
    .clk_samp_0    ( SYNCE_CLK_127_OUT  ),
    .clk_samp_1    ( SYNCE_CLK_128_OUT  ),
    .clk_samp_2    ( SYNCE_CLK_129_OUT  ),
    .clk_samp_3    ( SYNCE_CLK_130_OUT  ),
    .clk_samp_4    ( SYNCE_CLK_131_OUT  ),
    // Bank 65 fabric SYNCE inputs
    .clk_samp_5    ( SYNCE_CLK_65_1_OUT ),
    .clk_samp_6    ( SYNCE_CLK_65_2_OUT ),
    .clk_samp_7    ( 1'b0               )
);


// ---------------------------------------------------------------
//
//  Renesas GPIO and Reset
//
// ---------------------------------------------------------------

renesas_gpio renesas_gpio (
    .sys_if_clk    ( sys_if_clk     ),
    .sys_if_rstn   ( sys_if_rstn    ),
    .sys_if_wen    ( sys_if_wen_2   ),
    .sys_if_addr   ( sys_if_addr_0  ),
    .sys_if_wdata  ( sys_if_wdata   ),
    .sys_if_rdata  ( sys_if_rdata_2 ),

    .JITT1_RESETn ( jitt_resetn     ),

    .JITT1_GPIO5  ( jitt1_gpoi5     ),
    .JITT1_GPIO4  ( jitt1_gpoi4     ),
    .JITT1_GPIO3  ( jitt1_gpoi3     ),
    .JITT1_GPIO2  ( jitt1_gpoi2     ),
    .JITT1_GPIO1  ( jitt1_gpoi1     ),
    .JITT1_GPIO0  ( jitt1_gpoi0     )
);


// ---------------------------------------------------------------
//
//  Renesas BRAM
//
// ---------------------------------------------------------------
wire        i2c_clkb   ;
wire        i2c_web    ;
wire [15:0] i2c_addrb  ;
wire [15:0] i2c_dinb   ;
wire [15:0] i2c_doutb  ;


renesas_bram renesas_bram (
    .sys_if_clk    ( sys_if_clk     ),
    .sys_if_rstn   ( sys_if_rstn    ),
    .sys_if_wen    ( sys_if_wen_3   ),
    .sys_if_addr   ( sys_if_addr_0  ),
    .sys_if_wdata  ( sys_if_wdata   ),
    .sys_if_rdata  ( sys_if_rdata_3 ),

    .i2c_clkb      ( i2c_clkb       ),
    .i2c_web       ( i2c_web        ),
    .i2c_addrb     ( i2c_addrb      ),
    .i2c_dinb      ( i2c_dinb       ),
    .i2c_doutb     ( i2c_doutb      )
);


// ---------------------------------------------------------------
//
//  Renesas I2C
//
// ---------------------------------------------------------------

renesas_i2c_top renesas_i2c_top (
    .sys_if_clk    ( sys_if_clk     ),
    .sys_if_rstn   ( sys_if_rstn    ),
    .sys_if_wen    ( sys_if_wen_4   ),
    .sys_if_addr   ( sys_if_addr_0  ),
    .sys_if_wdata  ( sys_if_wdata   ),
    .sys_if_rdata  ( sys_if_rdata_4 ),

    .i2c_clkb      ( i2c_clkb       ),
    .i2c_web       ( i2c_web        ),
    .i2c_addrb     ( i2c_addrb      ),
    .i2c_dinb      ( i2c_dinb       ),
    .i2c_doutb     ( i2c_doutb      ),

    .sys_clk_50    ( sys_clk_50     ),
    .sys_rst_50    ( sys_rst_50     ),

    .CLKGEN_SDA    ( clkgen_sda_r   ),
    .CLKGEN_SCL    ( clkgen_scl_r   )
);


// ---------------------------------------------------------------
//
//  QSFP I2C Controller
//
// ---------------------------------------------------------------

qsfp_i2c_top qsfp_i2c_top (
    .sys_if_clk         ( sys_if_clk           ),
    .sys_if_rstn        ( sys_if_rstn          ),
    .sys_if_wen         ( sys_if_wen_5         ),
    .sys_if_addr        ( sys_if_addr_0        ),
    .sys_if_wdata       ( sys_if_wdata         ),
    .sys_if_rdata       ( sys_if_rdata_5       ),

    .FPGA_MUX_RSTN      ( fpga_mux_rstn        ),
    .QSFPDD1_IO_RESET_B ( qsfpdd1_io_reset_b   ),
    .QSFPDD2_IO_RESET_B ( qsfpdd2_io_reset_b   ),

    .FPGA_SDA_R         ( fpga_sda_r           ),
    .FPGA_SCL_R         ( fpga_scl_r           )
);


// -----------------------------------------------------------
//
//  GTF Port 0 (MAC) -- QSFPDD2 / Bank 130, recovered clock to pin
//
// -----------------------------------------------------------

wire SYNCE_CLK_130_INT;
wire RECOV_CLK10_INT;

gtf_top_0 #(
    .NUM_CHANNEL  ( NUM_CHANNEL  )
) gtf_top_0 (
    .gtf_ch_gtftxn      ( qsfpdd2_tx_n         ),
    .gtf_ch_gtftxp      ( qsfpdd2_tx_p         ),
    .gtf_ch_gtfrxn      ( qsfpdd2_rx_n         ),
    .gtf_ch_gtfrxp      ( qsfpdd2_rx_p         ),

    .SYNCE_CLK_LVDS_P   ( synce_clk_130_lvds_p ),
    .SYNCE_CLK_LVDS_N   ( synce_clk_130_lvds_n ),
    .SYNCE_CLK_OUT      ( SYNCE_CLK_130_INT    ),

    .gtf_freerun_clk    ( gtf_freerun_clk      ),
    .gtf_sys_clk_out    ( gtf_sys_clk_out      ),
    .gtf_clk_wiz_locked ( gtf_clk_wiz_locked   ),

    .sys_if_clk         ( sys_if_clk           ),
    .sys_if_rstn        ( sys_if_rstn          ),

    .sys_gtf_resetn     ( IO_SCRATCH_VALUE[0]  ),

    .s_axil_aclk        ( M_AXI_1_aclk         ),
    .s_axil_aresetn     ( M_AXI_1_aresetn      ),
    .s_axil_awaddr      ( M_AXI_1_awaddr & 32'h000F_FFFF ),
    .s_axil_awprot      ( M_AXI_1_awprot       ),
    .s_axil_awvalid     ( M_AXI_1_awvalid      ),
    .s_axil_awready     ( M_AXI_1_awready      ),
    .s_axil_wdata       ( M_AXI_1_wdata        ),
    .s_axil_wstrb       ( M_AXI_1_wstrb        ),
    .s_axil_wvalid      ( M_AXI_1_wvalid       ),
    .s_axil_wready      ( M_AXI_1_wready       ),
    .s_axil_bresp       ( M_AXI_1_bresp        ),
    .s_axil_bvalid      ( M_AXI_1_bvalid       ),
    .s_axil_bready      ( M_AXI_1_bready       ),
    .s_axil_araddr      ( M_AXI_1_araddr & 32'h000F_FFFF ),
    .s_axil_arprot      ( M_AXI_1_arprot       ),
    .s_axil_arvalid     ( M_AXI_1_arvalid      ),
    .s_axil_arready     ( M_AXI_1_arready      ),
    .s_axil_rdata       ( M_AXI_1_rdata        ),
    .s_axil_rresp       ( M_AXI_1_rresp        ),
    .s_axil_rvalid      ( M_AXI_1_rvalid       ),
    .s_axil_rready      ( M_AXI_1_rready       ),

    .RECOV_CLK10_INT    ( RECOV_CLK10_INT      ),
    .RECOV_CLK10_LVDS_P ( recov_clk_65_lvds_p  ),
    .RECOV_CLK10_LVDS_N ( recov_clk_65_lvds_n  ),

    .fifo_rst           ( IO_SCRATCH_VALUE[1]  ),

    .ctl_hwchk_frm_gen_en_in ( IO_SCRATCH_VALUE[8]  ),
    .ctl_hwchk_mon_en_in     ( IO_SCRATCH_VALUE[9]  )
);

// BUFG to move SYNCE clock onto fabric for frequency measurement
BUFG_GT BUFG_GT_INST_130 (
    .CE         ( 1'b1               ),
    .CEMASK     ( 1'b0               ),
    .CLR        ( 1'b0               ),
    .CLRMASK    ( 1'b0               ),
    .DIV        ( 3'b0               ),
    .I          ( SYNCE_CLK_130_INT  ),
    .O          ( SYNCE_CLK_130_OUT  )
);


// -----------------------------------------------------------
//
//  GTF Port 1 (RAW) -- QSFPDD1 / Bank 127, loopback FIFO
//
// -----------------------------------------------------------

wire SYNCE_CLK_127_INT;

gtf_top_1 #(
    .NUM_CHANNEL  ( NUM_CHANNEL  )
) gtf_top_1 (
    .gtf_ch_gtftxn      ( qsfpdd1_tx_n         ),
    .gtf_ch_gtftxp      ( qsfpdd1_tx_p         ),
    .gtf_ch_gtfrxn      ( qsfpdd1_rx_n         ),
    .gtf_ch_gtfrxp      ( qsfpdd1_rx_p         ),

    .SYNCE_CLK_LVDS_P   ( synce_clk_127_lvds_p ),
    .SYNCE_CLK_LVDS_N   ( synce_clk_127_lvds_n ),
    .SYNCE_CLK_OUT      ( SYNCE_CLK_127_INT    ),

    .gtf_freerun_clk    ( gtf_freerun_clk      ),
    .gtf_sys_clk_out    ( gtf_sys_clk_out      ),
    .gtf_clk_wiz_locked ( gtf_clk_wiz_locked   ),

    .sys_if_clk         ( sys_if_clk           ),
    .sys_if_rstn        ( sys_if_rstn          ),

    .sys_gtf_resetn     ( IO_SCRATCH_VALUE[0]  ),

    .s_axil_aclk        ( M_AXI_2_aclk         ),
    .s_axil_aresetn     ( M_AXI_2_aresetn      ),
    .s_axil_awaddr      ( M_AXI_2_awaddr & 32'h000F_FFFF ),
    .s_axil_awprot      ( M_AXI_2_awprot       ),
    .s_axil_awvalid     ( M_AXI_2_awvalid      ),
    .s_axil_awready     ( M_AXI_2_awready      ),
    .s_axil_wdata       ( M_AXI_2_wdata        ),
    .s_axil_wstrb       ( M_AXI_2_wstrb        ),
    .s_axil_wvalid      ( M_AXI_2_wvalid       ),
    .s_axil_wready      ( M_AXI_2_wready       ),
    .s_axil_bresp       ( M_AXI_2_bresp        ),
    .s_axil_bvalid      ( M_AXI_2_bvalid       ),
    .s_axil_bready      ( M_AXI_2_bready       ),
    .s_axil_araddr      ( M_AXI_2_araddr & 32'h000F_FFFF ),
    .s_axil_arprot      ( M_AXI_2_arprot       ),
    .s_axil_arvalid     ( M_AXI_2_arvalid      ),
    .s_axil_arready     ( M_AXI_2_arready      ),
    .s_axil_rdata       ( M_AXI_2_rdata        ),
    .s_axil_rresp       ( M_AXI_2_rresp        ),
    .s_axil_rvalid      ( M_AXI_2_rvalid       ),
    .s_axil_rready      ( M_AXI_2_rready       )
);

// BUFG to move SYNCE clock onto fabric for frequency measurement
BUFG_GT BUFG_GT_INST_127 (
    .CE         ( 1'b1               ),
    .CEMASK     ( 1'b0               ),
    .CLR        ( 1'b0               ),
    .CLRMASK    ( 1'b0               ),
    .DIV        ( 3'b0               ),
    .I          ( SYNCE_CLK_127_INT  ),
    .O          ( SYNCE_CLK_127_OUT  )
);

endmodule
