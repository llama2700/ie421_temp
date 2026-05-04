/*
Copyright (C) 2024, Advanced Micro Devices, Inc. All rights reserved.
SPDX-License-Identifier: X11
*/


module renesas_i2c_top #(
    parameter SIMULATION = "false",
    parameter NUM_SYNCE_CLK = 8
) (
    input  wire     clk_sys_lvds_300_p,
    input  wire     clk_sys_lvds_300_n,

    inout  wire     clkgen_sda_r ,
    inout  wire     clkgen_scl_r ,

    // SYNCE reference clocks from jitter cleaner (to measure frequencies)
    input  wire [NUM_SYNCE_CLK-1:0] synce_clk_lvds_p ,
    input  wire [NUM_SYNCE_CLK-1:0] synce_clk_lvds_n
);

localparam AXI_ADDR_WIDTH_0 = 9;
localparam AXI_DATA_WIDTH_0 = 32;


wire s_axi_aclk     ;
wire s_axi_aresetn  ;

// -----------------------------------------------------------
//
//    Clock Generation....
//       CLK IN  - 300Mhz
//       CLK OUT - 50Mhz
//
// -----------------------------------------------------------
clk_wiz_0 clk_wiz_0
 (
    .clk_in1_p  ( clk_sys_lvds_300_p ),
    .clk_in1_n  ( clk_sys_lvds_300_n ),
    .clk_out1   ( s_axi_aclk       ),
    .locked     ( s_axi_aresetn    )
 );

// -----------------------------------------------------------
//
//    I2C AXI Sequencer....
//
// -----------------------------------------------------------

wire vio_rstn;
reg  vio_rstn_r;
generate
if (SIMULATION == "false") begin
    vio_0 vio_0
    (
        .clk        ( s_axi_aclk    ),
        .probe_out0 ( vio_rstn      )
    );
end else begin
    assign vio_rstn = vio_rstn_r;
end
endgenerate

// -------------------------------------------------

reg [15:0] timer_start;
always@(posedge s_axi_aclk)
begin
    if (!vio_rstn)
        timer_start <= 'h2000;
    else if (timer_start == 0)
        timer_start <= 'h0;
    else
        timer_start <= timer_start - 1;
end

wire start_pulse = (timer_start == 'h1);

// -------------------------------------------------

// AXI Sequencer Register Interface...
wire                           seq_axi_wr_req ; // pulse
wire                           seq_axi_rd_req ; // pulse
wire [AXI_ADDR_WIDTH_0-1:0]    seq_axi_addr   ; // valid on wr/rd req pulse
wire [AXI_DATA_WIDTH_0-1:0]    seq_axi_wdata  ; // valid on wr/rd req pulse
wire [AXI_DATA_WIDTH_0/8-1:0]  seq_axi_wstrb  ; // valid on wr/rd req pulse
wire                           seq_axi_ack    ; // pulse upon completion
wire [AXI_DATA_WIDTH_0-1:0]    seq_axi_rdata  ; // valid on op_ack pulse


// Seq->I2C AXI Interface...
wire [AXI_ADDR_WIDTH_0-1:0]    m_axi_araddr   ;
wire                           m_axi_arvalid  ;
wire                           m_axi_arready  ;

wire [AXI_ADDR_WIDTH_0-1:0]    m_axi_awaddr   ;
wire                           m_axi_awvalid  ;
wire                           m_axi_awready  ;

wire                           m_axi_bready   ;
wire [1:0]                     m_axi_bresp    ;
wire                           m_axi_bvalid   ;

wire                           m_axi_rready   ;
wire [AXI_DATA_WIDTH_0-1:0]    m_axi_rdata    ;
wire [1:0]                     m_axi_rresp    ;
wire                           m_axi_rvalid   ;

wire [AXI_DATA_WIDTH_0-1:0]    m_axi_wdata    ;
wire [AXI_DATA_WIDTH_0/8-1:0]  m_axi_wstrb    ;
wire                           m_axi_wvalid   ;
wire                           m_axi_wready   ;

wire  [7:0]                    xfer_count     ;
wire  [31:0]                   xfer_offset    ;
wire                           xfer_enable    ;
wire                           bulk_seq_busy  ;

// Bulk sequencer outputs (before mux)
wire                           bulk_wr_req    ;
wire                           bulk_rd_req    ;
wire [AXI_ADDR_WIDTH_0-1:0]   bulk_addr      ;
wire [AXI_DATA_WIDTH_0-1:0]   bulk_wdata     ;

// -----------------------------------------------------------
//
//    I2C Command Sequencer (bulk programming from BRAM)....
//
// -------------------------------------------------

i2c_sequencer  #(
    .AXI_ADDR_WIDTH ( AXI_ADDR_WIDTH_0 ),
    .AXI_DATA_WIDTH ( AXI_DATA_WIDTH_0 )
) i2c_sequencer (
    .aclk             ( s_axi_aclk      ),
    .aresetn          ( s_axi_aresetn   ),

    .start_pulse      ( start_pulse     ),

    .seq_axi_wr_req   ( bulk_wr_req     ),
    .seq_axi_rd_req   ( bulk_rd_req     ),
    .seq_axi_addr     ( bulk_addr       ),
    .seq_axi_wdata    ( bulk_wdata      ),
    .seq_axi_ack      ( seq_axi_ack     ),
    .seq_axi_rdata    ( seq_axi_rdata   ),

    .xfer_count       ( xfer_count      ),
    .xfer_offset      ( xfer_offset     ),
    .xfer_enable      ( xfer_enable     ),
    .seq_busy         ( bulk_seq_busy   )
);


// -----------------------------------------------------------
//
//    User I2C Path (single-transaction read/write via JTAG)
//
// -----------------------------------------------------------

wire        user_ctrl_pulse ;
wire [0:0]  user_ctrl_rw    ;
wire [7:0]  user_ctrl_id    ;
wire [7:0]  user_addr_addr  ;
wire [7:0]  user_wdata_data ;
wire [7:0]  user_rdata_data ;
wire        user_ctrl_cmplt ;
wire [3:0]  user_wcount     ;
wire [63:0] user_wdata_buf  ;

wire                           user_wr_req    ;
wire                           user_rd_req    ;
wire [AXI_ADDR_WIDTH_0-1:0]   user_addr      ;
wire [AXI_DATA_WIDTH_0-1:0]   user_wdata     ;

reg_i2c_user_logic reg_i2c_user_logic (
    .aclk              ( s_axi_aclk        ),
    .aresetn           ( s_axi_aresetn     ),
    .wen               ( fc_sys_if_wen     ),
    .addr              ( fc_sys_if_addr    ),
    .wdata             ( fc_sys_if_wdata   ),
    .rdata             ( i2c_user_rdata    ),
    .IO_CONTROL_PULSE  ( user_ctrl_pulse   ),
    .IO_CONTROL_RW     ( user_ctrl_rw      ),
    .IO_CONTROL_ID     ( user_ctrl_id      ),
    .IO_ADDR_ADDR      ( user_addr_addr    ),
    .IO_WDATA_WDATA    ( user_wdata_data   ),
    .IO_RDATA_RDATA    ( user_rdata_data   ),
    .IO_CONTROL_CMPLT  ( user_ctrl_cmplt   ),
    .IO_WCOUNT         ( user_wcount       ),
    .IO_WDATA_BUF      ( user_wdata_buf    )
);

i2c_axi_sequencer #(
    .AXI_ADDR_WIDTH ( AXI_ADDR_WIDTH_0 ),
    .AXI_DATA_WIDTH ( AXI_DATA_WIDTH_0 )
) i2c_axi_sequencer (
    .aclk             ( s_axi_aclk        ),
    .aresetn          ( s_axi_aresetn     ),

    .IO_CONTROL_PULSE ( user_ctrl_pulse   ),
    .IO_CONTROL_RW    ( user_ctrl_rw      ),
    .IO_CONTROL_ID    ( user_ctrl_id      ),
    .IO_ADDR_ADDR     ( user_addr_addr    ),
    .IO_WDATA_WDATA   ( user_wdata_data   ),
    .IO_WCOUNT        ( user_wcount       ),
    .IO_WDATA_BUF     ( user_wdata_buf    ),
    .IO_RDATA_RDATA   ( user_rdata_data   ),
    .IO_CONTROL_CMPLT ( user_ctrl_cmplt   ),

    .seq_axi_wr_req   ( user_wr_req       ),
    .seq_axi_rd_req   ( user_rd_req       ),
    .seq_axi_addr     ( user_addr         ),
    .seq_axi_wdata    ( user_wdata        ),
    .seq_axi_ack      ( seq_axi_ack       ),
    .seq_axi_rdata    ( seq_axi_rdata     )
);


// -----------------------------------------------------------
//
//    seq_axi Mux: bulk sequencer vs user path
//    When bulk is busy → bulk has control
//    When bulk is idle (ST_RST or ST_DONE) → user has control
//
// -----------------------------------------------------------

assign seq_axi_wr_req = bulk_seq_busy ? bulk_wr_req : user_wr_req ;
assign seq_axi_rd_req = bulk_seq_busy ? bulk_rd_req : user_rd_req ;
assign seq_axi_addr   = bulk_seq_busy ? bulk_addr   : user_addr   ;
assign seq_axi_wdata  = bulk_seq_busy ? bulk_wdata  : user_wdata  ;
assign seq_axi_wstrb  = {AXI_DATA_WIDTH_0/8 {1'b1} };


// -----------------------------------------------------------
//
//    AXI I/F Driver (shared)....
//
// -----------------------------------------------------------

axi_master  #(
    .AXI_ADDR_WIDTH ( AXI_ADDR_WIDTH_0 ),
    .AXI_DATA_WIDTH ( AXI_DATA_WIDTH_0 )
) axi_master_0 (
    .m_axi_aclk     ( s_axi_aclk      ),
    .m_axi_aresetn  ( s_axi_aresetn   ),

    // Simple TG Interface (muxed)...
    .wr_req         ( seq_axi_wr_req      ),
    .rd_req         ( seq_axi_rd_req      ),
    .addr           ( seq_axi_addr        ),
    .wdata          ( seq_axi_wdata       ),
    .wstrb          ( seq_axi_wstrb       ),
    .op_ack         ( seq_axi_ack         ),
    .rdata          ( seq_axi_rdata       ),

    // AXI Master Interface...
    .m_axi_araddr   ( m_axi_araddr      ),
    .m_axi_arvalid  ( m_axi_arvalid     ),
    .m_axi_arready  ( m_axi_arready     ),

    .m_axi_awaddr   ( m_axi_awaddr      ),
    .m_axi_awvalid  ( m_axi_awvalid     ),
    .m_axi_awready  ( m_axi_awready     ),

    .m_axi_bready   ( m_axi_bready      ),
    .m_axi_bresp    ( m_axi_bresp       ),
    .m_axi_bvalid   ( m_axi_bvalid      ),

    .m_axi_rready   ( m_axi_rready      ),
    .m_axi_rdata    ( m_axi_rdata       ),
    .m_axi_rresp    ( m_axi_rresp       ),
    .m_axi_rvalid   ( m_axi_rvalid      ),

    .m_axi_wdata    ( m_axi_wdata       ),
    .m_axi_wstrb    ( m_axi_wstrb       ),
    .m_axi_wvalid   ( m_axi_wvalid      ),
    .m_axi_wready   ( m_axi_wready      )
);



// -----------------------------------------------------------
//
//    I2C Controller IP....
//
// -----------------------------------------------------------

wire sda_i ;
wire sda_o ;
wire sda_t ;
wire scl_i ;
wire scl_o ;
wire scl_t ;

axi_iic_0 axi_iic_0 (
    .s_axi_aclk      ( s_axi_aclk       ),

    .s_axi_aresetn   ( s_axi_aresetn    ), // & I2C_RESETB),
    .s_axi_awaddr    ( m_axi_awaddr     ),
    .s_axi_awvalid   ( m_axi_awvalid    ),
    .s_axi_awready   ( m_axi_awready    ),
    .s_axi_wdata     ( m_axi_wdata      ),
    .s_axi_wstrb     ( m_axi_wstrb      ),
    .s_axi_wvalid    ( m_axi_wvalid     ),
    .s_axi_wready    ( m_axi_wready     ),
    .s_axi_bresp     ( m_axi_bresp      ),
    .s_axi_bvalid    ( m_axi_bvalid     ),
    .s_axi_bready    ( m_axi_bready     ),
    .s_axi_araddr    ( m_axi_araddr     ),
    .s_axi_arvalid   ( m_axi_arvalid    ),
    .s_axi_arready   ( m_axi_arready    ),
    .s_axi_rdata     ( m_axi_rdata      ),
    .s_axi_rresp     ( m_axi_rresp      ),
    .s_axi_rvalid    ( m_axi_rvalid     ),
    .s_axi_rready    ( m_axi_rready     ),

    .sda_i           ( sda_i            ),
    .sda_o           ( sda_o            ),
    .sda_t           ( sda_t            ),
    .scl_i           ( scl_i            ),
    .scl_o           ( scl_o            ),
    .scl_t           ( scl_t            ),
    .iic2intc_irpt   (                  ),
    .gpo             (                  )
  );


IOBUF IOBUF_SDA( .IO(clkgen_sda_r), .I(sda_o), .O(sda_i), .T(sda_t));
IOBUF IOBUF_SCL( .IO(clkgen_scl_r), .I(scl_o), .O(scl_i), .T(scl_t));


// -----------------------------------------------------------
//
//    ILA to View I2C Transactions...
//
// -----------------------------------------------------------

reg         ila_scl ;
reg         ila_sda ;
reg  [7:0]  ila_xfer_count ;
reg  [31:0] ila_xfer_offset;
reg         ila_xfer_enable;


always@(posedge s_axi_aclk)
begin
    ila_scl         <= scl_i          ;
    ila_sda         <= sda_i          ;
    ila_xfer_count  <= xfer_count     ;
    ila_xfer_offset <= xfer_offset    ;
    ila_xfer_enable <= xfer_enable    ;
end


ila_0 ila_0 (
    .clk     ( s_axi_aclk       ),
    .probe0  ( ila_scl          ),  // 1b
    .probe1  ( ila_sda          ),  // 1b
    .probe2  ( ila_xfer_count   ),  // 8b
    .probe3  ( ila_xfer_offset  ),  // 32b
    .probe4  ( ila_xfer_enable  )   // 1b
);


// -----------------------------------------------------------
//
//    JTAG AXI Interface (for reading freq counters)
//
// -----------------------------------------------------------

wire [31:0]  jtag_m_axi_araddr  ;
wire         jtag_m_axi_arvalid ;
wire         jtag_m_axi_arready ;
wire [31:0]  jtag_m_axi_awaddr  ;
wire         jtag_m_axi_awvalid ;
wire         jtag_m_axi_awready ;
wire         jtag_m_axi_bready  ;
wire [1:0]   jtag_m_axi_bresp   ;
wire         jtag_m_axi_bvalid  ;
wire         jtag_m_axi_rready  ;
wire [31:0]  jtag_m_axi_rdata   ;
wire [1:0]   jtag_m_axi_rresp   ;
wire         jtag_m_axi_rvalid  ;
wire [31:0]  jtag_m_axi_wdata   ;
wire [3:0]   jtag_m_axi_wstrb   ;
wire         jtag_m_axi_wvalid  ;
wire         jtag_m_axi_wready  ;

generate
if (SIMULATION == "false") begin
jtag_axi_0 jtag_axi_0 (
    .aclk           ( s_axi_aclk          ),
    .aresetn        ( s_axi_aresetn       ),
    .m_axi_awaddr   ( jtag_m_axi_awaddr   ),
    .m_axi_awprot   (                     ),
    .m_axi_awvalid  ( jtag_m_axi_awvalid  ),
    .m_axi_awready  ( jtag_m_axi_awready  ),
    .m_axi_wdata    ( jtag_m_axi_wdata    ),
    .m_axi_wstrb    ( jtag_m_axi_wstrb    ),
    .m_axi_wvalid   ( jtag_m_axi_wvalid   ),
    .m_axi_wready   ( jtag_m_axi_wready   ),
    .m_axi_bresp    ( jtag_m_axi_bresp    ),
    .m_axi_bvalid   ( jtag_m_axi_bvalid   ),
    .m_axi_bready   ( jtag_m_axi_bready   ),
    .m_axi_araddr   ( jtag_m_axi_araddr   ),
    .m_axi_arprot   (                     ),
    .m_axi_arvalid  ( jtag_m_axi_arvalid  ),
    .m_axi_arready  ( jtag_m_axi_arready  ),
    .m_axi_rdata    ( jtag_m_axi_rdata    ),
    .m_axi_rresp    ( jtag_m_axi_rresp    ),
    .m_axi_rvalid   ( jtag_m_axi_rvalid   ),
    .m_axi_rready   ( jtag_m_axi_rready   )
);
end else begin
    assign jtag_m_axi_awaddr  = 'h0;
    assign jtag_m_axi_awvalid = 'h0;
    assign jtag_m_axi_wdata   = 'h0;
    assign jtag_m_axi_wstrb   = 'h0;
    assign jtag_m_axi_wvalid  = 'h0;
    assign jtag_m_axi_bready  = 'h0;
    assign jtag_m_axi_araddr  = 'h0;
    assign jtag_m_axi_arvalid = 'h0;
    assign jtag_m_axi_rready  = 'h0;
end
endgenerate


// -----------------------------------------------------------
//
//    AXI-Lite to sys_if bridge (for freq counter registers)
//
// -----------------------------------------------------------

wire        fc_sys_if_wen   ;
wire [31:0] fc_sys_if_addr  ;
wire [31:0] fc_sys_if_wdata ;
wire [31:0] fc_sys_if_rdata ;
wire [31:0] i2c_user_rdata  ;

reg_axi_slave reg_axi_slave (
    .s_axi_aclk     ( s_axi_aclk           ),
    .s_axi_aresetn  ( s_axi_aresetn        ),

    .s_axi_awaddr   ( jtag_m_axi_awaddr    ),
    .s_axi_awvalid  ( jtag_m_axi_awvalid   ),
    .s_axi_awready  ( jtag_m_axi_awready   ),
    .s_axi_wdata    ( jtag_m_axi_wdata     ),
    .s_axi_wstrb    ( jtag_m_axi_wstrb     ),
    .s_axi_wvalid   ( jtag_m_axi_wvalid    ),
    .s_axi_wready   ( jtag_m_axi_wready    ),
    .s_axi_bresp    ( jtag_m_axi_bresp     ),
    .s_axi_bvalid   ( jtag_m_axi_bvalid    ),
    .s_axi_bready   ( jtag_m_axi_bready    ),

    .s_axi_araddr   ( jtag_m_axi_araddr    ),
    .s_axi_arvalid  ( jtag_m_axi_arvalid   ),
    .s_axi_arready  ( jtag_m_axi_arready   ),
    .s_axi_rdata    ( jtag_m_axi_rdata     ),
    .s_axi_rresp    ( jtag_m_axi_rresp     ),
    .s_axi_rvalid   ( jtag_m_axi_rvalid    ),
    .s_axi_rready   ( jtag_m_axi_rready    ),

    .wr_en          ( fc_sys_if_wen        ),
    .addr           ( fc_sys_if_addr       ),
    .wdata          ( fc_sys_if_wdata      ),
    .wstrb          (                      ),
    .rdata          ( fc_sys_if_rdata | i2c_user_rdata )
);


// -----------------------------------------------------------
//
//    SYNCE Clock Buffering
//
// -----------------------------------------------------------

wire [NUM_SYNCE_CLK-1:0] synce_clk_out;

system_gtf_clk_buffer #(
    .CLK_BUS_WIDTH ( NUM_SYNCE_CLK )
) system_gtf_clk_buffer (
    .SYNCE_CLK_LVDS_P ( synce_clk_lvds_p ),
    .SYNCE_CLK_LVDS_N ( synce_clk_lvds_n ),
    .SYNCE_CLK_OUT    ( synce_clk_out     )
);


// -----------------------------------------------------------
//
//    Frequency Counter
//
// -----------------------------------------------------------

freq_counter_top freq_counter_top (
    .sys_if_clk    ( s_axi_aclk        ),
    .sys_if_rstn   ( s_axi_aresetn     ),
    .sys_if_wen    ( fc_sys_if_wen     ),
    .sys_if_addr   ( fc_sys_if_addr    ),
    .sys_if_wdata  ( fc_sys_if_wdata   ),
    .sys_if_rdata  ( fc_sys_if_rdata   ),
    .clk_samp_0    ( synce_clk_out[0]  ),
    .clk_samp_1    ( synce_clk_out[1]  ),
    .clk_samp_2    ( synce_clk_out[2]  ),
    .clk_samp_3    ( synce_clk_out[3]  ),
    .clk_samp_4    ( synce_clk_out[4]  ),
    .clk_samp_5    ( synce_clk_out[5]  ),
    .clk_samp_6    ( synce_clk_out[6]  ),
    .clk_samp_7    ( synce_clk_out[7]  )
);


endmodule
