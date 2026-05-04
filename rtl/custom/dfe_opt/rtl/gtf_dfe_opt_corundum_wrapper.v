/*

GTF DFE Optimization — Corundum Application Block Wrapper
Maps the standalone gtf_dfe_opt_core into Corundum's mqnic_app_block interface.

This module is instantiated inside mqnic_app_block.v and connects:
  - Corundum's s_axil_app_ctrl_* → core's AXI-Lite slave
  - GTF sideband signals passed through fpga.v → fpga_core.v → mqnic_core → app_block
  - TX alignment engine to the direct TX AXI-Stream interface

Integration steps:
  1. fpga.v: expose GTF_CHANNEL DRP + sideband ports to fpga_core
  2. fpga_core.v: pass sideband through to mqnic_core
  3. mqnic_core: pass through to app_block
  4. mqnic_app_block.v: instantiate this wrapper

*/

`resetall
`timescale 1ns / 1ps
`default_nettype none

module gtf_dfe_opt_corundum_wrapper #(
    parameter AXIL_APP_CTRL_ADDR_WIDTH = 16,
    parameter AXIL_APP_CTRL_DATA_WIDTH = 32,
    parameter AXIL_APP_CTRL_STRB_WIDTH = (AXIL_APP_CTRL_DATA_WIDTH/8),
    parameter NUM_TAPS = 15,
    parameter VERSION = 32'h47464401
)
(
    input  wire        clk,
    input  wire        rst,

    /*
     * Corundum AXI-Lite application control interface
     */
    input  wire [AXIL_APP_CTRL_ADDR_WIDTH-1:0]  s_axil_app_ctrl_awaddr,
    input  wire [2:0]                            s_axil_app_ctrl_awprot,
    input  wire                                  s_axil_app_ctrl_awvalid,
    output wire                                  s_axil_app_ctrl_awready,
    input  wire [AXIL_APP_CTRL_DATA_WIDTH-1:0]   s_axil_app_ctrl_wdata,
    input  wire [AXIL_APP_CTRL_STRB_WIDTH-1:0]   s_axil_app_ctrl_wstrb,
    input  wire                                  s_axil_app_ctrl_wvalid,
    output wire                                  s_axil_app_ctrl_wready,
    output wire [1:0]                            s_axil_app_ctrl_bresp,
    output wire                                  s_axil_app_ctrl_bvalid,
    input  wire                                  s_axil_app_ctrl_bready,
    input  wire [AXIL_APP_CTRL_ADDR_WIDTH-1:0]   s_axil_app_ctrl_araddr,
    input  wire [2:0]                            s_axil_app_ctrl_arprot,
    input  wire                                  s_axil_app_ctrl_arvalid,
    output wire                                  s_axil_app_ctrl_arready,
    output wire [AXIL_APP_CTRL_DATA_WIDTH-1:0]   s_axil_app_ctrl_rdata,
    output wire [1:0]                            s_axil_app_ctrl_rresp,
    output wire                                  s_axil_app_ctrl_rvalid,
    input  wire                                  s_axil_app_ctrl_rready,

    /*
     * GTF DRP
     */
    output wire [15:0] gtf_drp_addr,
    output wire [15:0] gtf_drp_di,
    output wire        gtf_drp_en,
    output wire        gtf_drp_we,
    input  wire [15:0] gtf_drp_do,
    input  wire        gtf_drp_rdy,

    /*
     * GTF sideband — DFE/LPM
     */
    output wire        gtf_rxlpmen,
    output wire [NUM_TAPS*2-1:0] gtf_tap_hold_ovrden,
    output wire        gtf_rxdfelpmreset,
    input  wire [15:0] gtf_dmonitorout,

    /*
     * GTF sideband — Eye scan
     */
    output wire        gtf_eyescanreset,
    input  wire        gtf_eyescandataerror,

    /*
     * GTF sideband — Buffer bypass
     */
    output wire        gtf_txphdlyreset,
    output wire        gtf_txphdlytstclk,
    output wire        gtf_txphalign,
    output wire        gtf_txphalignen,
    output wire        gtf_txphdlypd,
    input  wire        gtf_txphaligndone,
    input  wire        gtf_txsyncdone,
    output wire        gtf_rxphdlyreset,
    output wire        gtf_rxphalign,
    output wire        gtf_rxphalignen,
    output wire        gtf_rxphdlypd,
    input  wire        gtf_rxphaligndone,
    input  wire        gtf_rxsyncdone,
    input  wire        gtf_txresetdone,
    input  wire        gtf_rxresetdone,
    input  wire        gtf_rxcdrlock,

    /*
     * GTF sideband — FCS control
     */
    output wire        gtf_ctl_rx_check_fcs,
    output wire        gtf_ctl_rx_delete_fcs,
    output wire        gtf_ctl_tx_fcs_ins_enable,

    /*
     * GTF hard MAC — TX alignment
     */
    input  wire        gtf_txaxistcanstart,
    input  wire        gtf_txaxistready,
    input  wire        gtf_txgbseqstart,

    /*
     * Corundum direct TX interface passthrough
     */
    input  wire [15:0] s_axis_direct_tx_tdata,
    input  wire        s_axis_direct_tx_tvalid,
    input  wire [1:0]  s_axis_direct_tx_tlast,
    input  wire        s_axis_direct_tx_tsof,
    input  wire        s_axis_direct_tx_terr,
    input  wire [7:0]  s_axis_direct_tx_tpre,
    output wire        s_axis_direct_tx_tready,
    output wire [15:0] m_axis_direct_tx_tdata,
    output wire        m_axis_direct_tx_tvalid,
    output wire [1:0]  m_axis_direct_tx_tlast,
    output wire        m_axis_direct_tx_tsof,
    output wire        m_axis_direct_tx_terr,
    output wire [7:0]  m_axis_direct_tx_tpre,
    input  wire        m_axis_direct_tx_tready,

    /*
     * Interrupt output
     */
    output wire        irq
);

gtf_dfe_opt_core #(
    .AXIL_ADDR_WIDTH(AXIL_APP_CTRL_ADDR_WIDTH),
    .AXIL_DATA_WIDTH(AXIL_APP_CTRL_DATA_WIDTH),
    .NUM_TAPS(NUM_TAPS),
    .VERSION(VERSION)
)
core_inst (
    .clk(clk),
    .rst(rst),

    // AXI-Lite
    .s_axil_awaddr(s_axil_app_ctrl_awaddr),
    .s_axil_awprot(s_axil_app_ctrl_awprot),
    .s_axil_awvalid(s_axil_app_ctrl_awvalid),
    .s_axil_awready(s_axil_app_ctrl_awready),
    .s_axil_wdata(s_axil_app_ctrl_wdata),
    .s_axil_wstrb(s_axil_app_ctrl_wstrb),
    .s_axil_wvalid(s_axil_app_ctrl_wvalid),
    .s_axil_wready(s_axil_app_ctrl_wready),
    .s_axil_bresp(s_axil_app_ctrl_bresp),
    .s_axil_bvalid(s_axil_app_ctrl_bvalid),
    .s_axil_bready(s_axil_app_ctrl_bready),
    .s_axil_araddr(s_axil_app_ctrl_araddr),
    .s_axil_arprot(s_axil_app_ctrl_arprot),
    .s_axil_arvalid(s_axil_app_ctrl_arvalid),
    .s_axil_arready(s_axil_app_ctrl_arready),
    .s_axil_rdata(s_axil_app_ctrl_rdata),
    .s_axil_rresp(s_axil_app_ctrl_rresp),
    .s_axil_rvalid(s_axil_app_ctrl_rvalid),
    .s_axil_rready(s_axil_app_ctrl_rready),

    // DRP
    .gtf_drp_addr(gtf_drp_addr),
    .gtf_drp_di(gtf_drp_di),
    .gtf_drp_en(gtf_drp_en),
    .gtf_drp_we(gtf_drp_we),
    .gtf_drp_do(gtf_drp_do),
    .gtf_drp_rdy(gtf_drp_rdy),

    // DFE/LPM
    .gtf_rxlpmen(gtf_rxlpmen),
    .gtf_tap_hold_ovrden(gtf_tap_hold_ovrden),
    .gtf_rxdfelpmreset(gtf_rxdfelpmreset),
    .gtf_dmonitorout(gtf_dmonitorout),

    // Eye scan
    .gtf_eyescanreset(gtf_eyescanreset),
    .gtf_eyescandataerror(gtf_eyescandataerror),

    // Buffer bypass
    .gtf_txphdlyreset(gtf_txphdlyreset),
    .gtf_txphdlytstclk(gtf_txphdlytstclk),
    .gtf_txphalign(gtf_txphalign),
    .gtf_txphalignen(gtf_txphalignen),
    .gtf_txphdlypd(gtf_txphdlypd),
    .gtf_txphaligndone(gtf_txphaligndone),
    .gtf_txsyncdone(gtf_txsyncdone),
    .gtf_rxphdlyreset(gtf_rxphdlyreset),
    .gtf_rxphalign(gtf_rxphalign),
    .gtf_rxphalignen(gtf_rxphalignen),
    .gtf_rxphdlypd(gtf_rxphdlypd),
    .gtf_rxphaligndone(gtf_rxphaligndone),
    .gtf_rxsyncdone(gtf_rxsyncdone),
    .gtf_txresetdone(gtf_txresetdone),
    .gtf_rxresetdone(gtf_rxresetdone),
    .gtf_rxcdrlock(gtf_rxcdrlock),

    // FCS
    .gtf_ctl_rx_check_fcs(gtf_ctl_rx_check_fcs),
    .gtf_ctl_rx_delete_fcs(gtf_ctl_rx_delete_fcs),
    .gtf_ctl_tx_fcs_ins_enable(gtf_ctl_tx_fcs_ins_enable),

    // TX alignment
    .gtf_txaxistcanstart(gtf_txaxistcanstart),
    .gtf_txaxistready(gtf_txaxistready),
    .gtf_txgbseqstart(gtf_txgbseqstart),

    // TX passthrough
    .s_axis_tx_tdata(s_axis_direct_tx_tdata),
    .s_axis_tx_tvalid(s_axis_direct_tx_tvalid),
    .s_axis_tx_tlast(s_axis_direct_tx_tlast),
    .s_axis_tx_tsof(s_axis_direct_tx_tsof),
    .s_axis_tx_terr(s_axis_direct_tx_terr),
    .s_axis_tx_tpre(s_axis_direct_tx_tpre),
    .s_axis_tx_tready(s_axis_direct_tx_tready),
    .m_axis_tx_tdata(m_axis_direct_tx_tdata),
    .m_axis_tx_tvalid(m_axis_direct_tx_tvalid),
    .m_axis_tx_tlast(m_axis_direct_tx_tlast),
    .m_axis_tx_tsof(m_axis_direct_tx_tsof),
    .m_axis_tx_terr(m_axis_direct_tx_terr),
    .m_axis_tx_tpre(m_axis_direct_tx_tpre),
    .m_axis_tx_tready(m_axis_direct_tx_tready),

    // Interrupt
    .irq(irq)
);

endmodule

`resetall
