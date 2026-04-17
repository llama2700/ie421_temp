/*
 * Ultra-low-latency fused MAC+PCS top-level wrapper.
 *
 * Instantiates:
 *   - ll_eth_rx   (fused RX PCS+MAC with integrated frame sync)
 *   - ll_eth_tx   (fused TX MAC+PCS)
 *
 * Separate RX/TX clock domains.
 */

`resetall
`timescale 1ns / 1ps
`default_nettype none

module ll_eth_mac_pcs #(
    parameter DATA_WIDTH        = 64,
    parameter HDR_WIDTH         = 2,
    parameter SCRAMBLER_DISABLE = 0,
    parameter MIN_FRAME_LENGTH  = 64
)(
    input  wire                   rx_clk,
    input  wire                   rx_rst,
    input  wire                   tx_clk,
    input  wire                   tx_rst,

    // RX SerDes interface
    input  wire [DATA_WIDTH-1:0]  serdes_rx_data,
    input  wire [HDR_WIDTH-1:0]   serdes_rx_hdr,
    output wire                   serdes_rx_bitslip,

    // TX SerDes interface
    output wire [DATA_WIDTH-1:0]  serdes_tx_data,
    output wire [HDR_WIDTH-1:0]   serdes_tx_hdr,

    // RX AXI-Stream output
    output wire [DATA_WIDTH-1:0]  m_axis_tdata,
    output wire [7:0]             m_axis_tkeep,
    output wire                   m_axis_tvalid,
    output wire                   m_axis_tlast,
    output wire                   m_axis_tuser,

    // TX AXI-Stream input
    input  wire [DATA_WIDTH-1:0]  s_axis_tdata,
    input  wire [7:0]             s_axis_tkeep,
    input  wire                   s_axis_tvalid,
    output wire                   s_axis_tready,
    input  wire                   s_axis_tlast,
    input  wire                   s_axis_tuser,

    // Configuration
    input  wire [7:0]             cfg_ifg,

    // Status
    output wire                   rx_block_lock,
    output wire                   rx_bad_block,
    output wire                   rx_start_packet,
    output wire                   rx_error_bad_fcs,
    output wire                   tx_start_packet,
    output wire                   tx_error_underflow
);

initial begin
    if (DATA_WIDTH != 64) begin
        $display("Error: DATA_WIDTH must be 64");
        $finish;
    end
    if (HDR_WIDTH != 2) begin
        $display("Error: HDR_WIDTH must be 2");
        $finish;
    end
end

// ---------------------------------------------------------------
// RX path (rx_clk domain)
// ---------------------------------------------------------------
ll_eth_rx #(
    .DATA_WIDTH(DATA_WIDTH),
    .HDR_WIDTH(HDR_WIDTH),
    .SCRAMBLER_DISABLE(SCRAMBLER_DISABLE)
) rx_inst (
    .clk(rx_clk),
    .rst(rx_rst),

    .serdes_rx_data(serdes_rx_data),
    .serdes_rx_hdr(serdes_rx_hdr),

    .m_axis_tdata(m_axis_tdata),
    .m_axis_tkeep(m_axis_tkeep),
    .m_axis_tvalid(m_axis_tvalid),
    .m_axis_tlast(m_axis_tlast),
    .m_axis_tuser(m_axis_tuser),

    .rx_block_lock(rx_block_lock),
    .serdes_rx_bitslip(serdes_rx_bitslip),

    .rx_bad_block(rx_bad_block),
    .rx_start_packet(rx_start_packet),
    .rx_error_bad_fcs(rx_error_bad_fcs)
);

// ---------------------------------------------------------------
// TX path (tx_clk domain)
// ---------------------------------------------------------------
ll_eth_tx #(
    .DATA_WIDTH(DATA_WIDTH),
    .HDR_WIDTH(HDR_WIDTH),
    .SCRAMBLER_DISABLE(SCRAMBLER_DISABLE),
    .MIN_FRAME_LENGTH(MIN_FRAME_LENGTH)
) tx_inst (
    .clk(tx_clk),
    .rst(tx_rst),

    .s_axis_tdata(s_axis_tdata),
    .s_axis_tkeep(s_axis_tkeep),
    .s_axis_tvalid(s_axis_tvalid),
    .s_axis_tready(s_axis_tready),
    .s_axis_tlast(s_axis_tlast),
    .s_axis_tuser(s_axis_tuser),

    .serdes_tx_data(serdes_tx_data),
    .serdes_tx_hdr(serdes_tx_hdr),

    .cfg_ifg(cfg_ifg),

    .tx_start_packet(tx_start_packet),
    .tx_error_underflow(tx_error_underflow)
);

endmodule

`resetall
