/*

GTF DFE Optimization Core — Standalone Top-Level
Integrates all DFE optimization submodules with a unified AXI-Lite register
interface. Usable standalone in any GTF design or wrapped for Corundum.

Register Map (32-bit aligned):
  0x0000  CTRL           RW   Global control
  0x0004  STATUS         RO   Status
  0x0008  IRQ_EN         RW   Interrupt enable
  0x000C  IRQ_STATUS     RO/W1C Interrupt status
  0x0010  TAP_ACTIVE     RO   Active tap bitmask
  0x0014  TAP_COUNT      RO   Active tap count
  0x0020-0x005C TAP_VAL[0:15] RO   Converged tap values
  0x0060-0x009C TAP_OVR[0:15] RW   Override tap values
  0x00A0  EYE_HEIGHT     RO   Eye height
  0x00A4  EYE_WIDTH      RO   Eye width
  0x00A8  EYE_BER        RO   Estimated BER
  0x00AC  EYE_PRESCALE   RW   Eye scan prescale
  0x00B0  EYE_H_RANGE    RW   Horizontal scan range
  0x00B4  EYE_V_RANGE    RW   Vertical scan range
  0x00C0  TX_ALIGN_HIT   RO   TX alignment hit count
  0x00C4  TX_ALIGN_MISS  RO   TX alignment miss count
  0x00C8  TX_ALIGN_CTRL  RW   TX alignment control
  0x00D0  LPM_THRESHOLD  RW   LPM/DFE auto-selection threshold
  0x00D4  CAL_TIMEOUT    RW   Calibration timeout
  0x00D8  CAL_STABILITY  RW   Convergence stability count
  0x00E0  DRP_ADDR       RW   Direct DRP address
  0x00E4  DRP_DATA       RW   Direct DRP data
  0x00E8  DRP_CTRL       RW   Direct DRP control
  0x00EC  DRP_STATUS     RO   Direct DRP status
  0x00F0  SCAN_POINTS    RO   Eye scan point count
  0x00F4  SCAN_BRAM_ADDR RW   Eye scan BRAM read address
  0x00F8  SCAN_BRAM_DATA RO   Eye scan BRAM read data
  0x0100  VERSION        RO   Module version
  0x0104  SCRATCH        RW   Scratch register

*/

`resetall
`timescale 1ns / 1ps
`default_nettype none

module gtf_dfe_opt_core #(
    parameter AXIL_ADDR_WIDTH = 16,
    parameter AXIL_DATA_WIDTH = 32,
    parameter AXIL_STRB_WIDTH = (AXIL_DATA_WIDTH/8),
    parameter NUM_TAPS = 15,
    parameter EYE_BRAM_ADDR_WIDTH = 13,
    parameter VERSION = 32'h47464401  // "GFD\x01"
)
(
    input  wire        clk,
    input  wire        rst,

    /*
     * AXI-Lite slave
     */
    input  wire [AXIL_ADDR_WIDTH-1:0]  s_axil_awaddr,
    input  wire [2:0]                  s_axil_awprot,
    input  wire                        s_axil_awvalid,
    output wire                        s_axil_awready,
    input  wire [AXIL_DATA_WIDTH-1:0]  s_axil_wdata,
    input  wire [AXIL_STRB_WIDTH-1:0]  s_axil_wstrb,
    input  wire                        s_axil_wvalid,
    output wire                        s_axil_wready,
    output wire [1:0]                  s_axil_bresp,
    output wire                        s_axil_bvalid,
    input  wire                        s_axil_bready,
    input  wire [AXIL_ADDR_WIDTH-1:0]  s_axil_araddr,
    input  wire [2:0]                  s_axil_arprot,
    input  wire                        s_axil_arvalid,
    output wire                        s_axil_arready,
    output wire [AXIL_DATA_WIDTH-1:0]  s_axil_rdata,
    output wire [1:0]                  s_axil_rresp,
    output wire                        s_axil_rvalid,
    input  wire                        s_axil_rready,

    /*
     * DRP master to GTF_CHANNEL
     */
    output wire [15:0] gtf_drp_addr,
    output wire [15:0] gtf_drp_di,
    output wire        gtf_drp_en,
    output wire        gtf_drp_we,
    input  wire [15:0] gtf_drp_do,
    input  wire        gtf_drp_rdy,

    /*
     * GTF sideband — DFE/LPM control
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
     * GTF sideband — FCS control (direct ports)
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
     * TX AXI-Stream passthrough (for TX alignment engine)
     */
    input  wire [15:0] s_axis_tx_tdata,
    input  wire        s_axis_tx_tvalid,
    input  wire [1:0]  s_axis_tx_tlast,
    input  wire        s_axis_tx_tsof,
    input  wire        s_axis_tx_terr,
    input  wire [7:0]  s_axis_tx_tpre,
    output wire        s_axis_tx_tready,
    output wire [15:0] m_axis_tx_tdata,
    output wire        m_axis_tx_tvalid,
    output wire [1:0]  m_axis_tx_tlast,
    output wire        m_axis_tx_tsof,
    output wire        m_axis_tx_terr,
    output wire [7:0]  m_axis_tx_tpre,
    input  wire        m_axis_tx_tready,

    /*
     * Interrupt
     */
    output wire        irq
);

// ============================================================
// Register addresses
// ============================================================
localparam
    REG_CTRL          = 16'h0000,
    REG_STATUS        = 16'h0004,
    REG_IRQ_EN        = 16'h0008,
    REG_IRQ_STATUS    = 16'h000C,
    REG_TAP_ACTIVE    = 16'h0010,
    REG_TAP_COUNT     = 16'h0014,
    REG_TAP_VAL_BASE  = 16'h0020,  // 0x0020-0x005C (16 regs)
    REG_TAP_OVR_BASE  = 16'h0060,  // 0x0060-0x009C (16 regs)
    REG_EYE_HEIGHT    = 16'h00A0,
    REG_EYE_WIDTH     = 16'h00A4,
    REG_EYE_BER       = 16'h00A8,
    REG_EYE_PRESCALE  = 16'h00AC,
    REG_EYE_H_RANGE   = 16'h00B0,
    REG_EYE_V_RANGE   = 16'h00B4,
    REG_TX_ALIGN_HIT  = 16'h00C0,
    REG_TX_ALIGN_MISS = 16'h00C4,
    REG_TX_ALIGN_CTRL = 16'h00C8,
    REG_LPM_THRESHOLD = 16'h00D0,
    REG_CAL_TIMEOUT   = 16'h00D4,
    REG_CAL_STABILITY = 16'h00D8,
    REG_DRP_ADDR_REG  = 16'h00E0,
    REG_DRP_DATA      = 16'h00E4,
    REG_DRP_CTRL      = 16'h00E8,
    REG_DRP_STATUS    = 16'h00EC,
    REG_SCAN_POINTS   = 16'h00F0,
    REG_SCAN_BRAM_ADDR = 16'h00F4,
    REG_SCAN_BRAM_DATA = 16'h00F8,
    REG_VERSION       = 16'h0100,
    REG_SCRATCH       = 16'h0104;

// ============================================================
// Control/config registers
// ============================================================
reg [31:0] ctrl_reg;
reg [31:0] irq_en_reg;
reg [31:0] irq_status_reg;
reg [31:0] scratch_reg;
reg [4:0]  eye_prescale_reg;
reg signed [15:0] eye_h_min_reg, eye_h_max_reg;
reg signed [15:0] eye_v_min_reg, eye_v_max_reg;
reg [15:0] lpm_threshold_reg;
reg [31:0] cal_timeout_reg;
reg [31:0] cal_stability_reg;
reg [15:0] host_drp_addr_reg;
reg [15:0] host_drp_data_reg;
reg [1:0]  host_drp_ctrl_reg;  // [0]=start, [1]=wr
reg [1:0]  tx_align_ctrl_reg;  // [0]=force_align, [1]=predictive
reg [4:0]  tap_ovr_regs [0:NUM_TAPS-1];
reg [EYE_BRAM_ADDR_WIDTH-1:0] scan_bram_addr_reg;

// Control bit extraction
wire ctrl_cal_start        = ctrl_reg[0];
wire ctrl_eye_scan_start   = ctrl_reg[1];
wire ctrl_override_enable  = ctrl_reg[2];
wire ctrl_buf_bypass_en    = ctrl_reg[3];
wire ctrl_fcs_check_en     = ctrl_reg[4];
wire ctrl_fcs_delete_en    = ctrl_reg[5];
wire ctrl_fcs_tx_ins_en    = ctrl_reg[6];
wire ctrl_lpm_force        = ctrl_reg[7];
wire ctrl_dfe_force        = ctrl_reg[8];
wire ctrl_auto_mode_en     = ctrl_reg[9];
wire ctrl_quick_scan       = ctrl_reg[10];

// Pulse generators for start signals
reg ctrl_reg_prev_0, ctrl_reg_prev_1, ctrl_reg_prev_2, ctrl_reg_prev_10;
wire cal_start_pulse        = ctrl_reg[0] & ~ctrl_reg_prev_0;
wire eye_scan_start_pulse   = ctrl_reg[1] & ~ctrl_reg_prev_1;
wire override_start_pulse   = ctrl_reg[2] & ~ctrl_reg_prev_2;
wire quick_scan_start_pulse = ctrl_reg[10] & ~ctrl_reg_prev_10;

always @(posedge clk) begin
    if (rst) begin
        ctrl_reg_prev_0 <= 1'b0;
        ctrl_reg_prev_1 <= 1'b0;
        ctrl_reg_prev_2 <= 1'b0;
        ctrl_reg_prev_10 <= 1'b0;
    end else begin
        ctrl_reg_prev_0 <= ctrl_reg[0];
        ctrl_reg_prev_1 <= ctrl_reg[1];
        ctrl_reg_prev_2 <= ctrl_reg[2];
        ctrl_reg_prev_10 <= ctrl_reg[10];
    end
end

// ============================================================
// Submodule wires
// ============================================================

// DRP controller
wire [15:0] cal_drp_addr, cal_drp_wdata;
wire        cal_drp_valid, cal_drp_wr;
wire [15:0] cal_drp_rdata;
wire        cal_drp_done, cal_drp_grant;

wire [15:0] eye_drp_addr, eye_drp_wdata;
wire        eye_drp_valid, eye_drp_wr;
wire [15:0] eye_drp_rdata;
wire        eye_drp_done, eye_drp_grant;

wire [15:0] host_drp_rdata_w;
wire        host_drp_done_w, host_drp_grant_w;

// Calibration FSM
wire        cal_done_w, cal_busy_w, cal_error_w;
wire [3:0]  cal_active_tap_count;
wire [NUM_TAPS*2-1:0] cal_tap_hold_ovrden;
wire        cal_rxdfelpmreset;
wire [4:0]  cal_tap_val [0:NUM_TAPS-1];
wire [14:0] cal_tap_active_mask;

// Eye scan engine
wire        eye_scan_done_w, eye_scan_busy_w;
wire [15:0] eye_height_w, eye_width_w;
wire [31:0] eye_ber_w;
wire [EYE_BRAM_ADDR_WIDTH-1:0] eye_scan_point_count;
wire [31:0] eye_bram_rd_data;
wire        eye_eyescanreset;

// LPM/DFE selector
wire        sel_rxlpmen;
wire        sel_lpm_active, sel_dfe_active;
wire [15:0] sel_current_margin;
wire        sel_quick_scan_trigger;
wire        sel_cal_trigger;

// Buffer bypass
wire        bbp_tx_aligned, bbp_rx_aligned, bbp_aligned, bbp_error;
wire [15:0] bbp_drp_addr, bbp_drp_wdata;
wire        bbp_drp_valid, bbp_drp_wr;
wire [15:0] bbp_drp_rdata;
wire        bbp_drp_done;

// TX alignment
wire [31:0] tx_align_hit_w, tx_align_miss_w;
wire [4:0]  tx_last_wait_w;

// ============================================================
// DRP Controller
// ============================================================
gtf_drp_controller drp_ctrl_inst (
    .clk(clk),
    .rst(rst),
    .drp_addr(gtf_drp_addr),
    .drp_di(gtf_drp_di),
    .drp_en(gtf_drp_en),
    .drp_we(gtf_drp_we),
    .drp_do(gtf_drp_do),
    .drp_rdy(gtf_drp_rdy),
    // Requestor 0: Calibration FSM
    .req0_addr(cal_drp_addr),
    .req0_wdata(cal_drp_wdata),
    .req0_valid(cal_drp_valid),
    .req0_wr(cal_drp_wr),
    .req0_rdata(cal_drp_rdata),
    .req0_done(cal_drp_done),
    .req0_grant(cal_drp_grant),
    // Requestor 1: Eye Scan Engine
    .req1_addr(eye_drp_addr),
    .req1_wdata(eye_drp_wdata),
    .req1_valid(eye_drp_valid),
    .req1_wr(eye_drp_wr),
    .req1_rdata(eye_drp_rdata),
    .req1_done(eye_drp_done),
    .req1_grant(eye_drp_grant),
    // Requestor 2: Host Direct Access
    .req2_addr(host_drp_addr_reg),
    .req2_wdata(host_drp_data_reg),
    .req2_valid(host_drp_ctrl_reg[0]),
    .req2_wr(host_drp_ctrl_reg[1]),
    .req2_rdata(host_drp_rdata_w),
    .req2_done(host_drp_done_w),
    .req2_grant(host_drp_grant_w)
);

// ============================================================
// Calibration FSM
// ============================================================
gtf_cal_fsm cal_fsm_inst (
    .clk(clk),
    .rst(rst),
    .cal_start(cal_start_pulse | sel_cal_trigger),
    .override_start(override_start_pulse),
    .override_val_0(tap_ovr_regs[0]),
    .override_val_1(tap_ovr_regs[1]),
    .override_val_2(tap_ovr_regs[2]),
    .override_val_3(tap_ovr_regs[3]),
    .override_val_4(tap_ovr_regs[4]),
    .override_val_5(tap_ovr_regs[5]),
    .override_val_6(tap_ovr_regs[6]),
    .override_val_7(tap_ovr_regs[7]),
    .override_val_8(tap_ovr_regs[8]),
    .override_val_9(tap_ovr_regs[9]),
    .override_val_10(tap_ovr_regs[10]),
    .override_val_11(tap_ovr_regs[11]),
    .override_val_12(tap_ovr_regs[12]),
    .override_val_13(tap_ovr_regs[13]),
    .override_val_14(tap_ovr_regs[14]),
    .cal_done(cal_done_w),
    .cal_busy(cal_busy_w),
    .cal_error(cal_error_w),
    .active_tap_count(cal_active_tap_count),
    .drp_addr(cal_drp_addr),
    .drp_wdata(cal_drp_wdata),
    .drp_valid(cal_drp_valid),
    .drp_wr(cal_drp_wr),
    .drp_rdata(cal_drp_rdata),
    .drp_done(cal_drp_done),
    .drp_grant(cal_drp_grant),
    .tap_hold_ovrden(cal_tap_hold_ovrden),
    .rxdfelpmreset(cal_rxdfelpmreset),
    .dmonitorout(gtf_dmonitorout),
    .tap_val_0(cal_tap_val[0]),
    .tap_val_1(cal_tap_val[1]),
    .tap_val_2(cal_tap_val[2]),
    .tap_val_3(cal_tap_val[3]),
    .tap_val_4(cal_tap_val[4]),
    .tap_val_5(cal_tap_val[5]),
    .tap_val_6(cal_tap_val[6]),
    .tap_val_7(cal_tap_val[7]),
    .tap_val_8(cal_tap_val[8]),
    .tap_val_9(cal_tap_val[9]),
    .tap_val_10(cal_tap_val[10]),
    .tap_val_11(cal_tap_val[11]),
    .tap_val_12(cal_tap_val[12]),
    .tap_val_13(cal_tap_val[13]),
    .tap_val_14(cal_tap_val[14]),
    .tap_active_mask(cal_tap_active_mask)
);

assign gtf_tap_hold_ovrden = cal_tap_hold_ovrden;
assign gtf_rxdfelpmreset   = cal_rxdfelpmreset;

// ============================================================
// Eye Scan Engine
// ============================================================
gtf_eye_scan_engine eye_scan_inst (
    .clk(clk),
    .rst(rst),
    .scan_start(eye_scan_start_pulse),
    .quick_scan_start(quick_scan_start_pulse | sel_quick_scan_trigger),
    .prescale(eye_prescale_reg),
    .h_min(eye_h_min_reg),
    .h_max(eye_h_max_reg),
    .v_min(eye_v_min_reg),
    .v_max(eye_v_max_reg),
    .scan_done(eye_scan_done_w),
    .scan_busy(eye_scan_busy_w),
    .eye_height(eye_height_w),
    .eye_width(eye_width_w),
    .eye_ber(eye_ber_w),
    .scan_point_count(eye_scan_point_count),
    .bram_rd_addr(scan_bram_addr_reg),
    .bram_rd_data(eye_bram_rd_data),
    .drp_addr(eye_drp_addr),
    .drp_wdata(eye_drp_wdata),
    .drp_valid(eye_drp_valid),
    .drp_wr(eye_drp_wr),
    .drp_rdata(eye_drp_rdata),
    .drp_done(eye_drp_done),
    .drp_grant(eye_drp_grant),
    .eyescanreset(eye_eyescanreset),
    .eyescandataerror(gtf_eyescandataerror)
);

assign gtf_eyescanreset = eye_eyescanreset;

// ============================================================
// LPM/DFE Selector
// ============================================================
gtf_lpm_dfe_selector lpm_dfe_sel_inst (
    .clk(clk),
    .rst(rst),
    .auto_enable(ctrl_auto_mode_en),
    .force_lpm(ctrl_lpm_force),
    .force_dfe(ctrl_dfe_force),
    .threshold(lpm_threshold_reg),
    .lpm_active(sel_lpm_active),
    .dfe_active(sel_dfe_active),
    .current_margin(sel_current_margin),
    .selector_state(),
    .rxlpmen(sel_rxlpmen),
    .quick_scan_trigger(sel_quick_scan_trigger),
    .scan_done(eye_scan_done_w),
    .eye_height(eye_height_w),
    .cal_trigger(sel_cal_trigger),
    .cal_done(cal_done_w)
);

assign gtf_rxlpmen = sel_rxlpmen;

// ============================================================
// Buffer Bypass Controller
// ============================================================
gtf_buf_bypass_ctrl buf_bypass_inst (
    .clk(clk),
    .rst(rst),
    .buf_bypass_enable(ctrl_buf_bypass_en),
    .tx_bypass_aligned(bbp_tx_aligned),
    .rx_bypass_aligned(bbp_rx_aligned),
    .buf_bypass_aligned(bbp_aligned),
    .buf_bypass_error(bbp_error),
    .drp_addr(bbp_drp_addr),
    .drp_wdata(bbp_drp_wdata),
    .drp_valid(bbp_drp_valid),
    .drp_wr(bbp_drp_wr),
    .drp_rdata(bbp_drp_rdata),
    .drp_done(bbp_drp_done),
    .txphdlyreset(gtf_txphdlyreset),
    .txphdlytstclk(gtf_txphdlytstclk),
    .txphalign(gtf_txphalign),
    .txphalignen(gtf_txphalignen),
    .txphdlypd(gtf_txphdlypd),
    .txphaligndone(gtf_txphaligndone),
    .txsyncdone(gtf_txsyncdone),
    .rxphdlyreset(gtf_rxphdlyreset),
    .rxphalign(gtf_rxphalign),
    .rxphalignen(gtf_rxphalignen),
    .rxphdlypd(gtf_rxphdlypd),
    .rxphaligndone(gtf_rxphaligndone),
    .rxsyncdone(gtf_rxsyncdone),
    .txresetdone(gtf_txresetdone),
    .rxresetdone(gtf_rxresetdone),
    .rxcdrlock(gtf_rxcdrlock)
);

// ============================================================
// FCS Bypass
// ============================================================
gtf_rx_fcs_bypass fcs_bypass_inst (
    .clk(clk),
    .rst(rst),
    .fcs_check_enable(ctrl_fcs_check_en),
    .fcs_delete_enable(ctrl_fcs_delete_en),
    .fcs_tx_ins_enable(ctrl_fcs_tx_ins_en),
    .ctl_rx_check_fcs(gtf_ctl_rx_check_fcs),
    .ctl_rx_delete_fcs(gtf_ctl_rx_delete_fcs),
    .ctl_tx_fcs_ins_enable(gtf_ctl_tx_fcs_ins_enable),
    .fcs_check_active(),
    .fcs_delete_active(),
    .fcs_tx_ins_active()
);

// ============================================================
// TX Alignment Engine
// ============================================================
gtf_tx_align_engine tx_align_inst (
    .txusrclk(clk),  // Note: in real design, use txusrclk with CDC
    .txusrrst(rst),
    .txaxistcanstart(gtf_txaxistcanstart),
    .txaxistready(gtf_txaxistready),
    .txgbseqstart(gtf_txgbseqstart),
    .s_axis_tdata(s_axis_tx_tdata),
    .s_axis_tvalid(s_axis_tx_tvalid),
    .s_axis_tlast(s_axis_tx_tlast),
    .s_axis_tsof(s_axis_tx_tsof),
    .s_axis_terr(s_axis_tx_terr),
    .s_axis_tpre(s_axis_tx_tpre),
    .s_axis_tready(s_axis_tx_tready),
    .m_axis_tdata(m_axis_tx_tdata),
    .m_axis_tvalid(m_axis_tx_tvalid),
    .m_axis_tlast(m_axis_tx_tlast),
    .m_axis_tsof(m_axis_tx_tsof),
    .m_axis_terr(m_axis_tx_terr),
    .m_axis_tpre(m_axis_tx_tpre),
    .m_axis_tready(m_axis_tx_tready),
    .force_align(tx_align_ctrl_reg[0]),
    .predictive_en(tx_align_ctrl_reg[1]),
    .align_hit_count(tx_align_hit_w),
    .align_miss_count(tx_align_miss_w),
    .last_wait_cycles(tx_last_wait_w)
);

// ============================================================
// AXI-Lite Register Interface
// ============================================================

// AXI-Lite handshake
reg axil_awready_reg;
reg axil_wready_reg;
reg axil_bvalid_reg;
reg axil_arready_reg;
reg axil_rvalid_reg;
reg [AXIL_DATA_WIDTH-1:0] axil_rdata_reg;

assign s_axil_awready = axil_awready_reg;
assign s_axil_wready  = axil_wready_reg;
assign s_axil_bresp   = 2'b00;
assign s_axil_bvalid  = axil_bvalid_reg;
assign s_axil_arready = axil_arready_reg;
assign s_axil_rdata   = axil_rdata_reg;
assign s_axil_rresp   = 2'b00;
assign s_axil_rvalid  = axil_rvalid_reg;

// Write channel
reg [AXIL_ADDR_WIDTH-1:0] wr_addr_reg;
reg wr_addr_valid;

integer k;

always @(posedge clk) begin
    if (rst) begin
        axil_awready_reg <= 1'b0;
        axil_wready_reg  <= 1'b0;
        axil_bvalid_reg  <= 1'b0;
        wr_addr_valid    <= 1'b0;
        ctrl_reg         <= 32'd0;
        irq_en_reg       <= 32'd0;
        irq_status_reg   <= 32'd0;
        scratch_reg      <= 32'd0;
        eye_prescale_reg <= 5'd5;
        eye_h_min_reg    <= -16'sd32;
        eye_h_max_reg    <= 16'sd32;
        eye_v_min_reg    <= -16'sd64;
        eye_v_max_reg    <= 16'sd64;
        lpm_threshold_reg <= 16'd100;
        cal_timeout_reg  <= 32'd10000000;
        cal_stability_reg <= 32'd64;
        host_drp_addr_reg <= 16'd0;
        host_drp_data_reg <= 16'd0;
        host_drp_ctrl_reg <= 2'b00;
        tx_align_ctrl_reg <= 2'b00;
        scan_bram_addr_reg <= {EYE_BRAM_ADDR_WIDTH{1'b0}};
        for (k = 0; k < NUM_TAPS; k = k + 1)
            tap_ovr_regs[k] <= 5'd0;
    end else begin
        // AW channel
        if (s_axil_awvalid && !axil_awready_reg && !wr_addr_valid) begin
            axil_awready_reg <= 1'b1;
            wr_addr_reg <= s_axil_awaddr;
            wr_addr_valid <= 1'b1;
        end else begin
            axil_awready_reg <= 1'b0;
        end

        // W channel
        if (s_axil_wvalid && wr_addr_valid && !axil_wready_reg) begin
            axil_wready_reg <= 1'b1;
            wr_addr_valid <= 1'b0;

            case (wr_addr_reg[15:0])
                REG_CTRL:          ctrl_reg <= s_axil_wdata;
                REG_IRQ_EN:        irq_en_reg <= s_axil_wdata;
                REG_IRQ_STATUS:    irq_status_reg <= irq_status_reg & ~s_axil_wdata; // W1C
                REG_EYE_PRESCALE:  eye_prescale_reg <= s_axil_wdata[4:0];
                REG_EYE_H_RANGE: begin
                    eye_h_min_reg <= s_axil_wdata[15:0];
                    eye_h_max_reg <= s_axil_wdata[31:16];
                end
                REG_EYE_V_RANGE: begin
                    eye_v_min_reg <= s_axil_wdata[15:0];
                    eye_v_max_reg <= s_axil_wdata[31:16];
                end
                REG_TX_ALIGN_CTRL: tx_align_ctrl_reg <= s_axil_wdata[1:0];
                REG_LPM_THRESHOLD: lpm_threshold_reg <= s_axil_wdata[15:0];
                REG_CAL_TIMEOUT:   cal_timeout_reg <= s_axil_wdata;
                REG_CAL_STABILITY: cal_stability_reg <= s_axil_wdata;
                REG_DRP_ADDR_REG:  host_drp_addr_reg <= s_axil_wdata[15:0];
                REG_DRP_DATA:      host_drp_data_reg <= s_axil_wdata[15:0];
                REG_DRP_CTRL:      host_drp_ctrl_reg <= s_axil_wdata[1:0];
                REG_SCAN_BRAM_ADDR: scan_bram_addr_reg <= s_axil_wdata[EYE_BRAM_ADDR_WIDTH-1:0];
                REG_SCRATCH:       scratch_reg <= s_axil_wdata;
                default: begin
                    // TAP_OVR registers: 0x0060 + idx*4
                    if (wr_addr_reg >= REG_TAP_OVR_BASE &&
                        wr_addr_reg < REG_TAP_OVR_BASE + NUM_TAPS*4) begin
                        tap_ovr_regs[(wr_addr_reg - REG_TAP_OVR_BASE) >> 2] <= s_axil_wdata[4:0];
                    end
                end
            endcase
        end else begin
            axil_wready_reg <= 1'b0;
        end

        // B channel
        if (axil_wready_reg && !axil_bvalid_reg) begin
            axil_bvalid_reg <= 1'b1;
        end else if (s_axil_bready && axil_bvalid_reg) begin
            axil_bvalid_reg <= 1'b0;
        end

        // Auto-clear DRP start after done
        if (host_drp_done_w) begin
            host_drp_ctrl_reg[0] <= 1'b0;
            host_drp_data_reg <= host_drp_rdata_w;
        end

        // Auto-clear cal/scan start bits
        if (cal_done_w)       ctrl_reg[0] <= 1'b0;
        if (eye_scan_done_w)  ctrl_reg[1] <= 1'b0;
        if (cal_done_w)       ctrl_reg[2] <= 1'b0;
        if (eye_scan_done_w)  ctrl_reg[10] <= 1'b0;

        // IRQ status latching
        if (cal_done_w)      irq_status_reg[0] <= 1'b1;
        if (eye_scan_done_w) irq_status_reg[1] <= 1'b1;
        if (cal_error_w)     irq_status_reg[2] <= 1'b1;
    end
end

// Read channel
always @(posedge clk) begin
    if (rst) begin
        axil_arready_reg <= 1'b0;
        axil_rvalid_reg  <= 1'b0;
        axil_rdata_reg   <= 32'd0;
    end else begin
        if (s_axil_arvalid && !axil_arready_reg && !axil_rvalid_reg) begin
            axil_arready_reg <= 1'b1;
            axil_rvalid_reg  <= 1'b1;

            case (s_axil_araddr[15:0])
                REG_CTRL:          axil_rdata_reg <= ctrl_reg;
                REG_STATUS:        axil_rdata_reg <= {23'd0,
                                                      bbp_aligned,       // [8]
                                                      sel_dfe_active,    // [7]
                                                      sel_lpm_active,    // [6]
                                                      1'b0,              // [5]
                                                      gtf_rxcdrlock,     // [4]
                                                      eye_scan_busy_w,   // [3]
                                                      eye_scan_done_w,   // [2]
                                                      cal_busy_w,        // [1]
                                                      cal_done_w};       // [0]
                REG_IRQ_EN:        axil_rdata_reg <= irq_en_reg;
                REG_IRQ_STATUS:    axil_rdata_reg <= irq_status_reg;
                REG_TAP_ACTIVE:    axil_rdata_reg <= {17'd0, cal_tap_active_mask};
                REG_TAP_COUNT:     axil_rdata_reg <= {28'd0, cal_active_tap_count};
                REG_EYE_HEIGHT:    axil_rdata_reg <= {16'd0, eye_height_w};
                REG_EYE_WIDTH:     axil_rdata_reg <= {16'd0, eye_width_w};
                REG_EYE_BER:       axil_rdata_reg <= eye_ber_w;
                REG_EYE_PRESCALE:  axil_rdata_reg <= {27'd0, eye_prescale_reg};
                REG_EYE_H_RANGE:   axil_rdata_reg <= {eye_h_max_reg, eye_h_min_reg};
                REG_EYE_V_RANGE:   axil_rdata_reg <= {eye_v_max_reg, eye_v_min_reg};
                REG_TX_ALIGN_HIT:  axil_rdata_reg <= tx_align_hit_w;
                REG_TX_ALIGN_MISS: axil_rdata_reg <= tx_align_miss_w;
                REG_TX_ALIGN_CTRL: axil_rdata_reg <= {30'd0, tx_align_ctrl_reg};
                REG_LPM_THRESHOLD: axil_rdata_reg <= {16'd0, lpm_threshold_reg};
                REG_CAL_TIMEOUT:   axil_rdata_reg <= cal_timeout_reg;
                REG_CAL_STABILITY: axil_rdata_reg <= cal_stability_reg;
                REG_DRP_ADDR_REG:  axil_rdata_reg <= {16'd0, host_drp_addr_reg};
                REG_DRP_DATA:      axil_rdata_reg <= {16'd0, host_drp_data_reg};
                REG_DRP_CTRL:      axil_rdata_reg <= {30'd0, host_drp_ctrl_reg};
                REG_DRP_STATUS:    axil_rdata_reg <= {30'd0, host_drp_done_w, host_drp_ctrl_reg[0]};
                REG_SCAN_POINTS:   axil_rdata_reg <= {{(32-EYE_BRAM_ADDR_WIDTH){1'b0}}, eye_scan_point_count};
                REG_SCAN_BRAM_ADDR: axil_rdata_reg <= {{(32-EYE_BRAM_ADDR_WIDTH){1'b0}}, scan_bram_addr_reg};
                REG_SCAN_BRAM_DATA: axil_rdata_reg <= eye_bram_rd_data;
                REG_VERSION:       axil_rdata_reg <= VERSION;
                REG_SCRATCH:       axil_rdata_reg <= scratch_reg;
                default: begin
                    // TAP_VAL registers
                    if (s_axil_araddr >= REG_TAP_VAL_BASE &&
                        s_axil_araddr < REG_TAP_VAL_BASE + NUM_TAPS*4)
                        axil_rdata_reg <= {27'd0, cal_tap_val[(s_axil_araddr - REG_TAP_VAL_BASE) >> 2]};
                    // TAP_OVR registers
                    else if (s_axil_araddr >= REG_TAP_OVR_BASE &&
                             s_axil_araddr < REG_TAP_OVR_BASE + NUM_TAPS*4)
                        axil_rdata_reg <= {27'd0, tap_ovr_regs[(s_axil_araddr - REG_TAP_OVR_BASE) >> 2]};
                    else
                        axil_rdata_reg <= 32'hDEADBEEF;
                end
            endcase
        end else begin
            axil_arready_reg <= 1'b0;
        end

        if (s_axil_rready && axil_rvalid_reg) begin
            axil_rvalid_reg <= 1'b0;
        end
    end
end

// ============================================================
// Interrupt
// ============================================================
assign irq = |(irq_status_reg & irq_en_reg);

endmodule

`resetall
