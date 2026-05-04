/*

GTF Calibration FSM
Automated DFE calibration: converges tap weights via auto-adapt, reads
converged values from DMONITOROUT, freezes taps, and minimizes active
tap count. Supports override mode for instant loading of known-good values.

Per UG1549 Table 42, DMONITOROUT tap selection is controlled by
RXDFE_CFG1[4:0] via DRP, and values are read from DMONITOROUT[6:0].

*/

`resetall
`timescale 1ns / 1ps
`default_nettype none

module gtf_cal_fsm #(
    parameter NUM_TAPS = 15,
    parameter DMON_WIDTH = 16,
    parameter DRP_ADDR_WIDTH = 16,
    parameter DRP_DATA_WIDTH = 16,
    parameter CONVERGENCE_COUNT = 64,
    parameter CONVERGENCE_THRESHOLD = 1,
    parameter TIMEOUT_CYCLES = 10000000,  // ~15ms at 644MHz
    parameter SETTLE_CYCLES = 644000,     // ~1ms settle time

    // DRP addresses (verify against GTF Wizard output for your part)
    parameter RXDFE_CFG1_ADDR = 16'h004C,
    // TAP override register addresses (RXDFE_Hx_CFG1)
    parameter RXDFE_H2_CFG1_ADDR  = 16'h0063,
    parameter RXDFE_H3_CFG1_ADDR  = 16'h0065,
    parameter RXDFE_H4_CFG1_ADDR  = 16'h0067,
    parameter RXDFE_H5_CFG1_ADDR  = 16'h0069,
    parameter RXDFE_H6_CFG1_ADDR  = 16'h006B,
    parameter RXDFE_H7_CFG1_ADDR  = 16'h006D,
    parameter RXDFE_H8_CFG1_ADDR  = 16'h006F,
    parameter RXDFE_H9_CFG1_ADDR  = 16'h0071,
    parameter RXDFE_HA_CFG1_ADDR  = 16'h0073,
    parameter RXDFE_HB_CFG1_ADDR  = 16'h0075,
    parameter RXDFE_HC_CFG1_ADDR  = 16'h0077,
    parameter RXDFE_HD_CFG1_ADDR  = 16'h0079,
    parameter RXDFE_HE_CFG1_ADDR  = 16'h007B,
    parameter RXDFE_HF_CFG1_ADDR  = 16'h007D,
    // Minimum absolute tap value to consider "active"
    parameter TAP_ZERO_THRESHOLD = 2
)
(
    input  wire        clk,
    input  wire        rst,

    /*
     * Control
     */
    input  wire        cal_start,
    input  wire        override_start,
    input  wire [4:0]  override_val_0,
    input  wire [4:0]  override_val_1,
    input  wire [4:0]  override_val_2,
    input  wire [4:0]  override_val_3,
    input  wire [4:0]  override_val_4,
    input  wire [4:0]  override_val_5,
    input  wire [4:0]  override_val_6,
    input  wire [4:0]  override_val_7,
    input  wire [4:0]  override_val_8,
    input  wire [4:0]  override_val_9,
    input  wire [4:0]  override_val_10,
    input  wire [4:0]  override_val_11,
    input  wire [4:0]  override_val_12,
    input  wire [4:0]  override_val_13,
    input  wire [4:0]  override_val_14,

    /*
     * Status
     */
    output reg         cal_done,
    output reg         cal_busy,
    output reg         cal_error,
    output reg  [3:0]  active_tap_count,

    /*
     * DRP requestor interface
     */
    output reg  [DRP_ADDR_WIDTH-1:0]  drp_addr,
    output reg  [DRP_DATA_WIDTH-1:0]  drp_wdata,
    output reg                        drp_valid,
    output reg                        drp_wr,
    input  wire [DRP_DATA_WIDTH-1:0]  drp_rdata,
    input  wire                       drp_done,
    input  wire                       drp_grant,

    /*
     * GTF sideband
     */
    output reg  [NUM_TAPS*2-1:0]      tap_hold_ovrden,
    output reg                        rxdfelpmreset,
    input  wire [DMON_WIDTH-1:0]      dmonitorout,

    /*
     * Tap readback (active values after calibration)
     */
    output reg  [4:0]  tap_val_0,
    output reg  [4:0]  tap_val_1,
    output reg  [4:0]  tap_val_2,
    output reg  [4:0]  tap_val_3,
    output reg  [4:0]  tap_val_4,
    output reg  [4:0]  tap_val_5,
    output reg  [4:0]  tap_val_6,
    output reg  [4:0]  tap_val_7,
    output reg  [4:0]  tap_val_8,
    output reg  [4:0]  tap_val_9,
    output reg  [4:0]  tap_val_10,
    output reg  [4:0]  tap_val_11,
    output reg  [4:0]  tap_val_12,
    output reg  [4:0]  tap_val_13,
    output reg  [4:0]  tap_val_14,
    output reg  [14:0] tap_active_mask
);

// Store override values in array for indexed access
wire [4:0] override_vals [0:NUM_TAPS-1];
assign override_vals[0]  = override_val_0;
assign override_vals[1]  = override_val_1;
assign override_vals[2]  = override_val_2;
assign override_vals[3]  = override_val_3;
assign override_vals[4]  = override_val_4;
assign override_vals[5]  = override_val_5;
assign override_vals[6]  = override_val_6;
assign override_vals[7]  = override_val_7;
assign override_vals[8]  = override_val_8;
assign override_vals[9]  = override_val_9;
assign override_vals[10] = override_val_10;
assign override_vals[11] = override_val_11;
assign override_vals[12] = override_val_12;
assign override_vals[13] = override_val_13;
assign override_vals[14] = override_val_14;

// DRP address lookup table for tap override registers
wire [DRP_ADDR_WIDTH-1:0] tap_drp_addr [0:NUM_TAPS-1];
assign tap_drp_addr[0]  = RXDFE_H2_CFG1_ADDR;   // TAP1 uses H2 (first post-cursor)
assign tap_drp_addr[1]  = RXDFE_H3_CFG1_ADDR;
assign tap_drp_addr[2]  = RXDFE_H4_CFG1_ADDR;
assign tap_drp_addr[3]  = RXDFE_H5_CFG1_ADDR;
assign tap_drp_addr[4]  = RXDFE_H6_CFG1_ADDR;
assign tap_drp_addr[5]  = RXDFE_H7_CFG1_ADDR;
assign tap_drp_addr[6]  = RXDFE_H8_CFG1_ADDR;
assign tap_drp_addr[7]  = RXDFE_H9_CFG1_ADDR;
assign tap_drp_addr[8]  = RXDFE_HA_CFG1_ADDR;
assign tap_drp_addr[9]  = RXDFE_HB_CFG1_ADDR;
assign tap_drp_addr[10] = RXDFE_HC_CFG1_ADDR;
assign tap_drp_addr[11] = RXDFE_HD_CFG1_ADDR;
assign tap_drp_addr[12] = RXDFE_HE_CFG1_ADDR;
assign tap_drp_addr[13] = RXDFE_HF_CFG1_ADDR;
assign tap_drp_addr[14] = RXDFE_HF_CFG1_ADDR;  // TAP15 shares register with TAP14

// DMONITOROUT selection codes for each tap
// Per UG1549 Table 42
wire [4:0] dmon_sel [0:NUM_TAPS-1];
assign dmon_sel[0]  = 5'b00010;  // RXDFETAP2
assign dmon_sel[1]  = 5'b00011;  // RXDFETAP3
assign dmon_sel[2]  = 5'b01000;  // RXDFETAP4
assign dmon_sel[3]  = 5'b01001;  // RXDFETAP5
assign dmon_sel[4]  = 5'b01010;  // RXDFETAP6
assign dmon_sel[5]  = 5'b01011;  // RXDFETAP7
assign dmon_sel[6]  = 5'b01100;  // RXDFETAP8
assign dmon_sel[7]  = 5'b01101;  // RXDFETAP9
assign dmon_sel[8]  = 5'b01110;  // RXDFETAPA
assign dmon_sel[9]  = 5'b01111;  // RXDFETAPB
assign dmon_sel[10] = 5'b10000;  // RXDFETAPC
assign dmon_sel[11] = 5'b10001;  // RXDFETAPD
assign dmon_sel[12] = 5'b10010;  // RXDFETAPE
assign dmon_sel[13] = 5'b10011;  // RXDFETAPF
assign dmon_sel[14] = 5'b10011;  // TAP15

// State encoding
localparam [4:0]
    ST_IDLE           = 5'd0,
    ST_CAL_RESET_DFE  = 5'd1,
    ST_CAL_RELEASE    = 5'd2,
    ST_CAL_SETTLE     = 5'd3,
    ST_CAL_SEL_TAP    = 5'd4,
    ST_CAL_SEL_WAIT   = 5'd5,
    ST_CAL_READ_DMON  = 5'd6,
    ST_CAL_CHECK      = 5'd7,
    ST_CAL_NEXT_TAP   = 5'd8,
    ST_CAL_FREEZE     = 5'd9,
    ST_CAL_MINIMIZE   = 5'd10,
    ST_CAL_DONE       = 5'd11,
    ST_OVR_READ_REG   = 5'd12,
    ST_OVR_WRITE_REG  = 5'd13,
    ST_OVR_SET_OVRDEN = 5'd14,
    ST_OVR_DONE       = 5'd15,
    ST_ERROR          = 5'd16;

reg [4:0]  state_reg, state_next;
reg [3:0]  tap_idx;        // current tap being processed
reg [31:0] cycle_cnt;      // general-purpose counter
reg [6:0]  stable_cnt;     // convergence stability counter
reg [6:0]  prev_dmon_val;  // previous DMONITOROUT reading
reg [6:0]  curr_dmon_val;  // current reading

// Tap value storage
reg [4:0] tap_vals [0:NUM_TAPS-1];

// DRP read/modify/write saved data
reg [DRP_DATA_WIDTH-1:0] drp_rmw_data;

integer i;

always @(posedge clk) begin
    if (rst) begin
        state_reg        <= ST_IDLE;
        tap_idx          <= 4'd0;
        cycle_cnt        <= 32'd0;
        stable_cnt       <= 7'd0;
        prev_dmon_val    <= 7'd0;
        curr_dmon_val    <= 7'd0;
        cal_done         <= 1'b0;
        cal_busy         <= 1'b0;
        cal_error        <= 1'b0;
        active_tap_count <= 4'd0;
        tap_hold_ovrden  <= {NUM_TAPS*2{1'b0}};  // all auto-adapt
        rxdfelpmreset    <= 1'b0;
        drp_addr         <= {DRP_ADDR_WIDTH{1'b0}};
        drp_wdata        <= {DRP_DATA_WIDTH{1'b0}};
        drp_valid        <= 1'b0;
        drp_wr           <= 1'b0;
        drp_rmw_data     <= {DRP_DATA_WIDTH{1'b0}};
        tap_active_mask  <= 15'd0;
        tap_val_0  <= 5'd0; tap_val_1  <= 5'd0; tap_val_2  <= 5'd0;
        tap_val_3  <= 5'd0; tap_val_4  <= 5'd0; tap_val_5  <= 5'd0;
        tap_val_6  <= 5'd0; tap_val_7  <= 5'd0; tap_val_8  <= 5'd0;
        tap_val_9  <= 5'd0; tap_val_10 <= 5'd0; tap_val_11 <= 5'd0;
        tap_val_12 <= 5'd0; tap_val_13 <= 5'd0; tap_val_14 <= 5'd0;
        for (i = 0; i < NUM_TAPS; i = i + 1)
            tap_vals[i] <= 5'd0;
    end else begin
        cal_done  <= 1'b0;
        drp_valid <= 1'b0;

        case (state_reg)
            ST_IDLE: begin
                cal_busy <= 1'b0;
                if (cal_start) begin
                    cal_busy  <= 1'b1;
                    cal_error <= 1'b0;
                    cycle_cnt <= 32'd0;
                    state_reg <= ST_CAL_RESET_DFE;
                end else if (override_start) begin
                    cal_busy  <= 1'b1;
                    cal_error <= 1'b0;
                    tap_idx   <= 4'd0;
                    state_reg <= ST_OVR_READ_REG;
                end
            end

            // ---- CALIBRATION MODE ----

            // Assert RXDFELPMRESET for 16 cycles
            ST_CAL_RESET_DFE: begin
                rxdfelpmreset <= 1'b1;
                cycle_cnt <= cycle_cnt + 1;
                if (cycle_cnt >= 32'd16) begin
                    rxdfelpmreset <= 1'b0;
                    cycle_cnt <= 32'd0;
                    // Set all taps to auto-adapt {HOLD=0, OVRDEN=0}
                    tap_hold_ovrden <= {NUM_TAPS*2{1'b0}};
                    state_reg <= ST_CAL_SETTLE;
                end
            end

            // Wait for adaptation to settle
            ST_CAL_SETTLE: begin
                cycle_cnt <= cycle_cnt + 1;
                if (cycle_cnt >= SETTLE_CYCLES) begin
                    tap_idx    <= 4'd0;
                    cycle_cnt  <= 32'd0;
                    state_reg  <= ST_CAL_SEL_TAP;
                end
            end

            // Select tap via RXDFE_CFG1 DRP write
            ST_CAL_SEL_TAP: begin
                drp_addr  <= RXDFE_CFG1_ADDR;
                drp_wdata <= {10'b0, 1'b1, dmon_sel[tap_idx]};  // [5]=enable, [4:0]=select
                drp_valid <= 1'b1;
                drp_wr    <= 1'b1;
                state_reg <= ST_CAL_SEL_WAIT;
            end

            // Wait for DRP write to complete, then settle
            ST_CAL_SEL_WAIT: begin
                if (drp_done) begin
                    cycle_cnt <= 32'd0;
                    stable_cnt <= 7'd0;
                    prev_dmon_val <= 7'd0;
                    state_reg <= ST_CAL_READ_DMON;
                end
            end

            // Read DMONITOROUT (direct port, no DRP needed)
            ST_CAL_READ_DMON: begin
                // Allow a few cycles for DMONITOROUT to update after selection
                cycle_cnt <= cycle_cnt + 1;
                if (cycle_cnt >= 32'd8) begin
                    curr_dmon_val <= dmonitorout[6:0];
                    state_reg <= ST_CAL_CHECK;
                end
            end

            // Check convergence
            ST_CAL_CHECK: begin
                if (stable_cnt == 7'd0) begin
                    // First reading
                    prev_dmon_val <= curr_dmon_val;
                    stable_cnt <= 7'd1;
                    cycle_cnt <= 32'd0;
                    state_reg <= ST_CAL_READ_DMON;
                end else begin
                    // Compare with previous
                    if ((curr_dmon_val >= prev_dmon_val ?
                         curr_dmon_val - prev_dmon_val :
                         prev_dmon_val - curr_dmon_val) <= CONVERGENCE_THRESHOLD) begin
                        stable_cnt <= stable_cnt + 1;
                    end else begin
                        stable_cnt <= 7'd0;
                    end
                    prev_dmon_val <= curr_dmon_val;

                    if (stable_cnt >= CONVERGENCE_COUNT) begin
                        // Converged — store value
                        tap_vals[tap_idx] <= curr_dmon_val[4:0];
                        state_reg <= ST_CAL_NEXT_TAP;
                    end else if (cycle_cnt >= TIMEOUT_CYCLES) begin
                        // Timeout — store best guess and continue
                        tap_vals[tap_idx] <= curr_dmon_val[4:0];
                        state_reg <= ST_CAL_NEXT_TAP;
                    end else begin
                        cycle_cnt <= 32'd0;
                        state_reg <= ST_CAL_READ_DMON;
                    end
                end
            end

            // Advance to next tap or freeze
            ST_CAL_NEXT_TAP: begin
                if (tap_idx == NUM_TAPS - 1) begin
                    state_reg <= ST_CAL_FREEZE;
                end else begin
                    tap_idx   <= tap_idx + 1;
                    cycle_cnt <= 32'd0;
                    state_reg <= ST_CAL_SEL_TAP;
                end
            end

            // Freeze all taps: set {HOLD=1, OVRDEN=0}
            ST_CAL_FREEZE: begin
                for (i = 0; i < NUM_TAPS; i = i + 1) begin
                    tap_hold_ovrden[i*2 +: 2] <= 2'b10;  // {HOLD=1, OVRDEN=0}
                end
                tap_idx <= 4'd0;
                state_reg <= ST_CAL_MINIMIZE;
            end

            // Minimize: zero out taps with small values
            ST_CAL_MINIMIZE: begin
                active_tap_count <= 4'd0;
                tap_active_mask  <= 15'd0;
                for (i = 0; i < NUM_TAPS; i = i + 1) begin
                    if (tap_vals[i] < TAP_ZERO_THRESHOLD) begin
                        tap_vals[i] <= 5'd0;
                        // Override to zero: {HOLD=x, OVRDEN=1}
                        tap_hold_ovrden[i*2 +: 2] <= 2'b11;
                    end else begin
                        active_tap_count <= active_tap_count + 1;
                        tap_active_mask[i] <= 1'b1;
                    end
                end
                state_reg <= ST_CAL_DONE;
            end

            ST_CAL_DONE: begin
                // Copy tap values to output registers
                tap_val_0  <= tap_vals[0];  tap_val_1  <= tap_vals[1];
                tap_val_2  <= tap_vals[2];  tap_val_3  <= tap_vals[3];
                tap_val_4  <= tap_vals[4];  tap_val_5  <= tap_vals[5];
                tap_val_6  <= tap_vals[6];  tap_val_7  <= tap_vals[7];
                tap_val_8  <= tap_vals[8];  tap_val_9  <= tap_vals[9];
                tap_val_10 <= tap_vals[10]; tap_val_11 <= tap_vals[11];
                tap_val_12 <= tap_vals[12]; tap_val_13 <= tap_vals[13];
                tap_val_14 <= tap_vals[14];
                cal_done  <= 1'b1;
                state_reg <= ST_IDLE;
            end

            // ---- OVERRIDE MODE ----

            // Read-modify-write: read current register value
            ST_OVR_READ_REG: begin
                drp_addr  <= tap_drp_addr[tap_idx];
                drp_valid <= 1'b1;
                drp_wr    <= 1'b0;
                if (drp_done) begin
                    drp_rmw_data <= drp_rdata;
                    state_reg <= ST_OVR_WRITE_REG;
                end
            end

            // Write override value into bits [15:11]
            ST_OVR_WRITE_REG: begin
                drp_addr  <= tap_drp_addr[tap_idx];
                drp_wdata <= {override_vals[tap_idx], drp_rmw_data[10:0]};
                drp_valid <= 1'b1;
                drp_wr    <= 1'b1;
                if (drp_done) begin
                    if (tap_idx == NUM_TAPS - 1) begin
                        state_reg <= ST_OVR_SET_OVRDEN;
                    end else begin
                        tap_idx <= tap_idx + 1;
                        state_reg <= ST_OVR_READ_REG;
                    end
                end
            end

            // Set all taps to override mode: {HOLD=x, OVRDEN=1}
            ST_OVR_SET_OVRDEN: begin
                for (i = 0; i < NUM_TAPS; i = i + 1) begin
                    tap_hold_ovrden[i*2 +: 2] <= 2'b11;
                end
                // Copy override values to tap readback
                tap_val_0  <= override_vals[0];  tap_val_1  <= override_vals[1];
                tap_val_2  <= override_vals[2];  tap_val_3  <= override_vals[3];
                tap_val_4  <= override_vals[4];  tap_val_5  <= override_vals[5];
                tap_val_6  <= override_vals[6];  tap_val_7  <= override_vals[7];
                tap_val_8  <= override_vals[8];  tap_val_9  <= override_vals[9];
                tap_val_10 <= override_vals[10]; tap_val_11 <= override_vals[11];
                tap_val_12 <= override_vals[12]; tap_val_13 <= override_vals[13];
                tap_val_14 <= override_vals[14];
                state_reg <= ST_OVR_DONE;
            end

            ST_OVR_DONE: begin
                cal_done  <= 1'b1;
                state_reg <= ST_IDLE;
            end

            ST_ERROR: begin
                cal_error <= 1'b1;
                cal_done  <= 1'b1;
                state_reg <= ST_IDLE;
            end

            default: state_reg <= ST_IDLE;
        endcase
    end
end

endmodule

`resetall
