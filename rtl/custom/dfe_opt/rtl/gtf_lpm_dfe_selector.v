/*

GTF LPM/DFE Selector
Automatically selects between LPM (CTLE-only, lower latency) and DFE
(15-tap equalizer, better for lossy channels) based on measured eye margin.

Starts in LPM mode, runs a quick eye scan, and switches to DFE only if
the margin is insufficient. Periodically rechecks to handle changing
channel conditions.

*/

`resetall
`timescale 1ns / 1ps
`default_nettype none

module gtf_lpm_dfe_selector #(
    parameter DEFAULT_THRESHOLD = 16'd100,
    parameter RECHECK_INTERVAL = 32'd500000000,  // ~775ms at 644MHz
    parameter LINK_SETTLE_CYCLES = 32'd6440000    // ~10ms at 644MHz
)
(
    input  wire        clk,
    input  wire        rst,

    /*
     * Control
     */
    input  wire        auto_enable,     // enable auto-selection
    input  wire        force_lpm,       // force LPM regardless
    input  wire        force_dfe,       // force DFE regardless
    input  wire [15:0] threshold,       // eye margin threshold

    /*
     * Status
     */
    output reg         lpm_active,
    output reg         dfe_active,
    output reg  [15:0] current_margin,
    output reg  [1:0]  selector_state,  // for debug: 0=idle, 1=lpm, 2=dfe, 3=monitor

    /*
     * GTF sideband
     */
    output reg         rxlpmen,

    /*
     * Eye scan engine interface
     */
    output reg         quick_scan_trigger,
    input  wire        scan_done,
    input  wire [15:0] eye_height,

    /*
     * Calibration FSM interface
     */
    output reg         cal_trigger,
    input  wire        cal_done
);

// State encoding
localparam [3:0]
    ST_IDLE          = 4'd0,
    ST_START_LPM     = 4'd1,
    ST_LPM_SETTLE    = 4'd2,
    ST_LPM_SCAN      = 4'd3,
    ST_CHECK_LPM     = 4'd4,
    ST_SWITCH_TO_DFE = 4'd5,
    ST_DFE_CAL       = 4'd6,
    ST_DFE_SCAN      = 4'd7,
    ST_CHECK_DFE     = 4'd8,
    ST_MONITOR       = 4'd9,
    ST_RECHECK_SCAN  = 4'd10,
    ST_RECHECK_EVAL  = 4'd11;

reg [3:0]  state_reg;
reg [31:0] settle_cnt;
reg [31:0] recheck_cnt;

always @(posedge clk) begin
    if (rst) begin
        state_reg         <= ST_IDLE;
        settle_cnt        <= 32'd0;
        recheck_cnt       <= 32'd0;
        rxlpmen           <= 1'b1;  // default to LPM
        lpm_active        <= 1'b1;
        dfe_active        <= 1'b0;
        current_margin    <= 16'd0;
        selector_state    <= 2'd0;
        quick_scan_trigger <= 1'b0;
        cal_trigger       <= 1'b0;
    end else begin
        quick_scan_trigger <= 1'b0;
        cal_trigger        <= 1'b0;

        // Force mode overrides
        if (force_lpm && !force_dfe) begin
            rxlpmen    <= 1'b1;
            lpm_active <= 1'b1;
            dfe_active <= 1'b0;
            if (!auto_enable)
                state_reg <= ST_IDLE;
        end else if (force_dfe && !force_lpm) begin
            rxlpmen    <= 1'b0;
            lpm_active <= 1'b0;
            dfe_active <= 1'b1;
            if (!auto_enable)
                state_reg <= ST_IDLE;
        end

        case (state_reg)
            ST_IDLE: begin
                selector_state <= 2'd0;
                if (auto_enable && !force_lpm && !force_dfe) begin
                    state_reg <= ST_START_LPM;
                end
            end

            // Start in LPM mode
            ST_START_LPM: begin
                selector_state <= 2'd1;
                rxlpmen    <= 1'b1;
                lpm_active <= 1'b1;
                dfe_active <= 1'b0;
                settle_cnt <= 32'd0;
                state_reg  <= ST_LPM_SETTLE;
            end

            // Wait for link to settle in LPM mode
            ST_LPM_SETTLE: begin
                settle_cnt <= settle_cnt + 1;
                if (settle_cnt >= LINK_SETTLE_CYCLES) begin
                    quick_scan_trigger <= 1'b1;
                    state_reg <= ST_LPM_SCAN;
                end
            end

            // Wait for eye scan to complete
            ST_LPM_SCAN: begin
                if (scan_done) begin
                    current_margin <= eye_height;
                    state_reg <= ST_CHECK_LPM;
                end
            end

            // Check if LPM margin is sufficient
            ST_CHECK_LPM: begin
                if (current_margin >= threshold) begin
                    // LPM is good enough — monitor
                    recheck_cnt <= 32'd0;
                    state_reg <= ST_MONITOR;
                end else begin
                    // Need DFE
                    state_reg <= ST_SWITCH_TO_DFE;
                end
            end

            // Switch to DFE mode
            ST_SWITCH_TO_DFE: begin
                selector_state <= 2'd2;
                rxlpmen    <= 1'b0;
                lpm_active <= 1'b0;
                dfe_active <= 1'b1;
                settle_cnt <= 32'd0;
                // Trigger calibration
                cal_trigger <= 1'b1;
                state_reg <= ST_DFE_CAL;
            end

            // Wait for DFE calibration to complete
            ST_DFE_CAL: begin
                if (cal_done) begin
                    // Run eye scan after calibration
                    quick_scan_trigger <= 1'b1;
                    state_reg <= ST_DFE_SCAN;
                end
            end

            // Wait for post-calibration eye scan
            ST_DFE_SCAN: begin
                if (scan_done) begin
                    current_margin <= eye_height;
                    state_reg <= ST_CHECK_DFE;
                end
            end

            // Record DFE margin, enter monitoring
            ST_CHECK_DFE: begin
                recheck_cnt <= 32'd0;
                state_reg <= ST_MONITOR;
            end

            // Periodically recheck margin
            ST_MONITOR: begin
                selector_state <= 2'd3;
                recheck_cnt <= recheck_cnt + 1;
                if (recheck_cnt >= RECHECK_INTERVAL) begin
                    quick_scan_trigger <= 1'b1;
                    state_reg <= ST_RECHECK_SCAN;
                end
                // Exit if auto disabled
                if (!auto_enable) begin
                    state_reg <= ST_IDLE;
                end
            end

            // Wait for recheck scan
            ST_RECHECK_SCAN: begin
                if (scan_done) begin
                    current_margin <= eye_height;
                    state_reg <= ST_RECHECK_EVAL;
                end
            end

            // Evaluate recheck results
            ST_RECHECK_EVAL: begin
                if (lpm_active && current_margin < threshold) begin
                    // LPM margin degraded, switch to DFE
                    state_reg <= ST_SWITCH_TO_DFE;
                end else begin
                    // Margin still OK, continue monitoring
                    recheck_cnt <= 32'd0;
                    state_reg <= ST_MONITOR;
                end
            end

            default: state_reg <= ST_IDLE;
        endcase
    end
end

endmodule

`resetall
