/*

GTF Eye Scan Engine
Drives the GTF's built-in 2D eye scan hardware to measure eye height, width,
and BER. Supports full 2D scans and quick targeted scans for LPM/DFE
decision-making.

Eye scan results are stored in BRAM for host readout.

Per UG1549 RX Margin Analysis section, the eye scan is controlled through
DRP registers (ES_CONTROL, ES_HORZ_OFFSET, ES_VERT_OFFSET, etc.).

*/

`resetall
`timescale 1ns / 1ps
`default_nettype none

module gtf_eye_scan_engine #(
    parameter DRP_ADDR_WIDTH = 16,
    parameter DRP_DATA_WIDTH = 16,
    parameter BRAM_ADDR_WIDTH = 13,
    parameter BRAM_DATA_WIDTH = 32,
    parameter DEFAULT_PRESCALE = 5,

    // DRP addresses for eye scan registers (verify against GTF Wizard output)
    parameter ES_CONTROL_ADDR      = 16'h0090,
    parameter ES_HORZ_OFFSET_ADDR  = 16'h0091,
    parameter ES_VERT_OFFSET_ADDR  = 16'h0092,
    parameter ES_PRESCALE_ADDR     = 16'h0093,
    parameter ES_ERROR_COUNT_ADDR  = 16'h0094,
    parameter ES_SAMPLE_COUNT_ADDR = 16'h0095,
    parameter ES_STATUS_ADDR       = 16'h0096,

    // Scan range defaults
    parameter DEFAULT_H_MIN = -32,
    parameter DEFAULT_H_MAX = 32,
    parameter DEFAULT_V_MIN = -64,
    parameter DEFAULT_V_MAX = 64,
    parameter DEFAULT_H_STEP = 1,
    parameter DEFAULT_V_STEP = 2,

    // Quick scan parameters
    parameter QUICK_H_RANGE = 4,
    parameter QUICK_V_RANGE = 4,
    parameter QUICK_H_STEP  = 2,
    parameter QUICK_V_STEP  = 2
)
(
    input  wire        clk,
    input  wire        rst,

    /*
     * Control
     */
    input  wire        scan_start,
    input  wire        quick_scan_start,
    input  wire [4:0]  prescale,
    input  wire signed [15:0] h_min,
    input  wire signed [15:0] h_max,
    input  wire signed [15:0] v_min,
    input  wire signed [15:0] v_max,

    /*
     * Status
     */
    output reg         scan_done,
    output reg         scan_busy,
    output reg  [15:0] eye_height,
    output reg  [15:0] eye_width,
    output reg  [31:0] eye_ber,      // fixed-point log2 BER (higher = worse)
    output reg  [BRAM_ADDR_WIDTH-1:0] scan_point_count,

    /*
     * BRAM read port (host readout of scan data)
     * Each entry: {v_offset[15:0], h_offset[15:0]} at even addresses
     *             {error_count[15:0], sample_count[15:0]} at odd addresses
     */
    input  wire [BRAM_ADDR_WIDTH-1:0]  bram_rd_addr,
    output wire [BRAM_DATA_WIDTH-1:0]  bram_rd_data,

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
    output reg         eyescanreset,
    input  wire        eyescandataerror
);

// BRAM for scan results
reg [BRAM_DATA_WIDTH-1:0] scan_bram [0:(1<<BRAM_ADDR_WIDTH)-1];
reg [BRAM_ADDR_WIDTH-1:0] bram_wr_addr;
reg [BRAM_DATA_WIDTH-1:0] bram_wr_data;
reg                        bram_wr_en;

// BRAM read port
reg [BRAM_DATA_WIDTH-1:0] bram_rd_data_reg;
assign bram_rd_data = bram_rd_data_reg;

always @(posedge clk) begin
    bram_rd_data_reg <= scan_bram[bram_rd_addr];
    if (bram_wr_en)
        scan_bram[bram_wr_addr] <= bram_wr_data;
end

// State encoding
localparam [4:0]
    ST_IDLE           = 5'd0,
    ST_RESET          = 5'd1,
    ST_RESET_WAIT     = 5'd2,
    ST_SET_PRESCALE   = 5'd3,
    ST_SET_H_OFFSET   = 5'd4,
    ST_SET_V_OFFSET   = 5'd5,
    ST_START_MEAS     = 5'd6,
    ST_WAIT_MEAS      = 5'd7,
    ST_POLL_STATUS    = 5'd8,
    ST_READ_ERROR     = 5'd9,
    ST_READ_SAMPLE    = 5'd10,
    ST_STORE_RESULT   = 5'd11,
    ST_NEXT_V         = 5'd12,
    ST_NEXT_H         = 5'd13,
    ST_COMPUTE_EYE    = 5'd14,
    ST_DONE           = 5'd15;

reg [4:0] state_reg;

// Scan coordinates
reg signed [15:0] curr_h, curr_v;
reg signed [15:0] scan_h_min, scan_h_max;
reg signed [15:0] scan_v_min, scan_v_max;
reg signed [15:0] scan_h_step, scan_v_step;

// Measurement results for current point
reg [15:0] meas_error_count;
reg [15:0] meas_sample_count;

// Eye boundary tracking
reg signed [15:0] eye_v_min, eye_v_max;
reg signed [15:0] eye_h_min, eye_h_max;
reg        eye_v_found, eye_h_found;

// Reset counter
reg [4:0] reset_cnt;

// BER threshold for eye boundary (fixed-point: errors/samples < 1e-6 means open)
localparam BER_THRESHOLD = 16'd1;  // raw error count threshold

always @(posedge clk) begin
    if (rst) begin
        state_reg       <= ST_IDLE;
        scan_done       <= 1'b0;
        scan_busy       <= 1'b0;
        eye_height      <= 16'd0;
        eye_width       <= 16'd0;
        eye_ber         <= 32'd0;
        scan_point_count <= {BRAM_ADDR_WIDTH{1'b0}};
        eyescanreset    <= 1'b0;
        drp_valid       <= 1'b0;
        drp_wr          <= 1'b0;
        bram_wr_en      <= 1'b0;
        bram_wr_addr    <= {BRAM_ADDR_WIDTH{1'b0}};
        reset_cnt       <= 5'd0;
        curr_h          <= 16'sd0;
        curr_v          <= 16'sd0;
        eye_v_min       <= 16'sd0;
        eye_v_max       <= 16'sd0;
        eye_h_min       <= 16'sd0;
        eye_h_max       <= 16'sd0;
        eye_v_found     <= 1'b0;
        eye_h_found     <= 1'b0;
    end else begin
        scan_done  <= 1'b0;
        drp_valid  <= 1'b0;
        bram_wr_en <= 1'b0;

        case (state_reg)
            ST_IDLE: begin
                scan_busy <= 1'b0;
                if (scan_start) begin
                    scan_busy    <= 1'b1;
                    scan_h_min   <= h_min;
                    scan_h_max   <= h_max;
                    scan_v_min   <= v_min;
                    scan_v_max   <= v_max;
                    scan_h_step  <= DEFAULT_H_STEP;
                    scan_v_step  <= DEFAULT_V_STEP;
                    scan_point_count <= {BRAM_ADDR_WIDTH{1'b0}};
                    bram_wr_addr <= {BRAM_ADDR_WIDTH{1'b0}};
                    eye_v_found  <= 1'b0;
                    eye_h_found  <= 1'b0;
                    eye_v_min    <= 16'sd127;
                    eye_v_max    <= -16'sd127;
                    eye_h_min    <= 16'sd127;
                    eye_h_max    <= -16'sd127;
                    reset_cnt    <= 5'd0;
                    state_reg    <= ST_RESET;
                end else if (quick_scan_start) begin
                    scan_busy    <= 1'b1;
                    scan_h_min   <= -QUICK_H_RANGE;
                    scan_h_max   <= QUICK_H_RANGE;
                    scan_v_min   <= -QUICK_V_RANGE;
                    scan_v_max   <= QUICK_V_RANGE;
                    scan_h_step  <= QUICK_H_STEP;
                    scan_v_step  <= QUICK_V_STEP;
                    scan_point_count <= {BRAM_ADDR_WIDTH{1'b0}};
                    bram_wr_addr <= {BRAM_ADDR_WIDTH{1'b0}};
                    eye_v_found  <= 1'b0;
                    eye_h_found  <= 1'b0;
                    eye_v_min    <= 16'sd127;
                    eye_v_max    <= -16'sd127;
                    eye_h_min    <= 16'sd127;
                    eye_h_max    <= -16'sd127;
                    reset_cnt    <= 5'd0;
                    state_reg    <= ST_RESET;
                end
            end

            // Assert EYESCANRESET for 16 cycles
            ST_RESET: begin
                eyescanreset <= 1'b1;
                reset_cnt <= reset_cnt + 1;
                if (reset_cnt >= 5'd16) begin
                    eyescanreset <= 1'b0;
                    state_reg <= ST_RESET_WAIT;
                    reset_cnt <= 5'd0;
                end
            end

            // Wait a few cycles after reset deassert
            ST_RESET_WAIT: begin
                reset_cnt <= reset_cnt + 1;
                if (reset_cnt >= 5'd8) begin
                    state_reg <= ST_SET_PRESCALE;
                end
            end

            // Configure prescale
            ST_SET_PRESCALE: begin
                drp_addr  <= ES_PRESCALE_ADDR;
                drp_wdata <= {11'd0, prescale};
                drp_valid <= 1'b1;
                drp_wr    <= 1'b1;
                if (drp_done) begin
                    curr_h    <= scan_h_min;
                    curr_v    <= scan_v_min;
                    state_reg <= ST_SET_H_OFFSET;
                end
            end

            // Set horizontal offset
            ST_SET_H_OFFSET: begin
                drp_addr  <= ES_HORZ_OFFSET_ADDR;
                drp_wdata <= curr_h[15:0];
                drp_valid <= 1'b1;
                drp_wr    <= 1'b1;
                if (drp_done) begin
                    state_reg <= ST_SET_V_OFFSET;
                end
            end

            // Set vertical offset
            ST_SET_V_OFFSET: begin
                drp_addr  <= ES_VERT_OFFSET_ADDR;
                drp_wdata <= curr_v[15:0];
                drp_valid <= 1'b1;
                drp_wr    <= 1'b1;
                if (drp_done) begin
                    state_reg <= ST_START_MEAS;
                end
            end

            // Start measurement
            ST_START_MEAS: begin
                drp_addr  <= ES_CONTROL_ADDR;
                drp_wdata <= 16'h0001;  // start measurement
                drp_valid <= 1'b1;
                drp_wr    <= 1'b1;
                if (drp_done) begin
                    state_reg <= ST_POLL_STATUS;
                end
            end

            // Poll ES_STATUS for measurement complete
            ST_POLL_STATUS: begin
                drp_addr  <= ES_STATUS_ADDR;
                drp_valid <= 1'b1;
                drp_wr    <= 1'b0;
                if (drp_done) begin
                    if (drp_rdata[0]) begin  // done bit
                        state_reg <= ST_READ_ERROR;
                    end
                    // else keep polling
                end
            end

            // Read error count
            ST_READ_ERROR: begin
                drp_addr  <= ES_ERROR_COUNT_ADDR;
                drp_valid <= 1'b1;
                drp_wr    <= 1'b0;
                if (drp_done) begin
                    meas_error_count <= drp_rdata;
                    state_reg <= ST_READ_SAMPLE;
                end
            end

            // Read sample count
            ST_READ_SAMPLE: begin
                drp_addr  <= ES_SAMPLE_COUNT_ADDR;
                drp_valid <= 1'b1;
                drp_wr    <= 1'b0;
                if (drp_done) begin
                    meas_sample_count <= drp_rdata;
                    state_reg <= ST_STORE_RESULT;
                end
            end

            // Store result in BRAM and update eye boundaries
            ST_STORE_RESULT: begin
                // Store coordinates
                bram_wr_addr <= scan_point_count << 1;
                bram_wr_data <= {curr_v[15:0], curr_h[15:0]};
                bram_wr_en   <= 1'b1;
                state_reg    <= ST_NEXT_V;

                // Update eye boundaries (check if this point is "open")
                if (meas_error_count <= BER_THRESHOLD) begin
                    // Eye is open at this point
                    if (curr_h == 16'sd0) begin
                        // On the vertical axis — contributes to eye height
                        if (curr_v < eye_v_min) eye_v_min <= curr_v;
                        if (curr_v > eye_v_max) eye_v_max <= curr_v;
                        eye_v_found <= 1'b1;
                    end
                    if (curr_v == 16'sd0) begin
                        // On the horizontal axis — contributes to eye width
                        if (curr_h < eye_h_min) eye_h_min <= curr_h;
                        if (curr_h > eye_h_max) eye_h_max <= curr_h;
                        eye_h_found <= 1'b1;
                    end
                end

                scan_point_count <= scan_point_count + 1;
            end

            // Store error/sample at odd address, advance V
            ST_NEXT_V: begin
                bram_wr_addr <= (scan_point_count - 1) << 1 | 1;
                bram_wr_data <= {meas_error_count, meas_sample_count};
                bram_wr_en   <= 1'b1;

                if (curr_v + scan_v_step > scan_v_max) begin
                    state_reg <= ST_NEXT_H;
                end else begin
                    curr_v    <= curr_v + scan_v_step;
                    state_reg <= ST_SET_V_OFFSET;
                end
            end

            // Advance H, reset V
            ST_NEXT_H: begin
                if (curr_h + scan_h_step > scan_h_max) begin
                    state_reg <= ST_COMPUTE_EYE;
                end else begin
                    curr_h    <= curr_h + scan_h_step;
                    curr_v    <= scan_v_min;
                    state_reg <= ST_SET_H_OFFSET;
                end
            end

            // Compute final eye metrics
            ST_COMPUTE_EYE: begin
                if (eye_v_found)
                    eye_height <= eye_v_max - eye_v_min;
                else
                    eye_height <= 16'd0;

                if (eye_h_found)
                    eye_width <= eye_h_max - eye_h_min;
                else
                    eye_width <= 16'd0;

                // Simple BER estimate: use center point error count
                eye_ber <= {16'd0, meas_error_count};

                state_reg <= ST_DONE;
            end

            ST_DONE: begin
                scan_done <= 1'b1;
                state_reg <= ST_IDLE;
            end

            default: state_reg <= ST_IDLE;
        endcase
    end
end

endmodule

`resetall
