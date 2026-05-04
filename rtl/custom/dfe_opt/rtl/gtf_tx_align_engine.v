/*

GTF TX Alignment Engine
Aligns packet injection to 64b/66b codeword boundaries using the hard MAC's
TXAXISTCANSTART signal to minimize TX latency. In 10G mode, a new codeword
starts every ~4 TXUSRCLK cycles. Missing the boundary costs up to 4 cycles.

Supports three modes:
  - Pass-through: zero added latency, just monitors alignment stats
  - Aligned: holds packets until TXAXISTCANSTART asserts (max 4 cycle wait)
  - Predictive: tracks TXGBSEQSTART to pre-signal readiness

*/

`resetall
`timescale 1ns / 1ps
`default_nettype none

module gtf_tx_align_engine #(
    parameter DATA_WIDTH = 16,
    parameter GEARBOX_PERIOD = 33  // 10G gearbox pattern length
)
(
    input  wire                    txusrclk,
    input  wire                    txusrrst,

    /*
     * GTF hard MAC signals
     */
    input  wire                    txaxistcanstart,  // PCSRSVDOUT[2]
    input  wire                    txaxistready,
    input  wire                    txgbseqstart,     // gearbox sequence start pulse

    /*
     * AXI-Stream input (from user logic / Corundum direct interface)
     */
    input  wire [DATA_WIDTH-1:0]   s_axis_tdata,
    input  wire                    s_axis_tvalid,
    input  wire [1:0]              s_axis_tlast,
    input  wire                    s_axis_tsof,
    input  wire                    s_axis_terr,
    input  wire [7:0]              s_axis_tpre,
    output wire                    s_axis_tready,

    /*
     * AXI-Stream output (to GTF hard MAC TXAXIS interface)
     */
    output wire [DATA_WIDTH-1:0]   m_axis_tdata,
    output wire                    m_axis_tvalid,
    output wire [1:0]              m_axis_tlast,
    output wire                    m_axis_tsof,
    output wire                    m_axis_terr,
    output wire [7:0]              m_axis_tpre,
    input  wire                    m_axis_tready,

    /*
     * Control
     */
    input  wire                    force_align,     // 1 = hold until codeword boundary
    input  wire                    predictive_en,   // 1 = use TXGBSEQSTART prediction

    /*
     * Statistics (active in all modes, synchronized to txusrclk)
     */
    output reg  [31:0]             align_hit_count,
    output reg  [31:0]             align_miss_count,
    output reg  [4:0]              last_wait_cycles
);

// Gearbox sequence tracker
reg [5:0] gb_seq_count;
reg       gb_seq_valid;

always @(posedge txusrclk) begin
    if (txusrrst) begin
        gb_seq_count <= 6'd0;
        gb_seq_valid <= 1'b0;
    end else begin
        if (txgbseqstart) begin
            gb_seq_count <= 6'd0;
            gb_seq_valid <= 1'b1;
        end else if (gb_seq_valid) begin
            if (gb_seq_count == GEARBOX_PERIOD - 1)
                gb_seq_count <= 6'd0;
            else
                gb_seq_count <= gb_seq_count + 1;
        end
    end
end

// Predict next TXAXISTCANSTART using gearbox sequence
// Per UG1549: codewords start at counts 0, 4, 8, 12, 16, 20, 24, 28
// Optimal injection: one cycle before = counts 3, 7, 11, 15, 19, 23, 27, 32
wire predict_canstart;
assign predict_canstart = gb_seq_valid && (
    gb_seq_count == 6'd3  || gb_seq_count == 6'd7  ||
    gb_seq_count == 6'd11 || gb_seq_count == 6'd15 ||
    gb_seq_count == 6'd19 || gb_seq_count == 6'd23 ||
    gb_seq_count == 6'd27 || gb_seq_count == 6'd32
);

// Alignment gate: determines when we allow a new packet to start
wire can_start_now;
assign can_start_now = predictive_en ? (txaxistcanstart | predict_canstart) : txaxistcanstart;

// FSM for alignment
localparam [1:0]
    ST_IDLE     = 2'd0,
    ST_WAIT     = 2'd1,
    ST_TRANSFER = 2'd2;

reg [1:0] state_reg;
reg [4:0] wait_cnt;
reg       sof_seen;

// Internal ready signal
reg s_ready_int;

always @(posedge txusrclk) begin
    if (txusrrst) begin
        state_reg       <= ST_IDLE;
        wait_cnt        <= 5'd0;
        sof_seen        <= 1'b0;
        align_hit_count <= 32'd0;
        align_miss_count <= 32'd0;
        last_wait_cycles <= 5'd0;
        s_ready_int     <= 1'b0;
    end else begin
        case (state_reg)
            ST_IDLE: begin
                s_ready_int <= 1'b1;
                if (s_axis_tvalid && s_axis_tsof) begin
                    if (!force_align) begin
                        // Pass-through mode: always forward, just count
                        if (can_start_now)
                            align_hit_count <= align_hit_count + 1;
                        else
                            align_miss_count <= align_miss_count + 1;
                        state_reg <= ST_TRANSFER;
                        sof_seen <= 1'b1;
                    end else begin
                        // Aligned mode: check if we can start now
                        if (can_start_now) begin
                            align_hit_count <= align_hit_count + 1;
                            last_wait_cycles <= 5'd0;
                            state_reg <= ST_TRANSFER;
                            sof_seen <= 1'b1;
                        end else begin
                            // Must wait
                            s_ready_int <= 1'b0;
                            wait_cnt <= 5'd1;
                            state_reg <= ST_WAIT;
                        end
                    end
                end
            end

            ST_WAIT: begin
                s_ready_int <= 1'b0;
                if (can_start_now) begin
                    align_miss_count <= align_miss_count + 1;
                    last_wait_cycles <= wait_cnt;
                    s_ready_int <= 1'b1;
                    state_reg <= ST_TRANSFER;
                    sof_seen <= 1'b1;
                end else begin
                    wait_cnt <= wait_cnt + 1;
                end
            end

            ST_TRANSFER: begin
                s_ready_int <= 1'b1;
                sof_seen <= 1'b0;
                // Stay in transfer until end of packet
                if (s_axis_tvalid && |s_axis_tlast && s_axis_tready) begin
                    state_reg <= ST_IDLE;
                end
            end

            default: state_reg <= ST_IDLE;
        endcase
    end
end

// Output logic
// In pass-through mode (!force_align), data flows directly
// In aligned mode, we gate s_axis_tready until alignment is achieved
wire gate_ready;
assign gate_ready = force_align ? s_ready_int : 1'b1;

assign s_axis_tready = m_axis_tready & gate_ready;
assign m_axis_tdata  = s_axis_tdata;
assign m_axis_tvalid = s_axis_tvalid & gate_ready;
assign m_axis_tlast  = s_axis_tlast;
assign m_axis_tsof   = s_axis_tsof & sof_seen;
assign m_axis_terr   = s_axis_terr;
assign m_axis_tpre   = s_axis_tpre;

endmodule

`resetall
