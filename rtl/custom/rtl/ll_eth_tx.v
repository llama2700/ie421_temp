/*
 * Ultra-low-latency fused TX MAC+PCS.
 *
 * 2-stage pipeline:
 *   Cycle 0: AXI-S input registered, state machine determines output type
 *   Cycle 1: Encode to 64b/66b block type + scramble, register output
 * 
 */

`resetall
`timescale 1ns / 1ps
`default_nettype none

module ll_eth_tx #(
    parameter DATA_WIDTH       = 64,
    parameter HDR_WIDTH        = 2,
    parameter SCRAMBLER_DISABLE = 0,
    parameter MIN_FRAME_LENGTH  = 64
)(
    input  wire                   clk,
    input  wire                   rst,

    // AXI-Stream input
    input  wire [DATA_WIDTH-1:0]  s_axis_tdata,
    input  wire [7:0]             s_axis_tkeep,
    input  wire                   s_axis_tvalid,
    output wire                   s_axis_tready,
    input  wire                   s_axis_tlast,
    input  wire                   s_axis_tuser,

    // SerDes output (to GTF raw)
    output wire [DATA_WIDTH-1:0]  serdes_tx_data,
    output wire [HDR_WIDTH-1:0]   serdes_tx_hdr,

    // Configuration
    input  wire [7:0]             cfg_ifg,

    // Status
    output wire                   tx_start_packet,
    output wire                   tx_error_underflow
);

localparam KEEP_WIDTH    = DATA_WIDTH / 8;
localparam EMPTY_WIDTH   = $clog2(KEEP_WIDTH);
localparam MIN_LEN_WIDTH = $clog2(MIN_FRAME_LENGTH-4-KEEP_WIDTH+1);

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
// Constants
// ---------------------------------------------------------------
localparam [7:0]
    ETH_PRE = 8'h55,
    ETH_SFD = 8'hD5;

localparam [6:0]
    CTRL_IDLE  = 7'h00,
    CTRL_ERROR = 7'h1e;

localparam [1:0]
    SYNC_DATA = 2'b10,
    SYNC_CTRL = 2'b01;

localparam [7:0]
    BLOCK_TYPE_CTRL    = 8'h1e,
    BLOCK_TYPE_START_4 = 8'h33,
    BLOCK_TYPE_START_0 = 8'h78,
    BLOCK_TYPE_TERM_0  = 8'h87,
    BLOCK_TYPE_TERM_1  = 8'h99,
    BLOCK_TYPE_TERM_2  = 8'haa,
    BLOCK_TYPE_TERM_3  = 8'hb4,
    BLOCK_TYPE_TERM_4  = 8'hcc,
    BLOCK_TYPE_TERM_5  = 8'hd2,
    BLOCK_TYPE_TERM_6  = 8'he1,
    BLOCK_TYPE_TERM_7  = 8'hff;

localparam [3:0]
    OUTPUT_TYPE_IDLE    = 4'd0,
    OUTPUT_TYPE_ERROR   = 4'd1,
    OUTPUT_TYPE_START_0 = 4'd2,
    OUTPUT_TYPE_START_4 = 4'd3,
    OUTPUT_TYPE_DATA    = 4'd4,
    OUTPUT_TYPE_TERM_0  = 4'd8,
    OUTPUT_TYPE_TERM_1  = 4'd9,
    OUTPUT_TYPE_TERM_2  = 4'd10,
    OUTPUT_TYPE_TERM_3  = 4'd11,
    OUTPUT_TYPE_TERM_4  = 4'd12,
    OUTPUT_TYPE_TERM_5  = 4'd13,
    OUTPUT_TYPE_TERM_6  = 4'd14,
    OUTPUT_TYPE_TERM_7  = 4'd15;

// IDLE -> PREAMBLE -> PAYLOAD -> PAD -> FCS_1 -> FCS_2 -> IFG -> IDLE
localparam [2:0]
    STATE_IDLE     = 3'd0,
    STATE_PAYLOAD  = 3'd1,
    STATE_PAD      = 3'd2,
    STATE_FCS_1    = 3'd3,
    STATE_FCS_2    = 3'd4,
    STATE_IFG      = 3'd5,
    STATE_WAIT_END = 3'd6;

// ---------------------------------------------------------------
// Registers
// ---------------------------------------------------------------
reg [2:0] state_reg = STATE_IDLE, state_next;

reg reset_crc;
reg update_crc;

reg swap_lanes_reg = 1'b0, swap_lanes_next;
reg [31:0] swap_data = 32'd0;

reg delay_type_valid = 1'b0;
reg [3:0] delay_type = OUTPUT_TYPE_IDLE;

reg [DATA_WIDTH-1:0] s_axis_tdata_masked;

reg [DATA_WIDTH-1:0] s_tdata_reg = 0, s_tdata_next;
reg [EMPTY_WIDTH-1:0] s_empty_reg = 0, s_empty_next;

reg [DATA_WIDTH-1:0] fcs_output_data_0;
reg [DATA_WIDTH-1:0] fcs_output_data_1;
reg [3:0] fcs_output_type_0;
reg [3:0] fcs_output_type_1;

reg [7:0] ifg_offset;

reg [MIN_LEN_WIDTH-1:0] frame_min_count_reg = 0, frame_min_count_next;

reg [7:0] ifg_count_reg = 8'd0, ifg_count_next;
reg [1:0] deficit_idle_count_reg = 2'd0, deficit_idle_count_next;

reg s_axis_tready_reg = 1'b0, s_axis_tready_next;

reg [31:0] crc_state = 32'hFFFFFFFF;
wire [31:0] crc_next [7:0];

// Pre-encode output
reg [DATA_WIDTH-1:0] output_data_reg = {DATA_WIDTH{1'b0}}, output_data_next;
reg [3:0] output_type_reg = OUTPUT_TYPE_IDLE, output_type_next;

// Post-encode (pre-scramble)
reg [DATA_WIDTH-1:0] encoded_data;
reg [HDR_WIDTH-1:0]  encoded_hdr;

// Scrambled output registers
reg [DATA_WIDTH-1:0] serdes_tx_data_reg = {{8{CTRL_IDLE}}, BLOCK_TYPE_CTRL};
reg [HDR_WIDTH-1:0]  serdes_tx_hdr_reg  = SYNC_CTRL;

// Scrambler state
reg [57:0] scrambler_state = {58{1'b1}};

// Scrambler wires
wire [DATA_WIDTH-1:0] scrambled_data;
wire [57:0]           scrambler_state_out;

reg tx_start_packet_reg = 1'b0, tx_start_packet_next;
reg tx_error_underflow_reg = 1'b0, tx_error_underflow_next;

assign s_axis_tready     = s_axis_tready_reg;
assign serdes_tx_data    = serdes_tx_data_reg;
assign serdes_tx_hdr     = serdes_tx_hdr_reg;
assign tx_start_packet   = tx_start_packet_reg;
assign tx_error_underflow = tx_error_underflow_reg;

// ---------------------------------------------------------------
// CRC-32 instances (8 parallel, 1-8 bytes)
// ---------------------------------------------------------------
generate
    genvar n;
    for (n = 0; n < 8; n = n + 1) begin : crc
        lfsr #(
            .LFSR_WIDTH(32),
            .LFSR_POLY(32'h4c11db7),
            .LFSR_CONFIG("GALOIS"),
            .LFSR_FEED_FORWARD(0),
            .REVERSE(1),
            .DATA_WIDTH(8*(n+1)),
            .STYLE("AUTO")
        ) eth_crc (
            .data_in(s_tdata_reg[0 +: 8*(n+1)]),
            .state_in(crc_state),
            .data_out(),
            .state_out(crc_next[n])
        );
    end
endgenerate

// ---------------------------------------------------------------
// Scrambler (combinational, operates on encoded output)
// ---------------------------------------------------------------
generate
    if (!SCRAMBLER_DISABLE) begin : gen_scrambler
        lfsr #(
            .LFSR_WIDTH(58),
            .LFSR_POLY(58'h8000000001),
            .LFSR_CONFIG("FIBONACCI"),
            .LFSR_FEED_FORWARD(0),
            .REVERSE(1),
            .DATA_WIDTH(DATA_WIDTH),
            .STYLE("AUTO")
        ) scrambler_inst (
            .data_in(encoded_data),
            .state_in(scrambler_state),
            .data_out(scrambled_data),
            .state_out(scrambler_state_out)
        );
    end else begin : gen_no_scrambler
        assign scrambled_data      = encoded_data;
        assign scrambler_state_out = scrambler_state;
    end
endgenerate

// ---------------------------------------------------------------
// Helper: tkeep to empty bytes
// ---------------------------------------------------------------
function [2:0] keep2empty;
    input [7:0] k;
    casez (k)
        8'bzzzzzzz0: keep2empty = 3'd7;
        8'bzzzzzz01: keep2empty = 3'd7;
        8'bzzzzz011: keep2empty = 3'd6;
        8'bzzzz0111: keep2empty = 3'd5;
        8'bzzz01111: keep2empty = 3'd4;
        8'bzz011111: keep2empty = 3'd3;
        8'bz0111111: keep2empty = 3'd2;
        8'b01111111: keep2empty = 3'd1;
        8'b11111111: keep2empty = 3'd0;
    endcase
endfunction

// ---------------------------------------------------------------
// Mask input data by tkeep
// ---------------------------------------------------------------
integer j;
always @* begin
    for (j = 0; j < 8; j = j + 1) begin
        s_axis_tdata_masked[j*8 +: 8] = s_axis_tkeep[j] ? s_axis_tdata[j*8 +: 8] : 8'd0;
    end
end

// ---------------------------------------------------------------
// FCS cycle calculation
// ---------------------------------------------------------------
always @* begin
    casez (s_empty_reg)
        3'd7: begin
            fcs_output_data_0 = {24'd0, ~crc_next[0][31:0], s_tdata_reg[7:0]};
            fcs_output_data_1 = 64'd0;
            fcs_output_type_0 = OUTPUT_TYPE_TERM_5;
            fcs_output_type_1 = OUTPUT_TYPE_IDLE;
            ifg_offset = 8'd3;
        end
        3'd6: begin
            fcs_output_data_0 = {16'd0, ~crc_next[1][31:0], s_tdata_reg[15:0]};
            fcs_output_data_1 = 64'd0;
            fcs_output_type_0 = OUTPUT_TYPE_TERM_6;
            fcs_output_type_1 = OUTPUT_TYPE_IDLE;
            ifg_offset = 8'd2;
        end
        3'd5: begin
            fcs_output_data_0 = {8'd0, ~crc_next[2][31:0], s_tdata_reg[23:0]};
            fcs_output_data_1 = 64'd0;
            fcs_output_type_0 = OUTPUT_TYPE_TERM_7;
            fcs_output_type_1 = OUTPUT_TYPE_IDLE;
            ifg_offset = 8'd1;
        end
        3'd4: begin
            fcs_output_data_0 = {~crc_next[3][31:0], s_tdata_reg[31:0]};
            fcs_output_data_1 = 64'd0;
            fcs_output_type_0 = OUTPUT_TYPE_DATA;
            fcs_output_type_1 = OUTPUT_TYPE_TERM_0;
            ifg_offset = 8'd8;
        end
        3'd3: begin
            fcs_output_data_0 = {~crc_next[4][23:0], s_tdata_reg[39:0]};
            fcs_output_data_1 = {56'd0, ~crc_next[4][31:24]};
            fcs_output_type_0 = OUTPUT_TYPE_DATA;
            fcs_output_type_1 = OUTPUT_TYPE_TERM_1;
            ifg_offset = 8'd7;
        end
        3'd2: begin
            fcs_output_data_0 = {~crc_next[5][15:0], s_tdata_reg[47:0]};
            fcs_output_data_1 = {48'd0, ~crc_next[5][31:16]};
            fcs_output_type_0 = OUTPUT_TYPE_DATA;
            fcs_output_type_1 = OUTPUT_TYPE_TERM_2;
            ifg_offset = 8'd6;
        end
        3'd1: begin
            fcs_output_data_0 = {~crc_next[6][7:0], s_tdata_reg[55:0]};
            fcs_output_data_1 = {40'd0, ~crc_next[6][31:8]};
            fcs_output_type_0 = OUTPUT_TYPE_DATA;
            fcs_output_type_1 = OUTPUT_TYPE_TERM_3;
            ifg_offset = 8'd5;
        end
        3'd0: begin
            fcs_output_data_0 = s_tdata_reg;
            fcs_output_data_1 = {32'd0, ~crc_next[7][31:0]};
            fcs_output_type_0 = OUTPUT_TYPE_DATA;
            fcs_output_type_1 = OUTPUT_TYPE_TERM_4;
            ifg_offset = 8'd4;
        end
    endcase
end

// ---------------------------------------------------------------
// State machine (combinational)
// ---------------------------------------------------------------
always @* begin
    state_next = STATE_IDLE;

    reset_crc  = 1'b0;
    update_crc = 1'b0;

    swap_lanes_next = swap_lanes_reg;

    frame_min_count_next    = frame_min_count_reg;
    ifg_count_next          = ifg_count_reg;
    deficit_idle_count_next = deficit_idle_count_reg;

    s_axis_tready_next = 1'b0;

    s_tdata_next = s_tdata_reg;
    s_empty_next = s_empty_reg;

    output_data_next = s_tdata_reg;
    output_type_next = OUTPUT_TYPE_IDLE;

    tx_start_packet_next   = 1'b0;
    tx_error_underflow_next = 1'b0;

    case (state_reg)
        STATE_IDLE: begin
            frame_min_count_next = MIN_FRAME_LENGTH - 4 - KEEP_WIDTH;
            reset_crc = 1'b1;
            s_axis_tready_next = 1'b1;

            output_data_next = s_tdata_reg;
            output_type_next = OUTPUT_TYPE_IDLE;

            s_tdata_next = s_axis_tdata_masked;
            s_empty_next = keep2empty(s_axis_tkeep);

            if (s_axis_tvalid) begin
                if (swap_lanes_reg) begin
                    tx_start_packet_next = 1'b1;
                end else begin
                    tx_start_packet_next = 1'b1;
                end
                output_data_next = {ETH_SFD, {7{ETH_PRE}}};
                output_type_next = OUTPUT_TYPE_START_0;
                s_axis_tready_next = 1'b1;
                state_next = STATE_PAYLOAD;
            end else begin
                swap_lanes_next         = 1'b0;
                ifg_count_next          = 8'd0;
                deficit_idle_count_next = 2'd0;
                state_next = STATE_IDLE;
            end
        end
        STATE_PAYLOAD: begin
            update_crc = 1'b1;
            s_axis_tready_next = 1'b1;

            if (frame_min_count_reg > KEEP_WIDTH) begin
                frame_min_count_next = frame_min_count_reg - KEEP_WIDTH;
            end else begin
                frame_min_count_next = 0;
            end

            output_data_next = s_tdata_reg;
            output_type_next = OUTPUT_TYPE_DATA;

            s_tdata_next = s_axis_tdata_masked;
            s_empty_next = keep2empty(s_axis_tkeep);

            if (s_axis_tvalid) begin
                if (s_axis_tlast) begin
                    s_axis_tready_next = 1'b0;
                    if (s_axis_tuser) begin
                        output_type_next = OUTPUT_TYPE_ERROR;
                        ifg_count_next   = 8'd8;
                        state_next       = STATE_IFG;
                    end else begin
                        if (frame_min_count_reg) begin
                            if (frame_min_count_reg > KEEP_WIDTH) begin
                                s_empty_next = 0;
                                state_next   = STATE_PAD;
                            end else begin
                                if (keep2empty(s_axis_tkeep) > KEEP_WIDTH - frame_min_count_reg) begin
                                    s_empty_next = KEEP_WIDTH - frame_min_count_reg;
                                end
                                state_next = STATE_FCS_1;
                            end
                        end else begin
                            state_next = STATE_FCS_1;
                        end
                    end
                end else begin
                    state_next = STATE_PAYLOAD;
                end
            end else begin
                // tvalid deassert: fail frame
                output_type_next        = OUTPUT_TYPE_ERROR;
                ifg_count_next          = 8'd8;
                tx_error_underflow_next = 1'b1;
                state_next              = STATE_WAIT_END;
            end
        end
        STATE_PAD: begin
            s_axis_tready_next = 1'b0;

            output_data_next = s_tdata_reg;
            output_type_next = OUTPUT_TYPE_DATA;

            s_tdata_next = 64'd0;
            s_empty_next = 0;
            update_crc   = 1'b1;

            if (frame_min_count_reg > KEEP_WIDTH) begin
                frame_min_count_next = frame_min_count_reg - KEEP_WIDTH;
                state_next           = STATE_PAD;
            end else begin
                frame_min_count_next = 0;
                s_empty_next         = KEEP_WIDTH - frame_min_count_reg;
                state_next           = STATE_FCS_1;
            end
        end
        STATE_FCS_1: begin
            s_axis_tready_next = 1'b0;

            output_data_next = fcs_output_data_0;
            output_type_next = fcs_output_type_0;

            ifg_count_next = (cfg_ifg > 8'd12 ? cfg_ifg : 8'd12) - ifg_offset
                             + (swap_lanes_reg ? 8'd4 : 8'd0) + deficit_idle_count_reg;

            if (s_empty_reg <= 4) begin
                state_next = STATE_FCS_2;
            end else begin
                state_next = STATE_IFG;
            end
        end
        STATE_FCS_2: begin
            s_axis_tready_next = 1'b0;

            output_data_next = fcs_output_data_1;
            output_type_next = fcs_output_type_1;

            reset_crc = 1'b1;

            if (ifg_count_next > 8'd7) begin
                state_next = STATE_IFG;
            end else begin
                if (ifg_count_next >= 8'd4) begin
                    deficit_idle_count_next = ifg_count_next - 8'd4;
                    swap_lanes_next = 1'b1;
                end else begin
                    deficit_idle_count_next = ifg_count_next;
                    ifg_count_next  = 8'd0;
                    swap_lanes_next = 1'b0;
                end
                s_axis_tready_next = 1'b1;
                state_next = STATE_IDLE;
            end
        end
        STATE_IFG: begin
            if (ifg_count_reg > 8'd8) begin
                ifg_count_next = ifg_count_reg - 8'd8;
            end else begin
                ifg_count_next = 8'd0;
            end

            reset_crc = 1'b1;

            if (ifg_count_next > 8'd7) begin
                state_next = STATE_IFG;
            end else begin
                if (ifg_count_next >= 8'd4) begin
                    deficit_idle_count_next = ifg_count_next - 8'd4;
                    swap_lanes_next = 1'b1;
                end else begin
                    deficit_idle_count_next = ifg_count_next;
                    ifg_count_next  = 8'd0;
                    swap_lanes_next = 1'b0;
                end
                s_axis_tready_next = 1'b1;
                state_next = STATE_IDLE;
            end
        end
        STATE_WAIT_END: begin
            s_axis_tready_next = 1'b1;

            if (ifg_count_reg > 8'd4) begin
                ifg_count_next = ifg_count_reg - 8'd4;
            end else begin
                ifg_count_next = 8'd0;
            end

            reset_crc = 1'b1;

            if (s_axis_tvalid) begin
                if (s_axis_tlast) begin
                    s_axis_tready_next = 1'b0;
                    if (ifg_count_next > 8'd7) begin
                        state_next = STATE_IFG;
                    end else begin
                        if (ifg_count_next >= 8'd4) begin
                            deficit_idle_count_next = ifg_count_next - 8'd4;
                            swap_lanes_next = 1'b1;
                        end else begin
                            deficit_idle_count_next = ifg_count_next;
                            ifg_count_next  = 8'd0;
                            swap_lanes_next = 1'b0;
                        end
                        s_axis_tready_next = 1'b1;
                        state_next = STATE_IDLE;
                    end
                end else begin
                    state_next = STATE_WAIT_END;
                end
            end else begin
                state_next = STATE_WAIT_END;
            end
        end
    endcase
end

// ---------------------------------------------------------------
// Encode output_type_reg -> 64b/66b block (combinational)
// ---------------------------------------------------------------
always @* begin
    case (output_type_reg)
        OUTPUT_TYPE_IDLE: begin
            encoded_data = {{8{CTRL_IDLE}}, BLOCK_TYPE_CTRL};
            encoded_hdr  = SYNC_CTRL;
        end
        OUTPUT_TYPE_ERROR: begin
            encoded_data = {{8{CTRL_ERROR}}, BLOCK_TYPE_CTRL};
            encoded_hdr  = SYNC_CTRL;
        end
        OUTPUT_TYPE_START_0: begin
            encoded_data = {output_data_reg[63:8], BLOCK_TYPE_START_0};
            encoded_hdr  = SYNC_CTRL;
        end
        OUTPUT_TYPE_START_4: begin
            encoded_data = {output_data_reg[63:40], 4'd0, {4{CTRL_IDLE}}, BLOCK_TYPE_START_4};
            encoded_hdr  = SYNC_CTRL;
        end
        OUTPUT_TYPE_DATA: begin
            encoded_data = output_data_reg;
            encoded_hdr  = SYNC_DATA;
        end
        OUTPUT_TYPE_TERM_0: begin
            encoded_data = {{7{CTRL_IDLE}}, 7'd0, BLOCK_TYPE_TERM_0};
            encoded_hdr  = SYNC_CTRL;
        end
        OUTPUT_TYPE_TERM_1: begin
            encoded_data = {{6{CTRL_IDLE}}, 6'd0, output_data_reg[7:0], BLOCK_TYPE_TERM_1};
            encoded_hdr  = SYNC_CTRL;
        end
        OUTPUT_TYPE_TERM_2: begin
            encoded_data = {{5{CTRL_IDLE}}, 5'd0, output_data_reg[15:0], BLOCK_TYPE_TERM_2};
            encoded_hdr  = SYNC_CTRL;
        end
        OUTPUT_TYPE_TERM_3: begin
            encoded_data = {{4{CTRL_IDLE}}, 4'd0, output_data_reg[23:0], BLOCK_TYPE_TERM_3};
            encoded_hdr  = SYNC_CTRL;
        end
        OUTPUT_TYPE_TERM_4: begin
            encoded_data = {{3{CTRL_IDLE}}, 3'd0, output_data_reg[31:0], BLOCK_TYPE_TERM_4};
            encoded_hdr  = SYNC_CTRL;
        end
        OUTPUT_TYPE_TERM_5: begin
            encoded_data = {{2{CTRL_IDLE}}, 2'd0, output_data_reg[39:0], BLOCK_TYPE_TERM_5};
            encoded_hdr  = SYNC_CTRL;
        end
        OUTPUT_TYPE_TERM_6: begin
            encoded_data = {{1{CTRL_IDLE}}, 1'd0, output_data_reg[47:0], BLOCK_TYPE_TERM_6};
            encoded_hdr  = SYNC_CTRL;
        end
        OUTPUT_TYPE_TERM_7: begin
            encoded_data = {output_data_reg[55:0], BLOCK_TYPE_TERM_7};
            encoded_hdr  = SYNC_CTRL;
        end
        default: begin
            encoded_data = {{8{CTRL_ERROR}}, BLOCK_TYPE_CTRL};
            encoded_hdr  = SYNC_CTRL;
        end
    endcase
end

// ---------------------------------------------------------------
// Clocked logic
// ---------------------------------------------------------------
always @(posedge clk) begin
    state_reg <= state_next;

    swap_lanes_reg          <= swap_lanes_next;
    frame_min_count_reg     <= frame_min_count_next;
    ifg_count_reg           <= ifg_count_next;
    deficit_idle_count_reg  <= deficit_idle_count_next;

    s_tdata_reg       <= s_tdata_next;
    s_empty_reg       <= s_empty_next;
    s_axis_tready_reg <= s_axis_tready_next;

    tx_start_packet_reg    <= tx_start_packet_next;
    tx_error_underflow_reg <= tx_error_underflow_next;

    delay_type_valid <= 1'b0;
    delay_type       <= output_type_next ^ 4'd4;

    swap_data <= output_data_next[63:32];

    if (swap_lanes_reg) begin
        output_data_reg <= {output_data_next[31:0], swap_data};
        if (delay_type_valid) begin
            output_type_reg <= delay_type;
        end else if (output_type_next == OUTPUT_TYPE_START_0) begin
            output_type_reg <= OUTPUT_TYPE_START_4;
        end else if (output_type_next[3]) begin
            if (output_type_next[2]) begin
                delay_type_valid <= 1'b1;
                output_type_reg  <= OUTPUT_TYPE_DATA;
            end else begin
                output_type_reg <= output_type_next ^ 4'd4;
            end
        end else begin
            output_type_reg <= output_type_next;
        end
    end else begin
        output_data_reg <= output_data_next;
        output_type_reg <= output_type_next;
    end

    // Fused encode + scramble: register final output
    serdes_tx_data_reg <= scrambled_data;
    serdes_tx_hdr_reg  <= encoded_hdr;

    // Update scrambler state
    if (!SCRAMBLER_DISABLE) begin
        scrambler_state <= scrambler_state_out;
    end

    // CRC update
    if (reset_crc) begin
        crc_state <= 32'hFFFFFFFF;
    end else if (update_crc) begin
        crc_state <= crc_next[7];
    end

    // Reset
    if (rst) begin
        state_reg <= STATE_IDLE;

        swap_lanes_reg         <= 1'b0;
        ifg_count_reg          <= 8'd0;
        deficit_idle_count_reg <= 2'd0;
        s_axis_tready_reg      <= 1'b0;

        serdes_tx_data_reg <= {{8{CTRL_IDLE}}, BLOCK_TYPE_CTRL};
        serdes_tx_hdr_reg  <= SYNC_CTRL;

        output_data_reg <= {DATA_WIDTH{1'b0}};
        output_type_reg <= OUTPUT_TYPE_IDLE;

        scrambler_state <= {58{1'b1}};

        tx_start_packet_reg    <= 1'b0;
        tx_error_underflow_reg <= 1'b0;

        delay_type_valid <= 1'b0;
        delay_type       <= OUTPUT_TYPE_IDLE;
    end
end

endmodule

`resetall
