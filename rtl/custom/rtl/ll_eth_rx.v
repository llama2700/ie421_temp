/*
 * Ultra-low-latency fused RX PCS+MAC
 *
 *   Cycle 0: Register SerDes input
 *   Cycle 1: Combinational descramble + decode + output (registered)
 *            CRC-32 update runs in PARALLEL, off the data critical path
 *
 *   - FCS not stripped (output gets full frame + 4-byte FCS)
 *   - FCS check result reported on separate rx_fcs_valid/rx_fcs_ok ports,
 *     1 cycle AFTER tlast since CRC is decoupled to take it off the path
 *   - TERM_0 emits a zero-byte tlast beat (tvalid=1, tkeep=0, tlast=1)
 *   - START_4 (lane swap) supported via speculative swap buffer (1 mux, 0 extra stages)
 *   - m_axis_tuser indicates mid-frame errors (bad block), NOT FCS errors
 */

`resetall
`timescale 1ns / 1ps
`default_nettype none

module ll_eth_rx #(
    parameter DATA_WIDTH       = 64,
    parameter HDR_WIDTH        = 2,
    parameter SCRAMBLER_DISABLE = 0
)(
    input  wire                   clk,
    input  wire                   rst,

    // SerDes input (raw from GTF)
    input  wire [DATA_WIDTH-1:0]  serdes_rx_data,
    input  wire [HDR_WIDTH-1:0]   serdes_rx_hdr,

    // AXI-Stream output
    output wire [DATA_WIDTH-1:0]  m_axis_tdata,
    output wire [7:0]             m_axis_tkeep,
    output wire                   m_axis_tvalid,
    output wire                   m_axis_tlast,
    output wire                   m_axis_tuser,   // mid-frame error (bad block)

    // Delayed FCS
    output wire                   rx_fcs_valid,
    output wire                   rx_fcs_ok,

    // Frame sync interface
    output wire                   rx_block_lock,
    output wire                   serdes_rx_bitslip,

    // Status
    output wire                   rx_bad_block,
    output wire                   rx_start_packet,
    output wire                   rx_error_bad_fcs
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
// 10GBASE-R block type constants
// ---------------------------------------------------------------
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
    INPUT_TYPE_IDLE    = 4'd0,
    INPUT_TYPE_ERROR   = 4'd1,
    INPUT_TYPE_START_0 = 4'd2,
    INPUT_TYPE_START_4 = 4'd3,
    INPUT_TYPE_DATA    = 4'd4,
    INPUT_TYPE_TERM_0  = 4'd8,
    INPUT_TYPE_TERM_1  = 4'd9,
    INPUT_TYPE_TERM_2  = 4'd10,
    INPUT_TYPE_TERM_3  = 4'd11,
    INPUT_TYPE_TERM_4  = 4'd12,
    INPUT_TYPE_TERM_5  = 4'd13,
    INPUT_TYPE_TERM_6  = 4'd14,
    INPUT_TYPE_TERM_7  = 4'd15;

localparam
    STATE_IDLE    = 1'b0,
    STATE_PAYLOAD = 1'b1;

// ---------------------------------------------------------------
// Stage 0 registers: SerDes input capture + descrambler state
// ---------------------------------------------------------------
reg [DATA_WIDTH-1:0] rx_data_reg     = {DATA_WIDTH{1'b0}};
reg [HDR_WIDTH-1:0]  rx_hdr_reg      = 2'b00;
reg [57:0]           scrambler_state = {58{1'b1}};

// ---------------------------------------------------------------
// Descrambler (combinational from registered input)
// ---------------------------------------------------------------
wire [DATA_WIDTH-1:0] descrambled_data;
wire [57:0]           descrambler_state_out;

generate
    if (!SCRAMBLER_DISABLE) begin : gen_descrambler
        lfsr #(
            .LFSR_WIDTH(58),
            .LFSR_POLY(58'h8000000001),
            .LFSR_CONFIG("FIBONACCI"),
            .LFSR_FEED_FORWARD(1),
            .REVERSE(1),
            .DATA_WIDTH(DATA_WIDTH),
            .STYLE("AUTO")
        ) descrambler_inst (
            .data_in(rx_data_reg),
            .state_in(scrambler_state),
            .data_out(descrambled_data),
            .state_out(descrambler_state_out)
        );
    end else begin : gen_no_descrambler
        assign descrambled_data      = rx_data_reg;
        assign descrambler_state_out = scrambler_state;
    end
endgenerate

// ---------------------------------------------------------------
// Lane swap buffer (speculative, zero added pipeline stages)
// Every cycle, upper 32 bits of decoded data are stored.
// When lanes_swapped=1, output uses {current[31:0], swap_buf_reg}.
// ---------------------------------------------------------------
reg [31:0] swap_buf_reg  = 32'd0;
reg        lanes_swapped = 1'b0;

// ---------------------------------------------------------------
// Block type decode (combinational)
// ---------------------------------------------------------------
wire [7:0] block_type = descrambled_data[7:0];
reg  [3:0] decoded_type;
reg        decoded_bad_block;
reg  [DATA_WIDTH-1:0] decoded_data_masked;

always @* begin
    decoded_type        = INPUT_TYPE_IDLE;
    decoded_bad_block   = 1'b0;
    decoded_data_masked = {DATA_WIDTH{1'b0}};

    if (rx_hdr_reg == SYNC_DATA) begin
        decoded_type        = INPUT_TYPE_DATA;
        decoded_data_masked = descrambled_data;
    end else if (rx_hdr_reg == SYNC_CTRL) begin
        case (block_type)
            BLOCK_TYPE_CTRL:    decoded_type = INPUT_TYPE_IDLE;
            BLOCK_TYPE_START_0: decoded_type = INPUT_TYPE_START_0;
            BLOCK_TYPE_START_4: decoded_type = INPUT_TYPE_START_4;
            BLOCK_TYPE_TERM_0:  decoded_type = INPUT_TYPE_TERM_0;
            BLOCK_TYPE_TERM_1: begin
                decoded_type = INPUT_TYPE_TERM_1;
                decoded_data_masked = {56'd0, descrambled_data[15:8]};
            end
            BLOCK_TYPE_TERM_2: begin
                decoded_type = INPUT_TYPE_TERM_2;
                decoded_data_masked = {48'd0, descrambled_data[23:8]};
            end
            BLOCK_TYPE_TERM_3: begin
                decoded_type = INPUT_TYPE_TERM_3;
                decoded_data_masked = {40'd0, descrambled_data[31:8]};
            end
            BLOCK_TYPE_TERM_4: begin
                decoded_type = INPUT_TYPE_TERM_4;
                decoded_data_masked = {32'd0, descrambled_data[39:8]};
            end
            BLOCK_TYPE_TERM_5: begin
                decoded_type = INPUT_TYPE_TERM_5;
                decoded_data_masked = {24'd0, descrambled_data[47:8]};
            end
            BLOCK_TYPE_TERM_6: begin
                decoded_type = INPUT_TYPE_TERM_6;
                decoded_data_masked = {16'd0, descrambled_data[55:8]};
            end
            BLOCK_TYPE_TERM_7: begin
                decoded_type = INPUT_TYPE_TERM_7;
                decoded_data_masked = {8'd0, descrambled_data[63:8]};
            end
            default: begin
                decoded_type    = INPUT_TYPE_ERROR;
                decoded_bad_block = 1'b1;
            end
        endcase
    end else begin
        decoded_type    = INPUT_TYPE_ERROR;
        decoded_bad_block = 1'b1;
    end
end

// ---------------------------------------------------------------
// Lane swap: effective data/type after swap adjustment
// When lanes_swapped, data comes from {current_lower32, swap_buf},
// and TERM types are remapped by +4/-4 lanes.
// ---------------------------------------------------------------
reg [DATA_WIDTH-1:0] eff_data;
reg [3:0]            eff_type;

always @* begin
    if (lanes_swapped) begin
        eff_data = {decoded_data_masked[31:0], swap_buf_reg};
        // Remap TERM types: shift by 4 lanes
        case (decoded_type)
            INPUT_TYPE_DATA:   eff_type = INPUT_TYPE_DATA;
            INPUT_TYPE_TERM_0: eff_type = INPUT_TYPE_TERM_4;
            INPUT_TYPE_TERM_1: eff_type = INPUT_TYPE_TERM_5;
            INPUT_TYPE_TERM_2: eff_type = INPUT_TYPE_TERM_6;
            INPUT_TYPE_TERM_3: eff_type = INPUT_TYPE_TERM_7;
            // TERM_4..7: upper part goes to current beat, lower to next
            // In cut-through without hold-back, treat as DATA + deferred TERM
            INPUT_TYPE_TERM_4: eff_type = INPUT_TYPE_TERM_0; // 4 bytes consumed, 0 remain
            INPUT_TYPE_TERM_5: eff_type = INPUT_TYPE_TERM_1;
            INPUT_TYPE_TERM_6: eff_type = INPUT_TYPE_TERM_2;
            INPUT_TYPE_TERM_7: eff_type = INPUT_TYPE_TERM_3;
            INPUT_TYPE_IDLE:   eff_type = INPUT_TYPE_IDLE;
            INPUT_TYPE_ERROR:  eff_type = INPUT_TYPE_ERROR;
            default:           eff_type = decoded_type;
        endcase
    end else begin
        eff_data = decoded_data_masked;
        eff_type = decoded_type;
    end
end

// ---------------------------------------------------------------
// CRC-32 (runs in PARALLEL with data output, off critical path)
// ---------------------------------------------------------------
reg [31:0] crc_state = 32'hFFFFFFFF;
wire [31:0] crc_next;
wire [7:0] crc_valid;
reg  [7:0] crc_valid_save = 8'd0;

lfsr #(
    .LFSR_WIDTH(32),
    .LFSR_POLY(32'h4c11db7),
    .LFSR_CONFIG("GALOIS"),
    .LFSR_FEED_FORWARD(0),
    .REVERSE(1),
    .DATA_WIDTH(64),
    .STYLE("AUTO")
) eth_crc (
    .data_in(eff_data),
    .state_in(crc_state),
    .data_out(),
    .state_out(crc_next)
);

assign crc_valid[7] = crc_next == ~32'h2144df1c;
assign crc_valid[6] = crc_next == ~32'hc622f71d;
assign crc_valid[5] = crc_next == ~32'hb1c2a1a3;
assign crc_valid[4] = crc_next == ~32'h9d6cdf7e;
assign crc_valid[3] = crc_next == ~32'h6522df69;
assign crc_valid[2] = crc_next == ~32'he60914ae;
assign crc_valid[1] = crc_next == ~32'he38a6876;
assign crc_valid[0] = crc_next == ~32'h6b87b1ec;

// ---------------------------------------------------------------
// Cut-through state machine (combinational)
// Directly drives output from current-cycle decoded data.
// No look-ahead, no hold-back buffer.
// Uses eff_data/eff_type which incorporate lane swap.
// ---------------------------------------------------------------
reg        state_reg = STATE_IDLE, state_next;
reg        reset_crc;
reg        fcs_check_now;  // pulse: check FCS residue this cycle
reg        fcs_ok_comb;    // combinational FCS result

reg [DATA_WIDTH-1:0] m_axis_tdata_next;
reg [7:0]            m_axis_tkeep_next;
reg                  m_axis_tvalid_next;
reg                  m_axis_tlast_next;
reg                  m_axis_tuser_next;

always @* begin
    state_next         = STATE_IDLE;
    reset_crc          = 1'b0;
    fcs_check_now      = 1'b0;
    fcs_ok_comb        = 1'b0;

    m_axis_tdata_next  = decoded_data_masked;
    m_axis_tkeep_next  = 8'h00;
    m_axis_tvalid_next = 1'b0;
    m_axis_tlast_next  = 1'b0;
    m_axis_tuser_next  = 1'b0;

    case (state_reg)
        STATE_IDLE: begin
            reset_crc = 1'b1;
            if (decoded_type == INPUT_TYPE_START_0 ||
                decoded_type == INPUT_TYPE_START_4) begin
                // START detected — begin frame, CRC starts next cycle
                // Keep reset_crc=1 so crc_state is 0xFFFFFFFF for first DATA block
                state_next = STATE_PAYLOAD;
            end else begin
                state_next = STATE_IDLE;
            end
        end

        STATE_PAYLOAD: begin
            if (eff_type == INPUT_TYPE_DATA) begin
                // Data block — output all 8 bytes immediately
                m_axis_tdata_next  = lanes_swapped ?
                    {descrambled_data[31:0], swap_buf_reg} : descrambled_data;
                m_axis_tkeep_next  = 8'hff;
                m_axis_tvalid_next = 1'b1;
                state_next         = STATE_PAYLOAD;
            end else if (eff_type[3]) begin
                // TERM block — output partial data + tlast
                // FCS NOT stripped: output includes FCS bytes
                m_axis_tdata_next  = eff_data;
                m_axis_tvalid_next = 1'b1;
                m_axis_tlast_next  = 1'b1;
                fcs_check_now      = 1'b1;
                reset_crc          = 1'b1;
                state_next         = STATE_IDLE;

                // tkeep based on effective (lane-adjusted) TERM type
                case (eff_type)
                    INPUT_TYPE_TERM_0: m_axis_tkeep_next = 8'h00; // zero-byte end marker
                    INPUT_TYPE_TERM_1: m_axis_tkeep_next = 8'h01;
                    INPUT_TYPE_TERM_2: m_axis_tkeep_next = 8'h03;
                    INPUT_TYPE_TERM_3: m_axis_tkeep_next = 8'h07;
                    INPUT_TYPE_TERM_4: m_axis_tkeep_next = 8'h0f;
                    INPUT_TYPE_TERM_5: m_axis_tkeep_next = 8'h1f;
                    INPUT_TYPE_TERM_6: m_axis_tkeep_next = 8'h3f;
                    INPUT_TYPE_TERM_7: m_axis_tkeep_next = 8'h7f;
                    default:           m_axis_tkeep_next = 8'h00;
                endcase

                // FCS residue check based on effective TERM position
                case (eff_type)
                    INPUT_TYPE_TERM_0: fcs_ok_comb = crc_valid_save[7];
                    INPUT_TYPE_TERM_1: fcs_ok_comb = crc_valid[0];
                    INPUT_TYPE_TERM_2: fcs_ok_comb = crc_valid[1];
                    INPUT_TYPE_TERM_3: fcs_ok_comb = crc_valid[2];
                    INPUT_TYPE_TERM_4: fcs_ok_comb = crc_valid[3];
                    INPUT_TYPE_TERM_5: fcs_ok_comb = crc_valid[4];
                    INPUT_TYPE_TERM_6: fcs_ok_comb = crc_valid[5];
                    INPUT_TYPE_TERM_7: fcs_ok_comb = crc_valid[6];
                    default:           fcs_ok_comb = 1'b0;
                endcase
            end else begin
                // Error or unexpected block in mid-frame
                m_axis_tvalid_next = 1'b1;
                m_axis_tlast_next  = 1'b1;
                m_axis_tuser_next  = 1'b1; // error flag
                reset_crc          = 1'b1;
                state_next         = STATE_IDLE;
            end
        end
    endcase
end

// ---------------------------------------------------------------
// Output registers + FCS result
// ---------------------------------------------------------------
reg [DATA_WIDTH-1:0] m_axis_tdata_reg  = {DATA_WIDTH{1'b0}};
reg [7:0]            m_axis_tkeep_reg  = 8'd0;
reg                  m_axis_tvalid_reg = 1'b0;
reg                  m_axis_tlast_reg  = 1'b0;
reg                  m_axis_tuser_reg  = 1'b0;

reg                  rx_fcs_valid_reg  = 1'b0;
reg                  rx_fcs_ok_reg     = 1'b0;

reg                  rx_bad_block_reg     = 1'b0;
reg                  rx_start_packet_reg  = 1'b0;
reg                  rx_error_bad_fcs_reg = 1'b0;

assign m_axis_tdata  = m_axis_tdata_reg;
assign m_axis_tkeep  = m_axis_tkeep_reg;
assign m_axis_tvalid = m_axis_tvalid_reg;
assign m_axis_tlast  = m_axis_tlast_reg;
assign m_axis_tuser  = m_axis_tuser_reg;

assign rx_fcs_valid    = rx_fcs_valid_reg;
assign rx_fcs_ok       = rx_fcs_ok_reg;

assign rx_bad_block    = rx_bad_block_reg;
assign rx_start_packet = rx_start_packet_reg;
assign rx_error_bad_fcs = rx_error_bad_fcs_reg;

// ---------------------------------------------------------------
// Frame sync
// ---------------------------------------------------------------
ll_eth_frame_sync #(
    .HDR_WIDTH(HDR_WIDTH),
    .BITSLIP_HIGH_CYCLES(1),
    .BITSLIP_LOW_CYCLES(8)
) frame_sync_inst (
    .clk(clk),
    .rst(rst),
    .serdes_rx_hdr(serdes_rx_hdr),
    .serdes_rx_bitslip(serdes_rx_bitslip),
    .rx_block_lock(rx_block_lock)
);

// ---------------------------------------------------------------
// Sequential logic
// ---------------------------------------------------------------
always @(posedge clk) begin
    // --- Input register ---
    rx_data_reg <= serdes_rx_data;
    rx_hdr_reg  <= serdes_rx_hdr;

    // --- Scrambler state update ---
    if (!SCRAMBLER_DISABLE) begin
        scrambler_state <= descrambler_state_out;
    end

    // --- Lane swap tracking ---
    // Speculatively buffer upper 32 bits every cycle
    swap_buf_reg <= decoded_data_masked[63:32];

    if (decoded_type == INPUT_TYPE_START_0) begin
        lanes_swapped <= 1'b0;
    end else if (decoded_type == INPUT_TYPE_START_4) begin
        lanes_swapped <= 1'b1;
    end else if (state_reg == STATE_IDLE) begin
        lanes_swapped <= 1'b0;
    end

    // --- CRC state update ---
    if (reset_crc) begin
        crc_state <= 32'hFFFFFFFF;
    end else begin
        crc_state <= crc_next;
    end
    crc_valid_save <= crc_valid;

    // --- State machine ---
    state_reg <= state_next;

    // --- AXI-Stream output registers ---
    m_axis_tdata_reg  <= m_axis_tdata_next;
    m_axis_tkeep_reg  <= m_axis_tkeep_next;
    m_axis_tvalid_reg <= m_axis_tvalid_next;
    m_axis_tlast_reg  <= m_axis_tlast_next;
    m_axis_tuser_reg  <= m_axis_tuser_next;

    // --- Late FCS result (1 cycle after tlast) ---
    rx_fcs_valid_reg  <= fcs_check_now;
    rx_fcs_ok_reg     <= fcs_ok_comb;
    rx_error_bad_fcs_reg <= fcs_check_now && !fcs_ok_comb;

    // --- Status pulses ---
    rx_start_packet_reg <= (decoded_type == INPUT_TYPE_START_0) ||
                           (decoded_type == INPUT_TYPE_START_4);
    rx_bad_block_reg    <= decoded_bad_block;

    // --- Reset ---
    if (rst) begin
        rx_data_reg          <= {DATA_WIDTH{1'b0}};
        rx_hdr_reg           <= 2'b00;
        scrambler_state      <= {58{1'b1}};
        swap_buf_reg         <= 32'd0;
        lanes_swapped        <= 1'b0;
        crc_state            <= 32'hFFFFFFFF;
        state_reg            <= STATE_IDLE;
        m_axis_tvalid_reg    <= 1'b0;
        rx_fcs_valid_reg     <= 1'b0;
        rx_bad_block_reg     <= 1'b0;
        rx_start_packet_reg  <= 1'b0;
        rx_error_bad_fcs_reg <= 1'b0;
    end
end

endmodule

`resetall
