/*
 * Ultra-low-latency fused RX PCS+MAC.
 *
 * 2-stage pipeline (vs Corundum's 4):
 *   Cycle 0: Register SerDes input, combinational descramble + block type decode
 *   Cycle 1: Frame extraction, CRC check, register AXI-Stream outputs
 *
 * Eliminates extra pipeline stage for lane swap via speculative swap buffer.
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
    output wire                   m_axis_tuser,   // bit 0 = FCS error

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

// Internal decoded type (same encoding as Corundum for consistency)
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

localparam [1:0]
    STATE_IDLE    = 2'd0,
    STATE_PAYLOAD = 2'd1,
    STATE_LAST    = 2'd2;

// ---------------------------------------------------------------
// Cycle 0 registers: SerDes input capture + descrambler state
// ---------------------------------------------------------------
reg [DATA_WIDTH-1:0] rx_data_reg   = {DATA_WIDTH{1'b0}};
reg [HDR_WIDTH-1:0]  rx_hdr_reg    = 2'b00;
reg [57:0]           scrambler_state = {58{1'b1}};

// Descrambler wires (combinational from registered input)
wire [DATA_WIDTH-1:0] descrambled_data;
wire [57:0]           descrambler_state_out;

// Descrambler: 58-bit Fibonacci, feed-forward, bit-reverse
// Operates combinationally on registered serdes data
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
// Combinational block type decode (from registered + descrambled data)
// ---------------------------------------------------------------
wire [7:0] block_type = descrambled_data[7:0];  // type field in bits [7:0]
reg  [3:0] decoded_type;
reg        decoded_bad_block;

// Data masking: extract payload bytes from encoded block
reg [DATA_WIDTH-1:0] decoded_data_masked;

always @* begin
    decoded_type      = INPUT_TYPE_IDLE;
    decoded_bad_block = 1'b0;
    decoded_data_masked = {DATA_WIDTH{1'b0}};

    if (rx_hdr_reg == SYNC_DATA) begin
        // Pure data block
        decoded_type        = INPUT_TYPE_DATA;
        decoded_data_masked = descrambled_data;
    end else if (rx_hdr_reg == SYNC_CTRL) begin
        case (block_type)
            BLOCK_TYPE_CTRL: begin
                decoded_type = INPUT_TYPE_IDLE;
            end
            BLOCK_TYPE_START_0: begin
                decoded_type        = INPUT_TYPE_START_0;
                decoded_data_masked = {descrambled_data[63:8], 8'd0};
            end
            BLOCK_TYPE_START_4: begin
                decoded_type        = INPUT_TYPE_START_4;
                decoded_data_masked = {descrambled_data[63:8], 8'd0};
            end
            BLOCK_TYPE_TERM_0: begin
                decoded_type        = INPUT_TYPE_TERM_0;
                decoded_data_masked = 64'd0;
            end
            BLOCK_TYPE_TERM_1: begin
                decoded_type        = INPUT_TYPE_TERM_1;
                decoded_data_masked = {56'd0, descrambled_data[15:8]};
            end
            BLOCK_TYPE_TERM_2: begin
                decoded_type        = INPUT_TYPE_TERM_2;
                decoded_data_masked = {48'd0, descrambled_data[23:8]};
            end
            BLOCK_TYPE_TERM_3: begin
                decoded_type        = INPUT_TYPE_TERM_3;
                decoded_data_masked = {40'd0, descrambled_data[31:8]};
            end
            BLOCK_TYPE_TERM_4: begin
                decoded_type        = INPUT_TYPE_TERM_4;
                decoded_data_masked = {32'd0, descrambled_data[39:8]};
            end
            BLOCK_TYPE_TERM_5: begin
                decoded_type        = INPUT_TYPE_TERM_5;
                decoded_data_masked = {24'd0, descrambled_data[47:8]};
            end
            BLOCK_TYPE_TERM_6: begin
                decoded_type        = INPUT_TYPE_TERM_6;
                decoded_data_masked = {16'd0, descrambled_data[55:8]};
            end
            BLOCK_TYPE_TERM_7: begin
                decoded_type        = INPUT_TYPE_TERM_7;
                decoded_data_masked = {8'd0, descrambled_data[63:8]};
            end
            default: begin
                decoded_type      = INPUT_TYPE_ERROR;
                decoded_bad_block = 1'b1;
            end
        endcase
    end else begin
        // Invalid sync header
        decoded_type      = INPUT_TYPE_ERROR;
        decoded_bad_block = 1'b1;
    end
end

// ---------------------------------------------------------------
// Lane swap logic (zero added latency)
// Speculatively buffer upper 32 bits every cycle
// ---------------------------------------------------------------
reg [31:0] swap_buf_reg  = 32'd0;
reg        lanes_swapped = 1'b0;

reg        delay_type_valid = 1'b0;
reg [3:0]  delay_type       = INPUT_TYPE_IDLE;

// Pipeline stage d0: decoded + lane-adjusted data/type
reg [DATA_WIDTH-1:0] input_data_d0 = {DATA_WIDTH{1'b0}};
reg [3:0]            input_type_d0 = INPUT_TYPE_IDLE;

// ---------------------------------------------------------------
// CRC-32 computation
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
    .data_in(input_data_d0),
    .state_in(crc_state),
    .data_out(),
    .state_out(crc_next)
);

// CRC residue checks for each termination position
assign crc_valid[7] = crc_next == ~32'h2144df1c;
assign crc_valid[6] = crc_next == ~32'hc622f71d;
assign crc_valid[5] = crc_next == ~32'hb1c2a1a3;
assign crc_valid[4] = crc_next == ~32'h9d6cdf7e;
assign crc_valid[3] = crc_next == ~32'h6522df69;
assign crc_valid[2] = crc_next == ~32'he60914ae;
assign crc_valid[1] = crc_next == ~32'he38a6876;
assign crc_valid[0] = crc_next == ~32'h6b87b1ec;

// ---------------------------------------------------------------
// Cycle 1: Frame extraction state machine
// ---------------------------------------------------------------
reg [1:0] state_reg = STATE_IDLE, state_next;
reg       reset_crc;

// Pipeline stage d1: delayed for state machine output
reg [DATA_WIDTH-1:0] input_data_d1 = {DATA_WIDTH{1'b0}};
reg [3:0]            input_type_d1 = INPUT_TYPE_IDLE;

// AXI-Stream output registers
reg [DATA_WIDTH-1:0] m_axis_tdata_reg  = {DATA_WIDTH{1'b0}}, m_axis_tdata_next;
reg [7:0]            m_axis_tkeep_reg  = 8'd0, m_axis_tkeep_next;
reg                  m_axis_tvalid_reg = 1'b0, m_axis_tvalid_next;
reg                  m_axis_tlast_reg  = 1'b0, m_axis_tlast_next;
reg                  m_axis_tuser_reg  = 1'b0, m_axis_tuser_next;

// Status
reg                  rx_bad_block_reg    = 1'b0;
reg                  rx_start_packet_reg = 1'b0;
reg                  rx_error_bad_fcs_reg = 1'b0, rx_error_bad_fcs_next;

assign m_axis_tdata  = m_axis_tdata_reg;
assign m_axis_tkeep  = m_axis_tkeep_reg;
assign m_axis_tvalid = m_axis_tvalid_reg;
assign m_axis_tlast  = m_axis_tlast_reg;
assign m_axis_tuser  = m_axis_tuser_reg;

assign rx_bad_block    = rx_bad_block_reg;
assign rx_start_packet = rx_start_packet_reg;
assign rx_error_bad_fcs = rx_error_bad_fcs_reg;

// ---------------------------------------------------------------
// Frame sync instantiation
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
// State machine (combinational)
// ---------------------------------------------------------------
always @* begin
    state_next = STATE_IDLE;
    reset_crc  = 1'b0;

    m_axis_tdata_next  = input_data_d1;
    m_axis_tkeep_next  = 8'd0;
    m_axis_tvalid_next = 1'b0;
    m_axis_tlast_next  = 1'b0;
    m_axis_tuser_next  = 1'b0;
    rx_error_bad_fcs_next = 1'b0;

    case (state_reg)
        STATE_IDLE: begin
            reset_crc = 1'b1;
            if (input_type_d1 == INPUT_TYPE_START_0) begin
                reset_crc  = 1'b0;
                state_next = STATE_PAYLOAD;
            end else begin
                state_next = STATE_IDLE;
            end
        end
        STATE_PAYLOAD: begin
            m_axis_tdata_next  = input_data_d1;
            m_axis_tkeep_next  = 8'hff;
            m_axis_tvalid_next = 1'b1;
            m_axis_tlast_next  = 1'b0;
            m_axis_tuser_next  = 1'b0;

            if (input_type_d0[3]) begin
                // TERM coming next cycle — will reset CRC
                reset_crc = 1'b1;
            end

            if (input_type_d0 == INPUT_TYPE_DATA) begin
                state_next = STATE_PAYLOAD;
            end else if (input_type_d0[3]) begin
                // INPUT_TYPE_TERM_*
                if (input_type_d0 <= INPUT_TYPE_TERM_4) begin
                    // end this cycle
                    case (input_type_d0)
                        INPUT_TYPE_TERM_0: m_axis_tkeep_next = 8'b00001111;
                        INPUT_TYPE_TERM_1: m_axis_tkeep_next = 8'b00011111;
                        INPUT_TYPE_TERM_2: m_axis_tkeep_next = 8'b00111111;
                        INPUT_TYPE_TERM_3: m_axis_tkeep_next = 8'b01111111;
                        INPUT_TYPE_TERM_4: m_axis_tkeep_next = 8'b11111111;
                        default:           m_axis_tkeep_next = 8'hff;
                    endcase
                    m_axis_tlast_next = 1'b1;
                    if ((input_type_d0 == INPUT_TYPE_TERM_0 && crc_valid_save[7]) ||
                        (input_type_d0 == INPUT_TYPE_TERM_1 && crc_valid[0]) ||
                        (input_type_d0 == INPUT_TYPE_TERM_2 && crc_valid[1]) ||
                        (input_type_d0 == INPUT_TYPE_TERM_3 && crc_valid[2]) ||
                        (input_type_d0 == INPUT_TYPE_TERM_4 && crc_valid[3])) begin
                        // CRC valid
                    end else begin
                        m_axis_tuser_next     = 1'b1;
                        rx_error_bad_fcs_next = 1'b1;
                    end
                    state_next = STATE_IDLE;
                end else begin
                    // TERM_5..7: need extra cycle
                    state_next = STATE_LAST;
                end
            end else begin
                // control or error in packet
                m_axis_tlast_next = 1'b1;
                m_axis_tuser_next = 1'b1;
                reset_crc         = 1'b1;
                state_next        = STATE_IDLE;
            end
        end
        STATE_LAST: begin
            m_axis_tdata_next  = input_data_d1;
            m_axis_tkeep_next  = 8'hff;
            m_axis_tvalid_next = 1'b1;
            m_axis_tlast_next  = 1'b1;
            m_axis_tuser_next  = 1'b0;
            reset_crc          = 1'b1;

            case (input_type_d1)
                INPUT_TYPE_TERM_5: m_axis_tkeep_next = 8'b00000001;
                INPUT_TYPE_TERM_6: m_axis_tkeep_next = 8'b00000011;
                INPUT_TYPE_TERM_7: m_axis_tkeep_next = 8'b00000111;
                default:           m_axis_tkeep_next = 8'b00000111;
            endcase

            if ((input_type_d1 == INPUT_TYPE_TERM_5 && crc_valid_save[4]) ||
                (input_type_d1 == INPUT_TYPE_TERM_6 && crc_valid_save[5]) ||
                (input_type_d1 == INPUT_TYPE_TERM_7 && crc_valid_save[6])) begin
                // CRC valid
            end else begin
                m_axis_tuser_next     = 1'b1;
                rx_error_bad_fcs_next = 1'b1;
            end

            state_next = STATE_IDLE;
        end
    endcase
end

// ---------------------------------------------------------------
// Clocked logic
// ---------------------------------------------------------------
always @(posedge clk) begin
    // --- Cycle 0: capture SerDes input ---
    rx_data_reg <= serdes_rx_data;
    rx_hdr_reg  <= serdes_rx_hdr;

    // Update scrambler state (registered for next cycle's descramble)
    if (rx_hdr_reg == SYNC_DATA && !SCRAMBLER_DISABLE) begin
        scrambler_state <= descrambler_state_out;
    end else if (rx_hdr_reg == SYNC_CTRL && !SCRAMBLER_DISABLE) begin
        scrambler_state <= descrambler_state_out;
    end

    // --- Lane swap handling ---
    // Speculatively store upper 32 bits every cycle
    swap_buf_reg <= decoded_data_masked[63:32];

    rx_bad_block_reg    <= 1'b0;
    rx_start_packet_reg <= 1'b0;
    delay_type_valid    <= 1'b0;

    if (decoded_type == INPUT_TYPE_START_0) begin
        // START in lane 0: no swap
        lanes_swapped   <= 1'b0;
        rx_start_packet_reg <= 1'b1;
        input_type_d0   <= INPUT_TYPE_START_0;
        input_data_d0   <= decoded_data_masked;
    end else if (decoded_type == INPUT_TYPE_START_4) begin
        // START in lane 4: enable lane swap
        lanes_swapped       <= 1'b1;
        rx_start_packet_reg <= 1'b1;
        delay_type_valid    <= 1'b1;

        if (delay_type_valid) begin
            input_type_d0 <= delay_type;
        end else begin
            input_type_d0 <= INPUT_TYPE_IDLE;
        end
        input_data_d0 <= {decoded_data_masked[31:0], swap_buf_reg};
    end else if (lanes_swapped) begin
        if (delay_type_valid) begin
            input_type_d0 <= delay_type;
        end else if (decoded_type == INPUT_TYPE_DATA) begin
            input_type_d0 <= INPUT_TYPE_DATA;
        end else if (decoded_type[3]) begin
            // TERM block while lanes swapped
            case (decoded_type)
                INPUT_TYPE_TERM_0: input_type_d0 <= INPUT_TYPE_TERM_4;
                INPUT_TYPE_TERM_1: input_type_d0 <= INPUT_TYPE_TERM_5;
                INPUT_TYPE_TERM_2: input_type_d0 <= INPUT_TYPE_TERM_6;
                INPUT_TYPE_TERM_3: input_type_d0 <= INPUT_TYPE_TERM_7;
                INPUT_TYPE_TERM_4: begin
                    delay_type_valid <= 1'b1;
                    input_type_d0    <= INPUT_TYPE_DATA;
                end
                INPUT_TYPE_TERM_5: begin
                    delay_type_valid <= 1'b1;
                    input_type_d0    <= INPUT_TYPE_DATA;
                end
                INPUT_TYPE_TERM_6: begin
                    delay_type_valid <= 1'b1;
                    input_type_d0    <= INPUT_TYPE_DATA;
                end
                INPUT_TYPE_TERM_7: begin
                    delay_type_valid <= 1'b1;
                    input_type_d0    <= INPUT_TYPE_DATA;
                end
                default: begin
                    rx_bad_block_reg <= 1'b1;
                    input_type_d0    <= INPUT_TYPE_ERROR;
                end
            endcase
        end else if (decoded_type == INPUT_TYPE_IDLE) begin
            input_type_d0 <= INPUT_TYPE_IDLE;
        end else begin
            rx_bad_block_reg <= 1'b1;
            input_type_d0    <= INPUT_TYPE_ERROR;
        end
        input_data_d0 <= {decoded_data_masked[31:0], swap_buf_reg};
    end else begin
        // Not swapped
        if (decoded_type == INPUT_TYPE_DATA) begin
            input_type_d0 <= INPUT_TYPE_DATA;
        end else if (decoded_type[3]) begin
            input_type_d0 <= decoded_type;
        end else if (decoded_type == INPUT_TYPE_IDLE) begin
            input_type_d0 <= INPUT_TYPE_IDLE;
        end else if (decoded_bad_block) begin
            rx_bad_block_reg <= 1'b1;
            input_type_d0    <= INPUT_TYPE_ERROR;
        end else begin
            input_type_d0 <= INPUT_TYPE_IDLE;
        end
        input_data_d0 <= decoded_data_masked;
    end

    // Delayed type for lane-swap continuation
    if (decoded_type == INPUT_TYPE_DATA) begin
        delay_type <= INPUT_TYPE_DATA;
    end else if (rx_hdr_reg == SYNC_CTRL) begin
        case (decoded_type)
            INPUT_TYPE_START_4: delay_type <= INPUT_TYPE_START_0;
            INPUT_TYPE_TERM_0:  delay_type <= INPUT_TYPE_TERM_4;
            INPUT_TYPE_TERM_1:  delay_type <= INPUT_TYPE_TERM_5;
            INPUT_TYPE_TERM_2:  delay_type <= INPUT_TYPE_TERM_6;
            INPUT_TYPE_TERM_3:  delay_type <= INPUT_TYPE_TERM_7;
            INPUT_TYPE_TERM_4:  delay_type <= INPUT_TYPE_TERM_0;
            INPUT_TYPE_TERM_5:  delay_type <= INPUT_TYPE_TERM_1;
            INPUT_TYPE_TERM_6:  delay_type <= INPUT_TYPE_TERM_2;
            INPUT_TYPE_TERM_7:  delay_type <= INPUT_TYPE_TERM_3;
            default:            delay_type <= INPUT_TYPE_ERROR;
        endcase
    end else begin
        delay_type <= INPUT_TYPE_ERROR;
    end

    // --- Pipeline d0 -> d1 ---
    input_type_d1 <= input_type_d0;
    input_data_d1 <= input_data_d0;

    // --- CRC update ---
    if (reset_crc) begin
        crc_state <= 32'hFFFFFFFF;
    end else begin
        crc_state <= crc_next;
    end
    crc_valid_save <= crc_valid;

    // --- Cycle 1: state machine outputs ---
    state_reg <= state_next;

    m_axis_tdata_reg  <= m_axis_tdata_next;
    m_axis_tkeep_reg  <= m_axis_tkeep_next;
    m_axis_tvalid_reg <= m_axis_tvalid_next;
    m_axis_tlast_reg  <= m_axis_tlast_next;
    m_axis_tuser_reg  <= m_axis_tuser_next;

    rx_error_bad_fcs_reg <= rx_error_bad_fcs_next;

    // --- Reset ---
    if (rst) begin
        state_reg       <= STATE_IDLE;
        scrambler_state <= {58{1'b1}};
        lanes_swapped   <= 1'b0;
        delay_type_valid <= 1'b0;
        delay_type      <= INPUT_TYPE_IDLE;
        input_type_d0   <= INPUT_TYPE_IDLE;
        input_type_d1   <= INPUT_TYPE_IDLE;
        crc_state       <= 32'hFFFFFFFF;

        m_axis_tvalid_reg    <= 1'b0;
        rx_bad_block_reg     <= 1'b0;
        rx_start_packet_reg  <= 1'b0;
        rx_error_bad_fcs_reg <= 1'b0;
    end
end

endmodule

`resetall
