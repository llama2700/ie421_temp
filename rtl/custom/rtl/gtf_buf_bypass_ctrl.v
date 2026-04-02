/*

GTF Buffer Bypass Controller
Configures the GTF for TX and RX buffer bypass mode to achieve fixed,
deterministic latency through the transceiver. Handles alignment
sequencing after reset and automatic re-alignment on CDR re-lock.

*/

`resetall
`timescale 1ns / 1ps
`default_nettype none

module gtf_buf_bypass_ctrl #(
    parameter DRP_ADDR_WIDTH = 16,
    parameter DRP_DATA_WIDTH = 16,
    // TODO: verify DRP addresses against GTF Wizard output
    parameter TX_BUFFER_CFG_ADDR = 16'h0097,
    parameter RX_BUFFER_CFG_ADDR = 16'h0098,
    parameter ALIGN_TIMEOUT = 32'd1000000  // alignment timeout in clk cycles
)
(
    input  wire        clk,
    input  wire        rst,

    input  wire        buf_bypass_enable,
    output reg         tx_bypass_aligned,
    output reg         rx_bypass_aligned,
    output wire        buf_bypass_aligned,
    output reg         buf_bypass_error,

    /*
     * DRP requestor interface (directly to GTF DRP, not through arbiter
     * since this runs during init before other requestors are active)
     */
    output reg  [DRP_ADDR_WIDTH-1:0]  drp_addr,
    output reg  [DRP_DATA_WIDTH-1:0]  drp_wdata,
    output reg                        drp_valid,
    output reg                        drp_wr,
    input  wire [DRP_DATA_WIDTH-1:0]  drp_rdata,
    input  wire                       drp_done,

    /*
     * GTF sideband - TX phase alignment
     */
    output reg         txphdlyreset,
    output reg         txphdlytstclk,
    output reg         txphalign,
    output reg         txphalignen,
    output reg         txphdlypd,
    input  wire        txphaligndone,
    input  wire        txsyncdone,

    /*
     * GTF sideband - RX phase alignment
     */
    output reg         rxphdlyreset,
    output reg         rxphalign,
    output reg         rxphalignen,
    output reg         rxphdlypd,
    input  wire        rxphaligndone,
    input  wire        rxsyncdone,

    /*
     * GTF reset status
     */
    input  wire        txresetdone,
    input  wire        rxresetdone,
    input  wire        rxcdrlock
);

assign buf_bypass_aligned = tx_bypass_aligned & rx_bypass_aligned;

// State encoding
localparam [3:0]
    ST_IDLE             = 4'd0,
    ST_DRP_READ_TX      = 4'd1,
    ST_DRP_WRITE_TX     = 4'd2,
    ST_DRP_READ_RX      = 4'd3,
    ST_DRP_WRITE_RX     = 4'd4,
    ST_WAIT_TX_RESETDONE = 4'd5,
    ST_TX_ALIGN_START   = 4'd6,
    ST_TX_ALIGN_WAIT    = 4'd7,
    ST_WAIT_RX_RESETDONE = 4'd8,
    ST_RX_ALIGN_START   = 4'd9,
    ST_RX_ALIGN_WAIT    = 4'd10,
    ST_ALIGNED          = 4'd11,
    ST_ERROR            = 4'd12;

reg [3:0]  state_reg, state_next;
reg [31:0] timeout_cnt_reg, timeout_cnt_next;
reg [DRP_DATA_WIDTH-1:0] drp_saved_reg, drp_saved_next;

// CDR lock loss detection for re-alignment
reg rxcdrlock_prev;
wire rxcdrlock_lost;

always @(posedge clk) begin
    if (rst)
        rxcdrlock_prev <= 1'b0;
    else
        rxcdrlock_prev <= rxcdrlock;
end

assign rxcdrlock_lost = rxcdrlock_prev & ~rxcdrlock;

always @* begin
    state_next       = state_reg;
    timeout_cnt_next = timeout_cnt_reg;
    drp_saved_next   = drp_saved_reg;

    drp_addr  = {DRP_ADDR_WIDTH{1'b0}};
    drp_wdata = {DRP_DATA_WIDTH{1'b0}};
    drp_valid = 1'b0;
    drp_wr    = 1'b0;

    txphdlyreset  = 1'b0;
    txphdlytstclk = 1'b0;
    txphalign     = 1'b0;
    txphalignen   = 1'b0;
    txphdlypd     = 1'b1;  // powered down by default

    rxphdlyreset  = 1'b0;
    rxphalign     = 1'b0;
    rxphalignen   = 1'b0;
    rxphdlypd     = 1'b1;

    case (state_reg)
        ST_IDLE: begin
            if (buf_bypass_enable) begin
                state_next = ST_DRP_READ_TX;
            end
        end

        // Read current TX buffer config
        ST_DRP_READ_TX: begin
            drp_addr  = TX_BUFFER_CFG_ADDR;
            drp_valid = 1'b1;
            drp_wr    = 1'b0;
            if (drp_done) begin
                drp_saved_next = drp_rdata;
                state_next = ST_DRP_WRITE_TX;
            end
        end

        // Write TX buffer config for bypass mode (set bypass bit)
        ST_DRP_WRITE_TX: begin
            drp_addr  = TX_BUFFER_CFG_ADDR;
            drp_wdata = drp_saved_reg | 16'h0001;  // set bypass enable bit
            drp_valid = 1'b1;
            drp_wr    = 1'b1;
            if (drp_done) begin
                state_next = ST_DRP_READ_RX;
            end
        end

        // Read current RX buffer config
        ST_DRP_READ_RX: begin
            drp_addr  = RX_BUFFER_CFG_ADDR;
            drp_valid = 1'b1;
            drp_wr    = 1'b0;
            if (drp_done) begin
                drp_saved_next = drp_rdata;
                state_next = ST_DRP_WRITE_RX;
            end
        end

        // Write RX buffer config for bypass mode
        ST_DRP_WRITE_RX: begin
            drp_addr  = RX_BUFFER_CFG_ADDR;
            drp_wdata = drp_saved_reg | 16'h0001;
            drp_valid = 1'b1;
            drp_wr    = 1'b1;
            if (drp_done) begin
                timeout_cnt_next = {32{1'b0}};
                state_next = ST_WAIT_TX_RESETDONE;
            end
        end

        // Wait for TX reset to complete
        ST_WAIT_TX_RESETDONE: begin
            timeout_cnt_next = timeout_cnt_reg + 1;
            if (txresetdone) begin
                state_next = ST_TX_ALIGN_START;
                timeout_cnt_next = {32{1'b0}};
            end else if (timeout_cnt_reg >= ALIGN_TIMEOUT) begin
                state_next = ST_ERROR;
            end
        end

        // Start TX phase alignment
        ST_TX_ALIGN_START: begin
            txphdlypd    = 1'b0;  // power up
            txphdlyreset = 1'b1;  // reset phase delay
            state_next   = ST_TX_ALIGN_WAIT;
            timeout_cnt_next = {32{1'b0}};
        end

        // Wait for TX phase alignment to complete
        ST_TX_ALIGN_WAIT: begin
            txphdlypd   = 1'b0;
            txphalignen = 1'b1;
            txphalign   = 1'b1;
            timeout_cnt_next = timeout_cnt_reg + 1;
            if (txphaligndone) begin
                state_next = ST_WAIT_RX_RESETDONE;
                timeout_cnt_next = {32{1'b0}};
            end else if (timeout_cnt_reg >= ALIGN_TIMEOUT) begin
                state_next = ST_ERROR;
            end
        end

        // Wait for RX reset to complete
        ST_WAIT_RX_RESETDONE: begin
            txphdlypd   = 1'b0;
            txphalignen = 1'b1;
            timeout_cnt_next = timeout_cnt_reg + 1;
            if (rxresetdone) begin
                state_next = ST_RX_ALIGN_START;
                timeout_cnt_next = {32{1'b0}};
            end else if (timeout_cnt_reg >= ALIGN_TIMEOUT) begin
                state_next = ST_ERROR;
            end
        end

        // Start RX phase alignment
        ST_RX_ALIGN_START: begin
            txphdlypd    = 1'b0;
            txphalignen  = 1'b1;
            rxphdlypd    = 1'b0;
            rxphdlyreset = 1'b1;
            state_next   = ST_RX_ALIGN_WAIT;
            timeout_cnt_next = {32{1'b0}};
        end

        // Wait for RX phase alignment to complete
        ST_RX_ALIGN_WAIT: begin
            txphdlypd   = 1'b0;
            txphalignen = 1'b1;
            rxphdlypd   = 1'b0;
            rxphalignen = 1'b1;
            rxphalign   = 1'b1;
            timeout_cnt_next = timeout_cnt_reg + 1;
            if (rxphaligndone) begin
                state_next = ST_ALIGNED;
            end else if (timeout_cnt_reg >= ALIGN_TIMEOUT) begin
                state_next = ST_ERROR;
            end
        end

        // Aligned — monitor for CDR lock loss
        ST_ALIGNED: begin
            txphdlypd   = 1'b0;
            txphalignen = 1'b1;
            rxphdlypd   = 1'b0;
            rxphalignen = 1'b1;
            if (!buf_bypass_enable) begin
                state_next = ST_IDLE;
            end else if (rxcdrlock_lost) begin
                // CDR lost lock, need to re-align
                state_next = ST_RX_ALIGN_START;
                timeout_cnt_next = {32{1'b0}};
            end
        end

        ST_ERROR: begin
            if (!buf_bypass_enable) begin
                state_next = ST_IDLE;
            end
        end

        default: state_next = ST_IDLE;
    endcase
end

always @(posedge clk) begin
    if (rst) begin
        state_reg        <= ST_IDLE;
        timeout_cnt_reg  <= 32'd0;
        drp_saved_reg    <= {DRP_DATA_WIDTH{1'b0}};
        tx_bypass_aligned <= 1'b0;
        rx_bypass_aligned <= 1'b0;
        buf_bypass_error  <= 1'b0;
    end else begin
        state_reg       <= state_next;
        timeout_cnt_reg <= timeout_cnt_next;
        drp_saved_reg   <= drp_saved_next;

        tx_bypass_aligned <= (state_reg == ST_ALIGNED) || (state_reg == ST_WAIT_RX_RESETDONE) ||
                             (state_reg == ST_RX_ALIGN_START) || (state_reg == ST_RX_ALIGN_WAIT);
        rx_bypass_aligned <= (state_reg == ST_ALIGNED);
        buf_bypass_error  <= (state_reg == ST_ERROR);
    end
end

endmodule

`resetall
