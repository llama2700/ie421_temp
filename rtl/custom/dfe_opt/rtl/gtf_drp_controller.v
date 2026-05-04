/*

GTF DRP Controller
Serializes and arbitrates DRP transactions to the GTF_CHANNEL primitive.
Three requestors: calibration FSM (priority 0), eye scan engine (1), host (2).

*/

`resetall
`timescale 1ns / 1ps
`default_nettype none

module gtf_drp_controller #(
    parameter DRP_ADDR_WIDTH = 16,
    parameter DRP_DATA_WIDTH = 16
)
(
    input  wire                       clk,
    input  wire                       rst,

    /*
     * DRP master to GTF_CHANNEL
     */
    output reg  [DRP_ADDR_WIDTH-1:0]  drp_addr,
    output reg  [DRP_DATA_WIDTH-1:0]  drp_di,
    output reg                        drp_en,
    output reg                        drp_we,
    input  wire [DRP_DATA_WIDTH-1:0]  drp_do,
    input  wire                       drp_rdy,

    /*
     * Requestor 0: Calibration FSM (highest priority)
     */
    input  wire [DRP_ADDR_WIDTH-1:0]  req0_addr,
    input  wire [DRP_DATA_WIDTH-1:0]  req0_wdata,
    input  wire                       req0_valid,
    input  wire                       req0_wr,
    output wire [DRP_DATA_WIDTH-1:0]  req0_rdata,
    output wire                       req0_done,
    output wire                       req0_grant,

    /*
     * Requestor 1: Eye Scan Engine
     */
    input  wire [DRP_ADDR_WIDTH-1:0]  req1_addr,
    input  wire [DRP_DATA_WIDTH-1:0]  req1_wdata,
    input  wire                       req1_valid,
    input  wire                       req1_wr,
    output wire [DRP_DATA_WIDTH-1:0]  req1_rdata,
    output wire                       req1_done,
    output wire                       req1_grant,

    /*
     * Requestor 2: Host Direct Access (lowest priority)
     */
    input  wire [DRP_ADDR_WIDTH-1:0]  req2_addr,
    input  wire [DRP_DATA_WIDTH-1:0]  req2_wdata,
    input  wire                       req2_valid,
    input  wire                       req2_wr,
    output wire [DRP_DATA_WIDTH-1:0]  req2_rdata,
    output wire                       req2_done,
    output wire                       req2_grant
);

// State encoding
localparam [2:0]
    ST_IDLE      = 3'd0,
    ST_GRANT     = 3'd1,
    ST_ASSERT_EN = 3'd2,
    ST_WAIT_RDY  = 3'd3,
    ST_DONE      = 3'd4;

reg [2:0] state_reg, state_next;

// Which requestor is currently granted
reg [1:0] granted_reg, granted_next;

// Latched transaction
reg [DRP_ADDR_WIDTH-1:0] latched_addr_reg, latched_addr_next;
reg [DRP_DATA_WIDTH-1:0] latched_wdata_reg, latched_wdata_next;
reg                      latched_wr_reg, latched_wr_next;

// Read data capture
reg [DRP_DATA_WIDTH-1:0] rdata_reg, rdata_next;

// Done pulses
reg done_0_reg, done_0_next;
reg done_1_reg, done_1_next;
reg done_2_reg, done_2_next;

// Grant signals
reg grant_0_reg, grant_0_next;
reg grant_1_reg, grant_1_next;
reg grant_2_reg, grant_2_next;

// Outputs
assign req0_rdata = rdata_reg;
assign req0_done  = done_0_reg;
assign req0_grant = grant_0_reg;

assign req1_rdata = rdata_reg;
assign req1_done  = done_1_reg;
assign req1_grant = grant_1_reg;

assign req2_rdata = rdata_reg;
assign req2_done  = done_2_reg;
assign req2_grant = grant_2_reg;

always @* begin
    state_next       = state_reg;
    granted_next     = granted_reg;
    latched_addr_next  = latched_addr_reg;
    latched_wdata_next = latched_wdata_reg;
    latched_wr_next    = latched_wr_reg;
    rdata_next       = rdata_reg;

    done_0_next  = 1'b0;
    done_1_next  = 1'b0;
    done_2_next  = 1'b0;
    grant_0_next = 1'b0;
    grant_1_next = 1'b0;
    grant_2_next = 1'b0;

    drp_addr = {DRP_ADDR_WIDTH{1'b0}};
    drp_di   = {DRP_DATA_WIDTH{1'b0}};
    drp_en   = 1'b0;
    drp_we   = 1'b0;

    case (state_reg)
        ST_IDLE: begin
            // Fixed priority arbitration: 0 > 1 > 2
            if (req0_valid) begin
                granted_next     = 2'd0;
                latched_addr_next  = req0_addr;
                latched_wdata_next = req0_wdata;
                latched_wr_next    = req0_wr;
                grant_0_next     = 1'b1;
                state_next       = ST_ASSERT_EN;
            end else if (req1_valid) begin
                granted_next     = 2'd1;
                latched_addr_next  = req1_addr;
                latched_wdata_next = req1_wdata;
                latched_wr_next    = req1_wr;
                grant_1_next     = 1'b1;
                state_next       = ST_ASSERT_EN;
            end else if (req2_valid) begin
                granted_next     = 2'd2;
                latched_addr_next  = req2_addr;
                latched_wdata_next = req2_wdata;
                latched_wr_next    = req2_wr;
                grant_2_next     = 1'b1;
                state_next       = ST_ASSERT_EN;
            end
        end

        ST_ASSERT_EN: begin
            // Drive DRP for one cycle
            drp_addr = latched_addr_reg;
            drp_di   = latched_wdata_reg;
            drp_en   = 1'b1;
            drp_we   = latched_wr_reg;
            state_next = ST_WAIT_RDY;
        end

        ST_WAIT_RDY: begin
            // Hold address/data stable, wait for drp_rdy
            drp_addr = latched_addr_reg;
            drp_di   = latched_wdata_reg;
            if (drp_rdy) begin
                rdata_next = drp_do;
                state_next = ST_DONE;
            end
        end

        ST_DONE: begin
            // Assert done for the granted requestor
            case (granted_reg)
                2'd0: done_0_next = 1'b1;
                2'd1: done_1_next = 1'b1;
                2'd2: done_2_next = 1'b1;
                default: ;
            endcase
            state_next = ST_IDLE;
        end

        default: begin
            state_next = ST_IDLE;
        end
    endcase
end

always @(posedge clk) begin
    if (rst) begin
        state_reg        <= ST_IDLE;
        granted_reg      <= 2'd0;
        latched_addr_reg <= {DRP_ADDR_WIDTH{1'b0}};
        latched_wdata_reg <= {DRP_DATA_WIDTH{1'b0}};
        latched_wr_reg   <= 1'b0;
        rdata_reg        <= {DRP_DATA_WIDTH{1'b0}};
        done_0_reg       <= 1'b0;
        done_1_reg       <= 1'b0;
        done_2_reg       <= 1'b0;
        grant_0_reg      <= 1'b0;
        grant_1_reg      <= 1'b0;
        grant_2_reg      <= 1'b0;
    end else begin
        state_reg        <= state_next;
        granted_reg      <= granted_next;
        latched_addr_reg <= latched_addr_next;
        latched_wdata_reg <= latched_wdata_next;
        latched_wr_reg   <= latched_wr_next;
        rdata_reg        <= rdata_next;
        done_0_reg       <= done_0_next;
        done_1_reg       <= done_1_next;
        done_2_reg       <= done_2_next;
        grant_0_reg      <= grant_0_next;
        grant_1_reg      <= grant_1_next;
        grant_2_reg      <= grant_2_next;
    end
end

endmodule

`resetall
