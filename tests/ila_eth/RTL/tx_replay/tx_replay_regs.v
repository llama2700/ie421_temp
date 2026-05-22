/*
Copyright (c) 2023, Advanced Micro Devices, Inc. All rights reserved.
SPDX-License-Identifier: MIT
*/

module tx_replay_regs (
    output reg  [0:0]  IO_CONTROL_ENABLE,
    output reg  [1:0]  IO_CONTROL_MODE,
    output reg  [0:0]  IO_SW_TRIGGER_PULSE,
    output reg  [11:0] IO_FRAME_LENGTH,
    output reg  [31:0] IO_SCRATCH_VALUE,
    input  wire [0:0]  IO_STATUS_BUSY,
    input  wire [15:0] IO_STATUS_FRAME_CNT,
    input  wire        sys_if_clk,
    input  wire        sys_if_rstn,
    input  wire        sys_if_wen,
    input  wire [31:0] sys_if_addr,
    input  wire [31:0] sys_if_wdata,
    output reg  [31:0] sys_if_rdata
);


// ####################################################
// #
// #   Local Parameters
// #
// ####################################################

localparam ADDR_HEADER0 = 'h00008000;
localparam ADDR_HEADER1 = 'h00008004;
localparam ADDR_HEADER2 = 'h00008008;
localparam ADDR_HEADER3 = 'h0000800C;
localparam ADDR_STATUS  = 'h00008010;
localparam ADDR_CONTROL = 'h00008014;
localparam ADDR_FRAME_LENGTH = 'h00008018;
localparam ADDR_SCRATCH = 'h0000801C;

localparam DFLT_CONTROL_ENABLE  = 1'b0;
localparam DFLT_CONTROL_MODE    = 2'b00;       // sw-trigger only at reset
localparam DFLT_FRAME_LENGTH    = 12'd30;      // matches the legacy 30-word frame
localparam DFLT_SCRATCH         = 32'h0;

// "TX Rply v1.0    " when read as ascii, msb-first per 32b word
localparam HDR_VAL_0 = 32'h54582052;  // "TX R"
localparam HDR_VAL_1 = 32'h706C7920;  // "ply "
localparam HDR_VAL_2 = 32'h76312E30;  // "v1.0"
localparam HDR_VAL_3 = 32'h20202020;  // "    "


// ####################################################
// #
// #   Write Registers
// #
// ####################################################

// IO_CONTROL_ENABLE -- CONTROL bit[0]
always@(posedge sys_if_clk)
begin
    if (!sys_if_rstn)
        IO_CONTROL_ENABLE <= DFLT_CONTROL_ENABLE;
    else if ( (sys_if_addr == ADDR_CONTROL) && sys_if_wen)
        IO_CONTROL_ENABLE <= sys_if_wdata[0:0];
end

// IO_CONTROL_MODE -- CONTROL bits[2:1]
// Only update while not BUSY -- the wizard side treats MODE as quasi-static
always@(posedge sys_if_clk)
begin
    if (!sys_if_rstn)
        IO_CONTROL_MODE <= DFLT_CONTROL_MODE;
    else if ( (sys_if_addr == ADDR_CONTROL) && sys_if_wen && !IO_STATUS_BUSY)
        IO_CONTROL_MODE <= sys_if_wdata[2:1];
end

// IO_SW_TRIGGER_PULSE -- CONTROL bit[3], write-1-pulse self-clearing
always@(posedge sys_if_clk)
begin
    if (!sys_if_rstn)
        IO_SW_TRIGGER_PULSE <= 1'b0;
    else if ( (sys_if_addr == ADDR_CONTROL) && sys_if_wen)
        IO_SW_TRIGGER_PULSE <= sys_if_wdata[3:3];
    else
        IO_SW_TRIGGER_PULSE <= 1'b0;
end

// IO_FRAME_LENGTH -- 12b word count, clamped to >= 2
// Quasi-static, only update while not BUSY
always@(posedge sys_if_clk)
begin
    if (!sys_if_rstn)
        IO_FRAME_LENGTH <= DFLT_FRAME_LENGTH;
    else if ( (sys_if_addr == ADDR_FRAME_LENGTH) && sys_if_wen && !IO_STATUS_BUSY) begin
        if (sys_if_wdata[11:0] < 12'd2)
            IO_FRAME_LENGTH <= 12'd2;
        else
            IO_FRAME_LENGTH <= sys_if_wdata[11:0];
    end
end

// IO_SCRATCH_VALUE -- 32b read/write sanity register
always@(posedge sys_if_clk)
begin
    if (!sys_if_rstn)
        IO_SCRATCH_VALUE <= DFLT_SCRATCH;
    else if ( (sys_if_addr == ADDR_SCRATCH) && sys_if_wen)
        IO_SCRATCH_VALUE <= sys_if_wdata;
end


// ####################################################
// #
// #   Read Data Register Fill
// #
// ####################################################

reg [31:0] RDATA_HEADER0;
reg [31:0] RDATA_HEADER1;
reg [31:0] RDATA_HEADER2;
reg [31:0] RDATA_HEADER3;
reg [31:0] RDATA_STATUS;
reg [31:0] RDATA_CONTROL;
reg [31:0] RDATA_FRAME_LENGTH;
reg [31:0] RDATA_SCRATCH;

always@(*)
begin
    RDATA_HEADER0      = HDR_VAL_0;
    RDATA_HEADER1      = HDR_VAL_1;
    RDATA_HEADER2      = HDR_VAL_2;
    RDATA_HEADER3      = HDR_VAL_3;

    RDATA_STATUS       = 'h0;
    RDATA_STATUS[0:0]    = IO_STATUS_BUSY;
    RDATA_STATUS[31:16]  = IO_STATUS_FRAME_CNT;

    RDATA_CONTROL      = 'h0;
    RDATA_CONTROL[0:0]   = IO_CONTROL_ENABLE;
    RDATA_CONTROL[2:1]   = IO_CONTROL_MODE;

    RDATA_FRAME_LENGTH = 'h0;
    RDATA_FRAME_LENGTH[11:0] = IO_FRAME_LENGTH;

    RDATA_SCRATCH      = IO_SCRATCH_VALUE;
end


// ####################################################
// #
// #   Read Data Mux
// #
// ####################################################

always@(*)
begin
    sys_if_rdata <=
            ( {32{sys_if_addr == ADDR_HEADER0}}      & RDATA_HEADER0      ) |
            ( {32{sys_if_addr == ADDR_HEADER1}}      & RDATA_HEADER1      ) |
            ( {32{sys_if_addr == ADDR_HEADER2}}      & RDATA_HEADER2      ) |
            ( {32{sys_if_addr == ADDR_HEADER3}}      & RDATA_HEADER3      ) |
            ( {32{sys_if_addr == ADDR_STATUS}}       & RDATA_STATUS       ) |
            ( {32{sys_if_addr == ADDR_CONTROL}}      & RDATA_CONTROL      ) |
            ( {32{sys_if_addr == ADDR_FRAME_LENGTH}} & RDATA_FRAME_LENGTH ) |
            ( {32{sys_if_addr == ADDR_SCRATCH}}      & RDATA_SCRATCH      ) |
            'h0;
end

endmodule
