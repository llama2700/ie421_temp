/*
Copyright (C) 2024, Advanced Micro Devices, Inc. All rights reserved.
SPDX-License-Identifier: X11
*/

//------------------------------------------------------------------------------
//
//  Adapted from reg_qsfp_i2c_logic.v for Renesas jitter cleaner user access.
//  Register addresses offset to 0x100+ to coexist with freq_counter_regs.
//
//  Register Map:
//    0x100 - CONTROL: Write {bit[31]=RW, bits[7:0]=DeviceID} to trigger I2C txn
//                     Read  {bit[31]=RW, bit[30]=CMPLT, bits[7:0]=DeviceID}
//    0x104 - ADDR:    I2C register address (bits[7:0])
//    0x108 - WDATA:   Write data (bits[7:0])
//    0x10C - RDATA:   Read data result (bits[7:0])
//    0x120 - SCRATCH: Read/write test register (default 0x600DFEED)
//
//------------------------------------------------------------------------------

module reg_i2c_user_logic(
    output reg        IO_CONTROL_PULSE,
    output reg  [0:0] IO_CONTROL_RW,
    output reg  [7:0] IO_CONTROL_ID,
    output reg  [7:0] IO_ADDR_ADDR,
    output reg  [7:0] IO_WDATA_WDATA,
    input  wire [7:0] IO_RDATA_RDATA,
    input  wire       IO_CONTROL_CMPLT,
    input  wire        aclk,
    input  wire        aresetn,
    input  wire        wen,
    input  wire [31:0] addr,
    input  wire [31:0] wdata,
    output reg  [31:0] rdata
);


// ####################################################
// #
// #   Local Parameters
// #
// ####################################################

localparam ADDR_CONTROL = 'h00000100;
localparam ADDR_ADDR    = 'h00000104;
localparam ADDR_WDATA   = 'h00000108;
localparam ADDR_RDATA   = 'h0000010C;
localparam ADDR_SCRATCH = 'h00000120;

localparam DFLT_CONTROL_RW    = 'h00000000;
localparam DFLT_CONTROL_ID    = 'h00000000;
localparam DFLT_ADDR_ADDR     = 'h00000000;
localparam DFLT_WDATA_WDATA   = 'h00000000;
localparam DFLT_SCRATCH       = 'h600dfeed;


// ####################################################
// #
// #   Write Registers
// #
// ####################################################

always@(posedge aclk)
begin
    if (!aresetn)
        IO_CONTROL_PULSE <= 'h0;
    else if ( (addr == ADDR_CONTROL) && wen)
        IO_CONTROL_PULSE <= 'h1;
    else
        IO_CONTROL_PULSE <= 'h0;
end

always@(posedge aclk)
begin
    if (!aresetn)
        IO_CONTROL_RW <= DFLT_CONTROL_RW;
    else if ( (addr == ADDR_CONTROL) && wen)
        IO_CONTROL_RW <= wdata[31:31];
end

always@(posedge aclk)
begin
    if (!aresetn)
        IO_CONTROL_ID <= DFLT_CONTROL_ID;
    else if ( (addr == ADDR_CONTROL) && wen)
        IO_CONTROL_ID <= wdata[7:0];
end

always@(posedge aclk)
begin
    if (!aresetn)
        IO_ADDR_ADDR <= DFLT_ADDR_ADDR;
    else if ( (addr == ADDR_ADDR) && wen)
        IO_ADDR_ADDR <= wdata[7:0];
end

always@(posedge aclk)
begin
    if (!aresetn)
        IO_WDATA_WDATA <= DFLT_WDATA_WDATA;
    else if ( (addr == ADDR_WDATA) && wen)
        IO_WDATA_WDATA <= wdata[7:0];
end

reg [31:0] IO_SCRATCH;
always@(posedge aclk)
begin
    if (!aresetn)
        IO_SCRATCH <= DFLT_SCRATCH;
    else if ( (addr == ADDR_SCRATCH) && wen)
        IO_SCRATCH <= wdata[31:0];
end


// ####################################################
// #
// #   Read Data Register Definition
// #
// ####################################################

reg [31:0] RDATA_CONTROL;
reg [31:0] RDATA_ADDR;
reg [31:0] RDATA_WDATA;
reg [31:0] RDATA_RDATA;
reg [31:0] RDATA_SCRATCH;

// ####################################################
// #
// #   Read Data Register Fill
// #
// ####################################################

always@(*)
begin
    RDATA_CONTROL = 'h0;
    RDATA_ADDR = 'h0;
    RDATA_WDATA = 'h0;
    RDATA_RDATA = 'h0;
    RDATA_SCRATCH = 'h0;

    RDATA_CONTROL[31:31] = IO_CONTROL_RW;
    RDATA_CONTROL[30]    = IO_CONTROL_CMPLT;
    RDATA_CONTROL[7:0]   = IO_CONTROL_ID;
    RDATA_ADDR[7:0]      = IO_ADDR_ADDR;
    RDATA_WDATA[7:0]     = IO_WDATA_WDATA;
    RDATA_RDATA[7:0]     = IO_RDATA_RDATA;
    RDATA_SCRATCH        = IO_SCRATCH;
end

// ####################################################
// #
// #   Read Data Mux
// #
// ####################################################

always@(*)
begin
    rdata <=
            ( {32{addr == ADDR_CONTROL}} & RDATA_CONTROL ) |
            ( {32{addr == ADDR_ADDR}}    & RDATA_ADDR    ) |
            ( {32{addr == ADDR_WDATA}}   & RDATA_WDATA   ) |
            ( {32{addr == ADDR_RDATA}}   & RDATA_RDATA   ) |
            ( {32{addr == ADDR_SCRATCH}} & RDATA_SCRATCH ) |
            'h0;
end

endmodule
