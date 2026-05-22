/*
Copyright (c) 2023, Advanced Micro Devices, Inc. All rights reserved.
SPDX-License-Identifier: MIT
*/

// tx_replay_top
//
// Subsystem for the BRAM-driven GTF MAC TX replay path. Hangs on a single
// sys_if_switch slot (16-bit window); address[15] splits the window:
//   addr[15] = 0 -> BRAM (0x0000..0x7FFF, 8 KB usable at addr[12:2])
//   addr[15] = 1 -> control/status registers (0x8000..0xFFFF)
//
// Upward, exposes:
//   - the BRAM Port-B handle (tx_bram_clkb / web / addrb / dinb / doutb)
//     for the wizard state machine to consume on tx_axis_clk[0]
//   - the four control signals + two status returns

module tx_replay_top (
    // sys_if slot (sys_if_clk domain, 32b)
    input  wire        sys_if_clk    ,
    input  wire        sys_if_rstn   ,
    input  wire        sys_if_wen    ,
    input  wire [31:0] sys_if_addr   ,
    input  wire [31:0] sys_if_wdata  ,
    output wire [31:0] sys_if_rdata  ,

    // BRAM Port-B (tx_bram_clkb domain, 16b read) -- wired up to the wizard
    input  wire        tx_bram_clkb  ,
    input  wire        tx_bram_web   ,
    input  wire [15:0] tx_bram_addrb ,
    input  wire [15:0] tx_bram_dinb  ,
    output wire [15:0] tx_bram_doutb ,

    // Control signals out to the wizard (sys_if_clk domain)
    output wire        tx_enable     ,
    output wire [1:0]  tx_mode       ,
    output wire        tx_sw_trigger ,    // 1-cycle pulse in sys_if_clk
    output wire [11:0] tx_frame_len  ,

    // Status returns from the wizard (sys_if_clk domain after CDC in wizard)
    input  wire        tx_busy       ,
    input  wire [15:0] tx_frame_cnt
);


// ----------------------------------------------------------------
//  Address demux -- addr[15] selects BRAM (0) vs regs (1)
//
//  The sys_if_switch already filters writes to this slot via sys_if_wen,
//  so we just gate it further to keep the regs and BRAM independent
// ----------------------------------------------------------------

wire bram_sel = (sys_if_addr[15] == 1'b0);
wire regs_sel = (sys_if_addr[15] == 1'b1);

wire        bram_wen   = sys_if_wen & bram_sel;
wire        regs_wen   = sys_if_wen & regs_sel;

wire [31:0] bram_rdata;
wire [31:0] regs_rdata;

assign sys_if_rdata = ({32{bram_sel}} & bram_rdata) |
                      ({32{regs_sel}} & regs_rdata);


// ----------------------------------------------------------------
//  BRAM
// ----------------------------------------------------------------

tx_replay_bram tx_replay_bram (
    .sys_if_clk    ( sys_if_clk    ),
    .sys_if_rstn   ( sys_if_rstn   ),
    .sys_if_wen    ( bram_wen      ),
    .sys_if_addr   ( sys_if_addr   ),
    .sys_if_wdata  ( sys_if_wdata  ),
    .sys_if_rdata  ( bram_rdata    ),

    .tx_clkb       ( tx_bram_clkb  ),
    .tx_web        ( tx_bram_web   ),
    .tx_addrb      ( tx_bram_addrb ),
    .tx_dinb       ( tx_bram_dinb  ),
    .tx_doutb      ( tx_bram_doutb )
);


// ----------------------------------------------------------------
//  Control / status registers
// ----------------------------------------------------------------

tx_replay_regs tx_replay_regs (
    .sys_if_clk          ( sys_if_clk    ),
    .sys_if_rstn         ( sys_if_rstn   ),
    .sys_if_wen          ( regs_wen      ),
    .sys_if_addr         ( sys_if_addr   ),
    .sys_if_wdata        ( sys_if_wdata  ),
    .sys_if_rdata        ( regs_rdata    ),

    .IO_CONTROL_ENABLE   ( tx_enable     ),
    .IO_CONTROL_MODE     ( tx_mode       ),
    .IO_SW_TRIGGER_PULSE ( tx_sw_trigger ),
    .IO_FRAME_LENGTH     ( tx_frame_len  ),
    .IO_SCRATCH_VALUE    (               ),

    .IO_STATUS_BUSY      ( tx_busy       ),
    .IO_STATUS_FRAME_CNT ( tx_frame_cnt  )
);

endmodule
