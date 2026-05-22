/*
Copyright (c) 2023, Advanced Micro Devices, Inc. All rights reserved.
SPDX-License-Identifier: MIT
*/

/*
create_ip -name blk_mem_gen -vendor xilinx.com -library ip -version 8.4 -module_name blk_mem_gen_tx
set_property -dict [list \
  CONFIG.Interface_Type {Native} \
  CONFIG.Enable_A {Always_Enabled} \
  CONFIG.Enable_B {Always_Enabled} \
  CONFIG.Memory_Type {True_Dual_Port_RAM} \
  CONFIG.Write_Width_A {32} \
  CONFIG.Write_Depth_A {2048} \
  CONFIG.Write_Width_B {16} \
  CONFIG.Use_RSTA_Pin {false} \
  CONFIG.Use_RSTB_Pin {false} \
] [get_ips blk_mem_gen_tx]
*/


module tx_replay_bram (
    // System Interface (sys_if_clk domain, 32b)
    input  wire        sys_if_clk    ,
    input  wire        sys_if_rstn   ,
    input  wire        sys_if_wen    ,
    input  wire [31:0] sys_if_addr   ,
    input  wire [31:0] sys_if_wdata  ,
    output wire [31:0] sys_if_rdata  ,

    // TX replay Port-B (tx_clkb domain, 16b)
    input  wire        tx_clkb       ,
    input  wire        tx_web        ,
    input  wire [15:0] tx_addrb      ,
    input  wire [15:0] tx_dinb       ,
    output wire [15:0] tx_doutb
);

// True Dual Port RAM
// Port A - 32 bit data, 2048 deep, 11 bit addr (sys_if_addr[12:2] = 32b word index)
// Port B - 16 bit data, 4096 deep, 12 bit addr (tx_addrb[11:0] = 16b word index)
// Total capacity 8 KB; each 32b write at addr N populates two 16b reads at Port-B
// addresses 2N (low 16b) and 2N+1 (high 16b)
blk_mem_gen_tx blk_mem_gen_tx (
    .clka   ( sys_if_clk        ),
    .wea    ( sys_if_wen        ),
    .addra  ( sys_if_addr[12:2] ),
    .dina   ( sys_if_wdata      ),
    .douta  ( sys_if_rdata      ),

    .clkb   ( tx_clkb           ),
    .web    ( tx_web            ),
    .addrb  ( tx_addrb[11:0]    ),
    .dinb   ( tx_dinb           ),
    .doutb  ( tx_doutb          )
);

endmodule
