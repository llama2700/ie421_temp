#
# Copyright (c) 2023, Advanced Micro Devices, Inc. All rights reserved.
# SPDX-License-Identifier: MIT
#
################################################################################
#
#  UL3422 RECOV_CLK reference design constraints
#
#  Pinout pulled from origin/ul3422:QSFP_I2C/XDC/ul3422.xdc and
#  origin/ul3422:Renesas_I2C_Programming/XDC/ul3422.xdc
#
################################################################################


################################################################################
#
#  300 MHz LVDS Reference Clocks
#
################################################################################

#  300 MHz Reference clock for DDR1, Bank 66 (1.2V) -- system clock
set_property PACKAGE_PIN    AY22             [get_ports clk_ddr_lvds_300_n]    ;# Bank 66   - 1V2_VCCO  - CLK_DDR_LVDS_300_N        - IO_L13N_T2L_N1_GC_QBC_66_AY22
set_property IOSTANDARD     LVDS             [get_ports clk_ddr_lvds_300_n]    ;# Bank 66   - 1V2_VCCO  - CLK_DDR_LVDS_300_N        - IO_L13N_T2L_N1_GC_QBC_66_AY22
set_property PACKAGE_PIN    AW23             [get_ports clk_ddr_lvds_300_p]    ;# Bank 66   - 1V2_VCCO  - CLK_DDR_LVDS_300_P        - IO_L13P_T2L_N0_GC_QBC_66_AW23
set_property IOSTANDARD     LVDS             [get_ports clk_ddr_lvds_300_p]    ;# Bank 66   - 1V2_VCCO  - CLK_DDR_LVDS_300_P        - IO_L13P_T2L_N0_GC_QBC_66_AW23

#  300 MHz Reference clock, Bank 65 (1.8V) -- GTF freerun source
set_property PACKAGE_PIN    AW18             [get_ports clk_sys_lvds_300_n]    ;# Bank 65   - 1V8_SYS   - CLK_SYS_LVDS_300_N        - IO_L14N_T2L_N3_GC_A05_D21_65_AW18
set_property IOSTANDARD     LVDS             [get_ports clk_sys_lvds_300_n]    ;# Bank 65   - 1V8_SYS   - CLK_SYS_LVDS_300_N        - IO_L14N_T2L_N3_GC_A05_D21_65_AW18
set_property PACKAGE_PIN    AW19             [get_ports clk_sys_lvds_300_p]    ;# Bank 65   - 1V8_SYS   - CLK_SYS_LVDS_300_P        - IO_L14P_T2L_N2_GC_A04_D20_65_AW19
set_property IOSTANDARD     LVDS             [get_ports clk_sys_lvds_300_p]    ;# Bank 65   - 1V8_SYS   - CLK_SYS_LVDS_300_P        - IO_L14P_T2L_N2_GC_A04_D20_65_AW19


################################################################################
#
#  QSFPDD I2C Interface, Bank 93 (3.3V)
#
################################################################################

set_property PACKAGE_PIN    F12              [get_ports fpga_scl_r]            ;# Bank 93   - 3V3_VCC0  - FPGA_SCL_R                - IO_L12P_AD8P_93_F12
set_property IOSTANDARD     LVCMOS33         [get_ports fpga_scl_r]            ;# Bank 93   - 3V3_VCC0  - FPGA_SCL_R                - IO_L12P_AD8P_93_F12
set_property PACKAGE_PIN    F11              [get_ports fpga_sda_r]            ;# Bank 93   - 3V3_VCC0  - FPGA_SDA_R                - IO_L12N_AD8N_93_F11
set_property IOSTANDARD     LVCMOS33         [get_ports fpga_sda_r]            ;# Bank 93   - 3V3_VCC0  - FPGA_SDA_R                - IO_L12N_AD8N_93_F11

set_property PACKAGE_PIN    G14              [get_ports fpga_mux_rstn]         ;# Bank 93   - 3V3_VCC0  - FPGA_MUX_RSTN             - IO_L11P_AD9P_93_G14
set_property IOSTANDARD     LVCMOS33         [get_ports fpga_mux_rstn]         ;# Bank 93   - 3V3_VCC0  - FPGA_MUX_RSTN             - IO_L11P_AD9P_93_G14

set_property PACKAGE_PIN    H16              [get_ports qsfpdd1_io_reset_b]    ;# Bank 93   - 3V3_VCC0  - QSFPDD1_IO_RESET_B        - IO_L8N_HDGC_93_H16
set_property IOSTANDARD     LVCMOS33         [get_ports qsfpdd1_io_reset_b]    ;# Bank 93   - 3V3_VCC0  - QSFPDD1_IO_RESET_B        - IO_L8N_HDGC_93_H16
set_property PACKAGE_PIN    F15              [get_ports qsfpdd2_io_reset_b]    ;# Bank 93   - 3V3_VCC0  - QSFPDD2_IO_RESET_B        - IO_L9N_AD11N_93_F15
set_property IOSTANDARD     LVCMOS33         [get_ports qsfpdd2_io_reset_b]    ;# Bank 93   - 3V3_VCC0  - QSFPDD2_IO_RESET_B        - IO_L9N_AD11N_93_F15


################################################################################
#
#  QSFPDD1 GTF Connections, Bank 127 (1.5V) -- RAW (loopback)
#
################################################################################

set_property PACKAGE_PIN    AG37             [get_ports synce_clk_127_lvds_n]  ;# Bank 127              - SYNCE_CLK_127_LVDS_N      - MGTREFCLK0N_127_AG37
set_property PACKAGE_PIN    AG36             [get_ports synce_clk_127_lvds_p]  ;# Bank 127              - SYNCE_CLK_127_LVDS_P      - MGTREFCLK0P_127_AG36

set_property PACKAGE_PIN    AK44             [get_ports {qsfpdd1_rx_n[0]}]     ;# Bank 127              - QSFPDD1_RX1_N             - MGTFRXN0_127_AK44
set_property PACKAGE_PIN    AK43             [get_ports {qsfpdd1_rx_p[0]}]     ;# Bank 127              - QSFPDD1_RX1_P             - MGTFRXP0_127_AK43
set_property PACKAGE_PIN    AJ46             [get_ports {qsfpdd1_rx_n[1]}]     ;# Bank 127              - QSFPDD1_RX2_N             - MGTFRXN1_127_AJ46
set_property PACKAGE_PIN    AJ45             [get_ports {qsfpdd1_rx_p[1]}]     ;# Bank 127              - QSFPDD1_RX2_P             - MGTFRXP1_127_AJ45
set_property PACKAGE_PIN    AH44             [get_ports {qsfpdd1_rx_n[2]}]     ;# Bank 127              - QSFPDD1_RX3_N             - MGTFRXN2_127_AH44
set_property PACKAGE_PIN    AH43             [get_ports {qsfpdd1_rx_p[2]}]     ;# Bank 127              - QSFPDD1_RX3_P             - MGTFRXP2_127_AH43
set_property PACKAGE_PIN    AG46             [get_ports {qsfpdd1_rx_n[3]}]     ;# Bank 127              - QSFPDD1_RX4_N             - MGTFRXN3_127_AG46
set_property PACKAGE_PIN    AG45             [get_ports {qsfpdd1_rx_p[3]}]     ;# Bank 127              - QSFPDD1_RX4_P             - MGTFRXP3_127_AG45
set_property PACKAGE_PIN    AM39             [get_ports {qsfpdd1_tx_n[0]}]     ;# Bank 127              - QSFPDD1_TX1_N             - MGTFTXN0_127_AM39
set_property PACKAGE_PIN    AM38             [get_ports {qsfpdd1_tx_p[0]}]     ;# Bank 127              - QSFPDD1_TX1_P             - MGTFTXP0_127_AM38
set_property PACKAGE_PIN    AL41             [get_ports {qsfpdd1_tx_n[1]}]     ;# Bank 127              - QSFPDD1_TX2_N             - MGTFTXN1_127_AL41
set_property PACKAGE_PIN    AL40             [get_ports {qsfpdd1_tx_p[1]}]     ;# Bank 127              - QSFPDD1_TX2_P             - MGTFTXP1_127_AL40
set_property PACKAGE_PIN    AK39             [get_ports {qsfpdd1_tx_n[2]}]     ;# Bank 127              - QSFPDD1_TX3_N             - MGTFTXN2_127_AK39
set_property PACKAGE_PIN    AK38             [get_ports {qsfpdd1_tx_p[2]}]     ;# Bank 127              - QSFPDD1_TX3_P             - MGTFTXP2_127_AK38
set_property PACKAGE_PIN    AJ41             [get_ports {qsfpdd1_tx_n[3]}]     ;# Bank 127              - QSFPDD1_TX4_N             - MGTFTXN3_127_AJ41
set_property PACKAGE_PIN    AJ40             [get_ports {qsfpdd1_tx_p[3]}]     ;# Bank 127              - QSFPDD1_TX4_P             - MGTFTXP3_127_AJ40


################################################################################
#
#  QSFPDD1 spare GTF refclk, Bank 128 (1.5V) -- freq monitor only
#
################################################################################

set_property PACKAGE_PIN    AC37             [get_ports synce_clk_128_lvds_n]  ;# Bank 128              - SYNCE_CLK_128_LVDS_N      - MGTREFCLK0N_128_AC37
set_property PACKAGE_PIN    AC36             [get_ports synce_clk_128_lvds_p]  ;# Bank 128              - SYNCE_CLK_128_LVDS_P      - MGTREFCLK0P_128_AC36


################################################################################
#
#  QSFPDD2 GTF Connections, Bank 130 (1.5V) -- MAC (recovered clock source)
#
################################################################################

set_property PACKAGE_PIN    R37              [get_ports synce_clk_130_lvds_n]  ;# Bank 130              - SYNCE_CLK_130_LVDS_N      - MGTREFCLK0N_130_R37
set_property PACKAGE_PIN    R36              [get_ports synce_clk_130_lvds_p]  ;# Bank 130              - SYNCE_CLK_130_LVDS_P      - MGTREFCLK0P_130_R36

set_property PACKAGE_PIN    V44              [get_ports {qsfpdd2_rx_n[0]}]     ;# Bank 130              - QSFPDD2_RX1_N             - MGTFRXN0_130_V44
set_property PACKAGE_PIN    V43              [get_ports {qsfpdd2_rx_p[0]}]     ;# Bank 130              - QSFPDD2_RX1_P             - MGTFRXP0_130_V43
set_property PACKAGE_PIN    U46              [get_ports {qsfpdd2_rx_n[1]}]     ;# Bank 130              - QSFPDD2_RX2_N             - MGTFRXN1_130_U46
set_property PACKAGE_PIN    U45              [get_ports {qsfpdd2_rx_p[1]}]     ;# Bank 130              - QSFPDD2_RX2_P             - MGTFRXP1_130_U45
set_property PACKAGE_PIN    T44              [get_ports {qsfpdd2_rx_n[2]}]     ;# Bank 130              - QSFPDD2_RX3_N             - MGTFRXN2_130_T44
set_property PACKAGE_PIN    T43              [get_ports {qsfpdd2_rx_p[2]}]     ;# Bank 130              - QSFPDD2_RX3_P             - MGTFRXP2_130_T43
set_property PACKAGE_PIN    R46              [get_ports {qsfpdd2_rx_n[3]}]     ;# Bank 130              - QSFPDD2_RX4_N             - MGTFRXN3_130_R46
set_property PACKAGE_PIN    R45              [get_ports {qsfpdd2_rx_p[3]}]     ;# Bank 130              - QSFPDD2_RX4_P             - MGTFRXP3_130_R45
set_property PACKAGE_PIN    Y39              [get_ports {qsfpdd2_tx_n[0]}]     ;# Bank 130              - QSFPDD2_TX1_N             - MGTFTXN0_130_Y39
set_property PACKAGE_PIN    Y38              [get_ports {qsfpdd2_tx_p[0]}]     ;# Bank 130              - QSFPDD2_TX1_P             - MGTFTXP0_130_Y38
set_property PACKAGE_PIN    W41              [get_ports {qsfpdd2_tx_n[1]}]     ;# Bank 130              - QSFPDD2_TX2_N             - MGTFTXN1_130_W41
set_property PACKAGE_PIN    W40              [get_ports {qsfpdd2_tx_p[1]}]     ;# Bank 130              - QSFPDD2_TX2_P             - MGTFTXP1_130_W40
set_property PACKAGE_PIN    V39              [get_ports {qsfpdd2_tx_n[2]}]     ;# Bank 130              - QSFPDD2_TX3_N             - MGTFTXN2_130_V39
set_property PACKAGE_PIN    V38              [get_ports {qsfpdd2_tx_p[2]}]     ;# Bank 130              - QSFPDD2_TX3_P             - MGTFTXP2_130_V38
set_property PACKAGE_PIN    U41              [get_ports {qsfpdd2_tx_n[3]}]     ;# Bank 130              - QSFPDD2_TX4_N             - MGTFTXN3_130_U41
set_property PACKAGE_PIN    U40              [get_ports {qsfpdd2_tx_p[3]}]     ;# Bank 130              - QSFPDD2_TX4_P             - MGTFTXP3_130_U40


################################################################################
#
#  QSFPDD2 spare GTF refclk, Bank 131 (1.5V) -- freq monitor only
#
################################################################################

set_property PACKAGE_PIN    L37              [get_ports synce_clk_131_lvds_n]  ;# Bank 131              - SYNCE_CLK_131_LVDS_N      - MGTREFCLK0N_131_L37
set_property PACKAGE_PIN    L36              [get_ports synce_clk_131_lvds_p]  ;# Bank 131              - SYNCE_CLK_131_LVDS_P      - MGTREFCLK0P_131_L36


################################################################################
#
#  Alternate GTF refclk, Bank 129 (1.5V) -- freq monitor only
#
################################################################################

set_property PACKAGE_PIN    W37              [get_ports synce_clk_129_lvds_n]  ;# Bank 129              - SYNCE_CLK_129_LVDS_N      - MGTREFCLK0N_129_W37
set_property PACKAGE_PIN    W36              [get_ports synce_clk_129_lvds_p]  ;# Bank 129              - SYNCE_CLK_129_LVDS_P      - MGTREFCLK0P_129_W36


################################################################################
#
#  Renesas Jitter Cleaner GPIO and Reset, Bank 65 (1.8V)
#
################################################################################

set_property PACKAGE_PIN    AY16             [get_ports jitt_resetn]           ;# Bank 65   - 1V8_SYS   - JITT_RESETN               - IO_L10N_T1U_N7_QBC_AD4N_A13_D29_65_AY16
set_property IOSTANDARD     LVCMOS18         [get_ports jitt_resetn]           ;# Bank 65   - 1V8_SYS   - JITT_RESETN               - IO_L10N_T1U_N7_QBC_AD4N_A13_D29_65_AY16

set_property PACKAGE_PIN    AP16             [get_ports jitt1_gpoi0]           ;# Bank 65   - 1V8_SYS   - JITT1_GPOI0               - IO_L19P_T3L_N0_DBC_AD9P_D10_65_AP16
set_property IOSTANDARD     LVCMOS18         [get_ports jitt1_gpoi0]           ;# Bank 65   - 1V8_SYS   - JITT1_GPOI0               - IO_L19P_T3L_N0_DBC_AD9P_D10_65_AP16
set_property PACKAGE_PIN    AT17             [get_ports jitt1_gpoi1]           ;# Bank 65   - 1V8_SYS   - JITT1_GPOI1               - IO_L18P_T2U_N10_AD2P_D12_65_AT17
set_property IOSTANDARD     LVCMOS18         [get_ports jitt1_gpoi1]           ;# Bank 65   - 1V8_SYS   - JITT1_GPOI1               - IO_L18P_T2U_N10_AD2P_D12_65_AT17
set_property PACKAGE_PIN    AU16             [get_ports jitt1_gpoi2]           ;# Bank 65   - 1V8_SYS   - JITT1_GPOI2               - IO_L18N_T2U_N11_AD2N_D13_65_AU16
set_property IOSTANDARD     LVCMOS18         [get_ports jitt1_gpoi2]           ;# Bank 65   - 1V8_SYS   - JITT1_GPOI2               - IO_L18N_T2U_N11_AD2N_D13_65_AU16
set_property PACKAGE_PIN    AV19             [get_ports jitt1_gpoi3]           ;# Bank 65   - 1V8_SYS   - JITT1_GPOI3               - IO_T2U_N12_CSI_ADV_B_65_AV19
set_property IOSTANDARD     LVCMOS18         [get_ports jitt1_gpoi3]           ;# Bank 65   - 1V8_SYS   - JITT1_GPOI3               - IO_T2U_N12_CSI_ADV_B_65_AV19
set_property PACKAGE_PIN    AR16             [get_ports jitt1_gpoi4]           ;# Bank 65   - 1V8_SYS   - JITT1_GPOI4               - IO_L19N_T3L_N1_DBC_AD9N_D11_65_AR16
set_property IOSTANDARD     LVCMOS18         [get_ports jitt1_gpoi4]           ;# Bank 65   - 1V8_SYS   - JITT1_GPOI4               - IO_L19N_T3L_N1_DBC_AD9N_D11_65_AR16
set_property PACKAGE_PIN    AP18             [get_ports jitt1_gpoi5]           ;# Bank 65   - 1V8_SYS   - JITT1_GPOI5               - IO_L20P_T3L_N2_AD1P_D08_65_AP18
set_property IOSTANDARD     LVCMOS18         [get_ports jitt1_gpoi5]           ;# Bank 65   - 1V8_SYS   - JITT1_GPOI5               - IO_L20P_T3L_N2_AD1P_D08_65_AP18


################################################################################
#
#  Recovered Clock Output, Bank 65 (1.8V)
#
################################################################################

set_property PACKAGE_PIN    BF19             [get_ports recov_clk_65_lvds_n]   ;# Bank 65   - 1V8_SYS   - RECOV_CLK_65_LVDS_N       - IO_L4N_T0U_N7_DBC_AD7N_A25_65_BF19
set_property IOSTANDARD     LVDS             [get_ports recov_clk_65_lvds_n]   ;# Bank 65   - 1V8_SYS   - RECOV_CLK_65_LVDS_N       - IO_L4N_T0U_N7_DBC_AD7N_A25_65_BF19
set_property PACKAGE_PIN    BF20             [get_ports recov_clk_65_lvds_p]   ;# Bank 65   - 1V8_SYS   - RECOV_CLK_65_LVDS_P       - IO_L4P_T0U_N6_DBC_AD7P_A24_65_BF20
set_property IOSTANDARD     LVDS             [get_ports recov_clk_65_lvds_p]   ;# Bank 65   - 1V8_SYS   - RECOV_CLK_65_LVDS_P       - IO_L4P_T0U_N6_DBC_AD7P_A24_65_BF20


################################################################################
#
#  Renesas Jitter Cleaner SYNCE Inputs to fabric, Bank 65 (1.8V) -- freq monitor
#
################################################################################

set_property PACKAGE_PIN    BA17             [get_ports synce_clk_65_1_lvds_n] ;# Bank 65   - 1V8_SYS   - SYNCE_CLK_65_1_LVDS_N     - IO_L12N_T1U_N11_GC_A09_D25_65_BA17
set_property IOSTANDARD     LVDS             [get_ports synce_clk_65_1_lvds_n] ;# Bank 65   - 1V8_SYS   - SYNCE_CLK_65_1_LVDS_N     - IO_L12N_T1U_N11_GC_A09_D25_65_BA17
set_property PACKAGE_PIN    AY17             [get_ports synce_clk_65_1_lvds_p] ;# Bank 65   - 1V8_SYS   - SYNCE_CLK_65_1_LVDS_P     - IO_L12P_T1U_N10_GC_A08_D24_65_AY17
set_property IOSTANDARD     LVDS             [get_ports synce_clk_65_1_lvds_p] ;# Bank 65   - 1V8_SYS   - SYNCE_CLK_65_1_LVDS_P     - IO_L12P_T1U_N10_GC_A08_D24_65_AY17
set_property PACKAGE_PIN    BB20             [get_ports synce_clk_65_2_lvds_n] ;# Bank 65   - 1V8_SYS   - SYNCE_CLK_65_2_LVDS_N     - IO_L7N_T1L_N1_QBC_AD13N_A19_65_BB20
set_property IOSTANDARD     LVDS             [get_ports synce_clk_65_2_lvds_n] ;# Bank 65   - 1V8_SYS   - SYNCE_CLK_65_2_LVDS_N     - IO_L7N_T1L_N1_QBC_AD13N_A19_65_BB20
set_property PACKAGE_PIN    BA20             [get_ports synce_clk_65_2_lvds_p] ;# Bank 65   - 1V8_SYS   - SYNCE_CLK_65_2_LVDS_P     - IO_L7P_T1L_N0_QBC_AD13P_A18_65_BA20
set_property IOSTANDARD     LVDS             [get_ports synce_clk_65_2_lvds_p] ;# Bank 65   - 1V8_SYS   - SYNCE_CLK_65_2_LVDS_P     - IO_L7P_T1L_N0_QBC_AD13P_A18_65_BA20


################################################################################
#
#  Clock Generator I2C, Bank 65 (1.8V) -- shared bus to JC + clock generator
#
################################################################################

set_property PACKAGE_PIN    AR20             [get_ports clkgen_scl_r]          ;# Bank 65   - 1V8_SYS   - CLKGEN_SCL_R              - IO_L23P_T3U_N8_I2C_SCLK_65_AR20
set_property IOSTANDARD     LVCMOS18         [get_ports clkgen_scl_r]          ;# Bank 65   - 1V8_SYS   - CLKGEN_SCL_R              - IO_L23P_T3U_N8_I2C_SCLK_65_AR20
set_property PACKAGE_PIN    AT20             [get_ports clkgen_sda_r]          ;# Bank 65   - 1V8_SYS   - CLKGEN_SDA_R              - IO_L23N_T3U_N9_PERSTN1_I2C_SDA_65_AT20
set_property IOSTANDARD     LVCMOS18         [get_ports clkgen_sda_r]          ;# Bank 65   - 1V8_SYS   - CLKGEN_SDA_R              - IO_L23N_T3U_N9_PERSTN1_I2C_SDA_65_AT20


################################################################################
#
#  GTF placement -- pblocks + LOC for the recovered-clock BUFG_GT
#
################################################################################

# Bank 130 -- gtf_top_0 (MAC, recovered-clock source)
create_pblock pblock_bank_130
resize_pblock [get_pblocks pblock_bank_130] -add {CLOCKREGION_X0Y6:CLOCKREGION_X0Y6}
add_cells_to_pblock [get_pblocks pblock_bank_130] [get_cells gtf_top_0/u_gtfwizard_0_example_gtfmac_top/i_gtfmac/IBUFDS_GTE4_INST]
add_cells_to_pblock [get_pblocks pblock_bank_130] [get_cells gtf_top_0/u_gtfwizard_0_example_gtfmac_top/i_gtfmac/gen_blk_multi_ch[0].u_bufg_gt_gtf_recov_clk]
set_property LOC GTF_COMMON_X0Y6   [get_cells gtf_top_0/u_gtfwizard_0_example_gtfmac_top/i_gtfmac/example_gtf_common_inst/gtf_common_inst]
set_property LOC GTF_CHANNEL_X0Y24 [get_cells gtf_top_0/u_gtfwizard_0_example_gtfmac_top/i_gtfmac/gen_blk_multi_ch[0].u_gtf_wiz_ip_top/inst/gtf_channel_inst/gtf_channel_inst]
set_property LOC GTF_CHANNEL_X0Y25 [get_cells gtf_top_0/u_gtfwizard_0_example_gtfmac_top/i_gtfmac/gen_blk_multi_ch[1].u_gtf_wiz_ip_top/inst/gtf_channel_inst/gtf_channel_inst]
set_property LOC GTF_CHANNEL_X0Y26 [get_cells gtf_top_0/u_gtfwizard_0_example_gtfmac_top/i_gtfmac/gen_blk_multi_ch[2].u_gtf_wiz_ip_top/inst/gtf_channel_inst/gtf_channel_inst]
set_property LOC GTF_CHANNEL_X0Y27 [get_cells gtf_top_0/u_gtfwizard_0_example_gtfmac_top/i_gtfmac/gen_blk_multi_ch[3].u_gtf_wiz_ip_top/inst/gtf_channel_inst/gtf_channel_inst]

# Bank 127 -- gtf_top_1 (RAW, loopback)
create_pblock pblock_bank_127
resize_pblock [get_pblocks pblock_bank_127] -add {CLOCKREGION_X0Y3:CLOCKREGION_X0Y3}
add_cells_to_pblock [get_pblocks pblock_bank_127] [get_cells gtf_top_1/gtfwizard_raw_example_top/IBUFDS_GTE4_INST]
set_property LOC GTF_COMMON_X0Y3   [get_cells gtf_top_1/gtfwizard_raw_example_top/example_gtf_common_inst/gtf_common_inst]
set_property LOC GTF_CHANNEL_X0Y12 [get_cells gtf_top_1/gtfwizard_raw_example_top/gen_blk_multi_ch[0].u_gtf_wiz_ip_top/inst/gtf_channel_inst/gtf_channel_inst]
set_property LOC GTF_CHANNEL_X0Y13 [get_cells gtf_top_1/gtfwizard_raw_example_top/gen_blk_multi_ch[1].u_gtf_wiz_ip_top/inst/gtf_channel_inst/gtf_channel_inst]
set_property LOC GTF_CHANNEL_X0Y14 [get_cells gtf_top_1/gtfwizard_raw_example_top/gen_blk_multi_ch[2].u_gtf_wiz_ip_top/inst/gtf_channel_inst/gtf_channel_inst]
set_property LOC GTF_CHANNEL_X0Y15 [get_cells gtf_top_1/gtfwizard_raw_example_top/gen_blk_multi_ch[3].u_gtf_wiz_ip_top/inst/gtf_channel_inst/gtf_channel_inst]
