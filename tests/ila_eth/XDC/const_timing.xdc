#
# Copyright (c) 2023, Advanced Micro Devices, Inc. All rights reserved.
# SPDX-License-Identifier: MIT
#
################################################################################
#
#  UL3422 RECOV_CLK reference design timing constraints
#
#  Translated from origin/ul3524:RECOV_CLK/XDC/constraint.xdc.  Only the
#  GTF banks/refclks that exist on the UL3422 (127, 128, 129, 130, 131
#  for MGTREFCLK0; 65 for the two fabric SYNCE pairs) are referenced.
#
#  Hierarchical paths (gtf_top_0/u_gtfwizard_0_example_gtfmac_top/i_gtfmac/...,
#  gtf_top_1/gtfwizard_raw_example_top/..., clk_reset/clk_wiz_100mhz,
#  clk_wiz_300_to_161_inst, gen_blk_multi_ch[*]) match the names used in
#  RECOV_CLK/RTL on this branch.
#
################################################################################


################################################################################
#
#  Primary clocks
#
################################################################################

# 300 MHz LVDS reference clocks
create_clock -period 3.333 -name clk_ddr_lvds_300_p   [get_ports clk_ddr_lvds_300_p]
create_clock -period 3.333 -name clk_sys_lvds_300_p   [get_ports clk_sys_lvds_300_p]

# 161.1328125 MHz GTF reference clocks (MGTREFCLK0 of each GT bank)
create_clock -period 6.207 -name synce_clk_127_p      [get_ports synce_clk_127_lvds_p]
create_clock -period 6.207 -name synce_clk_128_p      [get_ports synce_clk_128_lvds_p]
create_clock -period 6.207 -name synce_clk_129_p      [get_ports synce_clk_129_lvds_p]
create_clock -period 6.207 -name synce_clk_130_p      [get_ports synce_clk_130_lvds_p]
create_clock -period 6.207 -name synce_clk_131_p      [get_ports synce_clk_131_lvds_p]

# 161.1328125 MHz fabric SYNCE inputs
create_clock -period 6.207 -name synce_clk_65_1_p     [get_ports synce_clk_65_1_lvds_p]
create_clock -period 6.207 -name synce_clk_65_2_p     [get_ports synce_clk_65_2_lvds_p]


################################################################################
#
#  CDC synchronizer false paths (RTL/sync/syncer_*)
#
################################################################################

set_false_path -to [get_cells -hierarchical -filter {NAME =~ *meta_reg[0]}]


################################################################################
#
#  GTF Port 0 (MAC, gtf_top_0, Bank 130) generated clocks and async groups
#
################################################################################

create_generated_clock -name GTF_0_RXOUTCLK_0 \
                       -source        [get_pins gtf_top_0/u_gtfwizard_0_example_gtfmac_top/i_gtfmac/IBUFDS_GTE4_INST/O] \
                       -multiply_by 4 [get_pins gtf_top_0/u_gtfwizard_0_example_gtfmac_top/i_gtfmac/gen_blk_multi_ch[0].u_gtf_wiz_ip_top/inst/gtf_channel_inst/gtf_channel_inst/RXOUTCLK]

create_generated_clock -name GTF_0_TXOUTCLK_0 \
                       -source        [get_pins gtf_top_0/u_gtfwizard_0_example_gtfmac_top/i_gtfmac/IBUFDS_GTE4_INST/O] \
                       -multiply_by 4 [get_pins gtf_top_0/u_gtfwizard_0_example_gtfmac_top/i_gtfmac/gen_blk_multi_ch[0].u_gtf_wiz_ip_top/inst/gtf_channel_inst/gtf_channel_inst/TXOUTCLK]

create_generated_clock -name GTF_0_RXOUTCLK_1 \
                       -source        [get_pins gtf_top_0/u_gtfwizard_0_example_gtfmac_top/i_gtfmac/IBUFDS_GTE4_INST/O] \
                       -multiply_by 4 [get_pins gtf_top_0/u_gtfwizard_0_example_gtfmac_top/i_gtfmac/gen_blk_multi_ch[1].u_gtf_wiz_ip_top/inst/gtf_channel_inst/gtf_channel_inst/RXOUTCLK]

create_generated_clock -name GTF_0_TXOUTCLK_1 \
                       -source        [get_pins gtf_top_0/u_gtfwizard_0_example_gtfmac_top/i_gtfmac/IBUFDS_GTE4_INST/O] \
                       -multiply_by 4 [get_pins gtf_top_0/u_gtfwizard_0_example_gtfmac_top/i_gtfmac/gen_blk_multi_ch[1].u_gtf_wiz_ip_top/inst/gtf_channel_inst/gtf_channel_inst/TXOUTCLK]

create_generated_clock -name GTF_0_RXOUTCLK_2 \
                       -source        [get_pins gtf_top_0/u_gtfwizard_0_example_gtfmac_top/i_gtfmac/IBUFDS_GTE4_INST/O] \
                       -multiply_by 4 [get_pins gtf_top_0/u_gtfwizard_0_example_gtfmac_top/i_gtfmac/gen_blk_multi_ch[2].u_gtf_wiz_ip_top/inst/gtf_channel_inst/gtf_channel_inst/RXOUTCLK]

create_generated_clock -name GTF_0_TXOUTCLK_2 \
                       -source        [get_pins gtf_top_0/u_gtfwizard_0_example_gtfmac_top/i_gtfmac/IBUFDS_GTE4_INST/O] \
                       -multiply_by 4 [get_pins gtf_top_0/u_gtfwizard_0_example_gtfmac_top/i_gtfmac/gen_blk_multi_ch[2].u_gtf_wiz_ip_top/inst/gtf_channel_inst/gtf_channel_inst/TXOUTCLK]

create_generated_clock -name GTF_0_RXOUTCLK_3 \
                       -source        [get_pins gtf_top_0/u_gtfwizard_0_example_gtfmac_top/i_gtfmac/IBUFDS_GTE4_INST/O] \
                       -multiply_by 4 [get_pins gtf_top_0/u_gtfwizard_0_example_gtfmac_top/i_gtfmac/gen_blk_multi_ch[3].u_gtf_wiz_ip_top/inst/gtf_channel_inst/gtf_channel_inst/RXOUTCLK]

create_generated_clock -name GTF_0_TXOUTCLK_3 \
                       -source        [get_pins gtf_top_0/u_gtfwizard_0_example_gtfmac_top/i_gtfmac/IBUFDS_GTE4_INST/O] \
                       -multiply_by 4 [get_pins gtf_top_0/u_gtfwizard_0_example_gtfmac_top/i_gtfmac/gen_blk_multi_ch[3].u_gtf_wiz_ip_top/inst/gtf_channel_inst/gtf_channel_inst/TXOUTCLK]


# clk_wiz_300_to_161_inst CLKOUT0/1 (gtf_freerun_clk + 425MHz) <-> GTF_0 outclks
set_false_path  -from [get_clocks -of_objects [get_pins clk_wiz_300_to_161_inst/inst/mmcme4_adv_inst/CLKOUT0]] \
                -to   [list [get_clocks -of_objects [get_pins clk_wiz_300_to_161_inst/inst/mmcme4_adv_inst/CLKOUT1]] \
                            [get_clocks GTF_0_RXOUTCLK_0] \
                            [get_clocks GTF_0_RXOUTCLK_1] \
                            [get_clocks GTF_0_RXOUTCLK_2] \
                            [get_clocks GTF_0_RXOUTCLK_3] \
                            [get_clocks GTF_0_TXOUTCLK_0] \
                            [get_clocks GTF_0_TXOUTCLK_1] \
                            [get_clocks GTF_0_TXOUTCLK_2] \
                            [get_clocks GTF_0_TXOUTCLK_3] ]

set_false_path  -from [get_clocks -of_objects [get_pins clk_wiz_300_to_161_inst/inst/mmcme4_adv_inst/CLKOUT1]] \
                -to   [list [get_clocks -of_objects [get_pins clk_wiz_300_to_161_inst/inst/mmcme4_adv_inst/CLKOUT0]] \
                            [get_clocks GTF_0_RXOUTCLK_0] \
                            [get_clocks GTF_0_RXOUTCLK_1] \
                            [get_clocks GTF_0_RXOUTCLK_2] \
                            [get_clocks GTF_0_RXOUTCLK_3] \
                            [get_clocks GTF_0_TXOUTCLK_0] \
                            [get_clocks GTF_0_TXOUTCLK_1] \
                            [get_clocks GTF_0_TXOUTCLK_2] \
                            [get_clocks GTF_0_TXOUTCLK_3] ]

# Each GTF_0 channel TXOUTCLK/RXOUTCLK is async w.r.t. every other GTF_0
# channel and the freerun MMCM outputs.

set_false_path  -from [get_clocks GTF_0_TXOUTCLK_0] \
                -to   [list [get_clocks -of_objects [get_pins clk_wiz_300_to_161_inst/inst/mmcme4_adv_inst/CLKOUT0]] \
                            [get_clocks -of_objects [get_pins clk_wiz_300_to_161_inst/inst/mmcme4_adv_inst/CLKOUT1]] \
                            [get_clocks GTF_0_TXOUTCLK_1] \
                            [get_clocks GTF_0_TXOUTCLK_2] \
                            [get_clocks GTF_0_TXOUTCLK_3] \
                            [get_clocks GTF_0_RXOUTCLK_0] \
                            [get_clocks GTF_0_RXOUTCLK_1] \
                            [get_clocks GTF_0_RXOUTCLK_2] \
                            [get_clocks GTF_0_RXOUTCLK_3] ]

set_false_path  -from [get_clocks GTF_0_TXOUTCLK_1] \
                -to   [list [get_clocks -of_objects [get_pins clk_wiz_300_to_161_inst/inst/mmcme4_adv_inst/CLKOUT0]] \
                            [get_clocks -of_objects [get_pins clk_wiz_300_to_161_inst/inst/mmcme4_adv_inst/CLKOUT1]] \
                            [get_clocks GTF_0_TXOUTCLK_0] \
                            [get_clocks GTF_0_TXOUTCLK_2] \
                            [get_clocks GTF_0_TXOUTCLK_3] \
                            [get_clocks GTF_0_RXOUTCLK_0] \
                            [get_clocks GTF_0_RXOUTCLK_1] \
                            [get_clocks GTF_0_RXOUTCLK_2] \
                            [get_clocks GTF_0_RXOUTCLK_3] ]

set_false_path  -from [get_clocks GTF_0_TXOUTCLK_2] \
                -to   [list [get_clocks -of_objects [get_pins clk_wiz_300_to_161_inst/inst/mmcme4_adv_inst/CLKOUT0]] \
                            [get_clocks -of_objects [get_pins clk_wiz_300_to_161_inst/inst/mmcme4_adv_inst/CLKOUT1]] \
                            [get_clocks GTF_0_TXOUTCLK_0] \
                            [get_clocks GTF_0_TXOUTCLK_1] \
                            [get_clocks GTF_0_TXOUTCLK_3] \
                            [get_clocks GTF_0_RXOUTCLK_0] \
                            [get_clocks GTF_0_RXOUTCLK_1] \
                            [get_clocks GTF_0_RXOUTCLK_2] \
                            [get_clocks GTF_0_RXOUTCLK_3] ]

set_false_path  -from [get_clocks GTF_0_TXOUTCLK_3] \
                -to   [list [get_clocks -of_objects [get_pins clk_wiz_300_to_161_inst/inst/mmcme4_adv_inst/CLKOUT0]] \
                            [get_clocks -of_objects [get_pins clk_wiz_300_to_161_inst/inst/mmcme4_adv_inst/CLKOUT1]] \
                            [get_clocks GTF_0_TXOUTCLK_0] \
                            [get_clocks GTF_0_TXOUTCLK_1] \
                            [get_clocks GTF_0_TXOUTCLK_2] \
                            [get_clocks GTF_0_RXOUTCLK_0] \
                            [get_clocks GTF_0_RXOUTCLK_1] \
                            [get_clocks GTF_0_RXOUTCLK_2] \
                            [get_clocks GTF_0_RXOUTCLK_3] ]

set_false_path  -from [get_clocks GTF_0_RXOUTCLK_0] \
                -to   [list [get_clocks -of_objects [get_pins clk_wiz_300_to_161_inst/inst/mmcme4_adv_inst/CLKOUT0]] \
                            [get_clocks -of_objects [get_pins clk_wiz_300_to_161_inst/inst/mmcme4_adv_inst/CLKOUT1]] \
                            [get_clocks GTF_0_TXOUTCLK_0] \
                            [get_clocks GTF_0_TXOUTCLK_1] \
                            [get_clocks GTF_0_TXOUTCLK_2] \
                            [get_clocks GTF_0_TXOUTCLK_3] \
                            [get_clocks GTF_0_RXOUTCLK_1] \
                            [get_clocks GTF_0_RXOUTCLK_2] \
                            [get_clocks GTF_0_RXOUTCLK_3] ]

set_false_path  -from [get_clocks GTF_0_RXOUTCLK_1] \
                -to   [list [get_clocks -of_objects [get_pins clk_wiz_300_to_161_inst/inst/mmcme4_adv_inst/CLKOUT0]] \
                            [get_clocks -of_objects [get_pins clk_wiz_300_to_161_inst/inst/mmcme4_adv_inst/CLKOUT1]] \
                            [get_clocks GTF_0_TXOUTCLK_0] \
                            [get_clocks GTF_0_TXOUTCLK_1] \
                            [get_clocks GTF_0_TXOUTCLK_2] \
                            [get_clocks GTF_0_TXOUTCLK_3] \
                            [get_clocks GTF_0_RXOUTCLK_0] \
                            [get_clocks GTF_0_RXOUTCLK_2] \
                            [get_clocks GTF_0_RXOUTCLK_3] ]

set_false_path  -from [get_clocks GTF_0_RXOUTCLK_2] \
                -to   [list [get_clocks -of_objects [get_pins clk_wiz_300_to_161_inst/inst/mmcme4_adv_inst/CLKOUT0]] \
                            [get_clocks -of_objects [get_pins clk_wiz_300_to_161_inst/inst/mmcme4_adv_inst/CLKOUT1]] \
                            [get_clocks GTF_0_TXOUTCLK_0] \
                            [get_clocks GTF_0_TXOUTCLK_1] \
                            [get_clocks GTF_0_TXOUTCLK_2] \
                            [get_clocks GTF_0_TXOUTCLK_3] \
                            [get_clocks GTF_0_RXOUTCLK_0] \
                            [get_clocks GTF_0_RXOUTCLK_1] \
                            [get_clocks GTF_0_RXOUTCLK_3] ]

set_false_path  -from [get_clocks GTF_0_RXOUTCLK_3] \
                -to   [list [get_clocks -of_objects [get_pins clk_wiz_300_to_161_inst/inst/mmcme4_adv_inst/CLKOUT0]] \
                            [get_clocks -of_objects [get_pins clk_wiz_300_to_161_inst/inst/mmcme4_adv_inst/CLKOUT1]] \
                            [get_clocks GTF_0_TXOUTCLK_0] \
                            [get_clocks GTF_0_TXOUTCLK_1] \
                            [get_clocks GTF_0_TXOUTCLK_2] \
                            [get_clocks GTF_0_TXOUTCLK_3] \
                            [get_clocks GTF_0_RXOUTCLK_0] \
                            [get_clocks GTF_0_RXOUTCLK_1] \
                            [get_clocks GTF_0_RXOUTCLK_2] ]


################################################################################
#
#  GTF Port 1 (RAW, gtf_top_1, Bank 127) generated clocks and async groups
#
################################################################################

create_generated_clock -name GTF_1_RXOUTCLK_0 \
                       -source        [get_pins gtf_top_1/u_gtfwizard_0_example_gtfmac_top/i_gtfmac/IBUFDS_GTE4_INST/O] \
                       -multiply_by 4 [get_pins gtf_top_1/u_gtfwizard_0_example_gtfmac_top/i_gtfmac/gen_blk_multi_ch[0].u_gtf_wiz_ip_top/inst/gtf_channel_inst/gtf_channel_inst/RXOUTCLK]

create_generated_clock -name GTF_1_TXOUTCLK_0 \
                       -source        [get_pins gtf_top_1/u_gtfwizard_0_example_gtfmac_top/i_gtfmac/IBUFDS_GTE4_INST/O] \
                       -multiply_by 4 [get_pins gtf_top_1/u_gtfwizard_0_example_gtfmac_top/i_gtfmac/gen_blk_multi_ch[0].u_gtf_wiz_ip_top/inst/gtf_channel_inst/gtf_channel_inst/TXOUTCLK]

create_generated_clock -name GTF_1_RXOUTCLK_1 \
                       -source        [get_pins gtf_top_1/u_gtfwizard_0_example_gtfmac_top/i_gtfmac/IBUFDS_GTE4_INST/O] \
                       -multiply_by 4 [get_pins gtf_top_1/u_gtfwizard_0_example_gtfmac_top/i_gtfmac/gen_blk_multi_ch[1].u_gtf_wiz_ip_top/inst/gtf_channel_inst/gtf_channel_inst/RXOUTCLK]

create_generated_clock -name GTF_1_TXOUTCLK_1 \
                       -source        [get_pins gtf_top_1/u_gtfwizard_0_example_gtfmac_top/i_gtfmac/IBUFDS_GTE4_INST/O] \
                       -multiply_by 4 [get_pins gtf_top_1/u_gtfwizard_0_example_gtfmac_top/i_gtfmac/gen_blk_multi_ch[1].u_gtf_wiz_ip_top/inst/gtf_channel_inst/gtf_channel_inst/TXOUTCLK]

create_generated_clock -name GTF_1_RXOUTCLK_2 \
                       -source        [get_pins gtf_top_1/u_gtfwizard_0_example_gtfmac_top/i_gtfmac/IBUFDS_GTE4_INST/O] \
                       -multiply_by 4 [get_pins gtf_top_1/u_gtfwizard_0_example_gtfmac_top/i_gtfmac/gen_blk_multi_ch[2].u_gtf_wiz_ip_top/inst/gtf_channel_inst/gtf_channel_inst/RXOUTCLK]

create_generated_clock -name GTF_1_TXOUTCLK_2 \
                       -source        [get_pins gtf_top_1/u_gtfwizard_0_example_gtfmac_top/i_gtfmac/IBUFDS_GTE4_INST/O] \
                       -multiply_by 4 [get_pins gtf_top_1/u_gtfwizard_0_example_gtfmac_top/i_gtfmac/gen_blk_multi_ch[2].u_gtf_wiz_ip_top/inst/gtf_channel_inst/gtf_channel_inst/TXOUTCLK]

create_generated_clock -name GTF_1_RXOUTCLK_3 \
                       -source        [get_pins gtf_top_1/u_gtfwizard_0_example_gtfmac_top/i_gtfmac/IBUFDS_GTE4_INST/O] \
                       -multiply_by 4 [get_pins gtf_top_1/u_gtfwizard_0_example_gtfmac_top/i_gtfmac/gen_blk_multi_ch[3].u_gtf_wiz_ip_top/inst/gtf_channel_inst/gtf_channel_inst/RXOUTCLK]

create_generated_clock -name GTF_1_TXOUTCLK_3 \
                       -source        [get_pins gtf_top_1/u_gtfwizard_0_example_gtfmac_top/i_gtfmac/IBUFDS_GTE4_INST/O] \
                       -multiply_by 4 [get_pins gtf_top_1/u_gtfwizard_0_example_gtfmac_top/i_gtfmac/gen_blk_multi_ch[3].u_gtf_wiz_ip_top/inst/gtf_channel_inst/gtf_channel_inst/TXOUTCLK]


# clk_wiz_300_to_161_inst CLKOUT0/1 (gtf_freerun_clk + 425MHz) <-> GTF_1 outclks
set_false_path  -from [get_clocks -of_objects [get_pins clk_wiz_300_to_161_inst/inst/mmcme4_adv_inst/CLKOUT0]] \
                -to   [list [get_clocks -of_objects [get_pins clk_wiz_300_to_161_inst/inst/mmcme4_adv_inst/CLKOUT1]] \
                            [get_clocks GTF_1_RXOUTCLK_0] \
                            [get_clocks GTF_1_RXOUTCLK_1] \
                            [get_clocks GTF_1_RXOUTCLK_2] \
                            [get_clocks GTF_1_RXOUTCLK_3] \
                            [get_clocks GTF_1_TXOUTCLK_0] \
                            [get_clocks GTF_1_TXOUTCLK_1] \
                            [get_clocks GTF_1_TXOUTCLK_2] \
                            [get_clocks GTF_1_TXOUTCLK_3] ]

set_false_path  -from [get_clocks -of_objects [get_pins clk_wiz_300_to_161_inst/inst/mmcme4_adv_inst/CLKOUT1]] \
                -to   [list [get_clocks -of_objects [get_pins clk_wiz_300_to_161_inst/inst/mmcme4_adv_inst/CLKOUT0]] \
                            [get_clocks GTF_1_RXOUTCLK_0] \
                            [get_clocks GTF_1_RXOUTCLK_1] \
                            [get_clocks GTF_1_RXOUTCLK_2] \
                            [get_clocks GTF_1_RXOUTCLK_3] \
                            [get_clocks GTF_1_TXOUTCLK_0] \
                            [get_clocks GTF_1_TXOUTCLK_1] \
                            [get_clocks GTF_1_TXOUTCLK_2] \
                            [get_clocks GTF_1_TXOUTCLK_3] ]

# Each GTF_1 channel TXOUTCLK/RXOUTCLK is async w.r.t. every other GTF_1
# channel and the freerun MMCM outputs.

set_false_path  -from [get_clocks GTF_1_TXOUTCLK_0] \
                -to   [list [get_clocks -of_objects [get_pins clk_wiz_300_to_161_inst/inst/mmcme4_adv_inst/CLKOUT0]] \
                            [get_clocks -of_objects [get_pins clk_wiz_300_to_161_inst/inst/mmcme4_adv_inst/CLKOUT1]] \
                            [get_clocks GTF_1_TXOUTCLK_1] \
                            [get_clocks GTF_1_TXOUTCLK_2] \
                            [get_clocks GTF_1_TXOUTCLK_3] \
                            [get_clocks GTF_1_RXOUTCLK_0] \
                            [get_clocks GTF_1_RXOUTCLK_1] \
                            [get_clocks GTF_1_RXOUTCLK_2] \
                            [get_clocks GTF_1_RXOUTCLK_3] ]

set_false_path  -from [get_clocks GTF_1_TXOUTCLK_1] \
                -to   [list [get_clocks -of_objects [get_pins clk_wiz_300_to_161_inst/inst/mmcme4_adv_inst/CLKOUT0]] \
                            [get_clocks -of_objects [get_pins clk_wiz_300_to_161_inst/inst/mmcme4_adv_inst/CLKOUT1]] \
                            [get_clocks GTF_1_TXOUTCLK_0] \
                            [get_clocks GTF_1_TXOUTCLK_2] \
                            [get_clocks GTF_1_TXOUTCLK_3] \
                            [get_clocks GTF_1_RXOUTCLK_0] \
                            [get_clocks GTF_1_RXOUTCLK_1] \
                            [get_clocks GTF_1_RXOUTCLK_2] \
                            [get_clocks GTF_1_RXOUTCLK_3] ]

set_false_path  -from [get_clocks GTF_1_TXOUTCLK_2] \
                -to   [list [get_clocks -of_objects [get_pins clk_wiz_300_to_161_inst/inst/mmcme4_adv_inst/CLKOUT0]] \
                            [get_clocks -of_objects [get_pins clk_wiz_300_to_161_inst/inst/mmcme4_adv_inst/CLKOUT1]] \
                            [get_clocks GTF_1_TXOUTCLK_0] \
                            [get_clocks GTF_1_TXOUTCLK_1] \
                            [get_clocks GTF_1_TXOUTCLK_3] \
                            [get_clocks GTF_1_RXOUTCLK_0] \
                            [get_clocks GTF_1_RXOUTCLK_1] \
                            [get_clocks GTF_1_RXOUTCLK_2] \
                            [get_clocks GTF_1_RXOUTCLK_3] ]

set_false_path  -from [get_clocks GTF_1_TXOUTCLK_3] \
                -to   [list [get_clocks -of_objects [get_pins clk_wiz_300_to_161_inst/inst/mmcme4_adv_inst/CLKOUT0]] \
                            [get_clocks -of_objects [get_pins clk_wiz_300_to_161_inst/inst/mmcme4_adv_inst/CLKOUT1]] \
                            [get_clocks GTF_1_TXOUTCLK_0] \
                            [get_clocks GTF_1_TXOUTCLK_1] \
                            [get_clocks GTF_1_TXOUTCLK_2] \
                            [get_clocks GTF_1_RXOUTCLK_0] \
                            [get_clocks GTF_1_RXOUTCLK_1] \
                            [get_clocks GTF_1_RXOUTCLK_2] \
                            [get_clocks GTF_1_RXOUTCLK_3] ]

set_false_path  -from [get_clocks GTF_1_RXOUTCLK_0] \
                -to   [list [get_clocks -of_objects [get_pins clk_wiz_300_to_161_inst/inst/mmcme4_adv_inst/CLKOUT0]] \
                            [get_clocks -of_objects [get_pins clk_wiz_300_to_161_inst/inst/mmcme4_adv_inst/CLKOUT1]] \
                            [get_clocks GTF_1_TXOUTCLK_0] \
                            [get_clocks GTF_1_TXOUTCLK_1] \
                            [get_clocks GTF_1_TXOUTCLK_2] \
                            [get_clocks GTF_1_TXOUTCLK_3] \
                            [get_clocks GTF_1_RXOUTCLK_1] \
                            [get_clocks GTF_1_RXOUTCLK_2] \
                            [get_clocks GTF_1_RXOUTCLK_3] ]

set_false_path  -from [get_clocks GTF_1_RXOUTCLK_1] \
                -to   [list [get_clocks -of_objects [get_pins clk_wiz_300_to_161_inst/inst/mmcme4_adv_inst/CLKOUT0]] \
                            [get_clocks -of_objects [get_pins clk_wiz_300_to_161_inst/inst/mmcme4_adv_inst/CLKOUT1]] \
                            [get_clocks GTF_1_TXOUTCLK_0] \
                            [get_clocks GTF_1_TXOUTCLK_1] \
                            [get_clocks GTF_1_TXOUTCLK_2] \
                            [get_clocks GTF_1_TXOUTCLK_3] \
                            [get_clocks GTF_1_RXOUTCLK_0] \
                            [get_clocks GTF_1_RXOUTCLK_2] \
                            [get_clocks GTF_1_RXOUTCLK_3] ]

set_false_path  -from [get_clocks GTF_1_RXOUTCLK_2] \
                -to   [list [get_clocks -of_objects [get_pins clk_wiz_300_to_161_inst/inst/mmcme4_adv_inst/CLKOUT0]] \
                            [get_clocks -of_objects [get_pins clk_wiz_300_to_161_inst/inst/mmcme4_adv_inst/CLKOUT1]] \
                            [get_clocks GTF_1_TXOUTCLK_0] \
                            [get_clocks GTF_1_TXOUTCLK_1] \
                            [get_clocks GTF_1_TXOUTCLK_2] \
                            [get_clocks GTF_1_TXOUTCLK_3] \
                            [get_clocks GTF_1_RXOUTCLK_0] \
                            [get_clocks GTF_1_RXOUTCLK_1] \
                            [get_clocks GTF_1_RXOUTCLK_3] ]

set_false_path  -from [get_clocks GTF_1_RXOUTCLK_3] \
                -to   [list [get_clocks -of_objects [get_pins clk_wiz_300_to_161_inst/inst/mmcme4_adv_inst/CLKOUT0]] \
                            [get_clocks -of_objects [get_pins clk_wiz_300_to_161_inst/inst/mmcme4_adv_inst/CLKOUT1]] \
                            [get_clocks GTF_1_TXOUTCLK_0] \
                            [get_clocks GTF_1_TXOUTCLK_1] \
                            [get_clocks GTF_1_TXOUTCLK_2] \
                            [get_clocks GTF_1_TXOUTCLK_3] \
                            [get_clocks GTF_1_RXOUTCLK_0] \
                            [get_clocks GTF_1_RXOUTCLK_1] \
                            [get_clocks GTF_1_RXOUTCLK_2] ]

# create_generated_clock -name GTF_1_TXOUTCLK_0 \
#                        [get_pins gtf_top_1/gtfwizard_raw_example_top/gen_blk_multi_ch[0].u_gtf_wiz_ip_top/inst/gtf_channel_inst/gtf_channel_inst/TXOUTCLK]

# create_generated_clock -name GTF_1_TXOUTCLK_1 \
#                        [get_pins gtf_top_1/gtfwizard_raw_example_top/gen_blk_multi_ch[1].u_gtf_wiz_ip_top/inst/gtf_channel_inst/gtf_channel_inst/TXOUTCLK]

# create_generated_clock -name GTF_1_TXOUTCLK_2 \
#                        [get_pins gtf_top_1/gtfwizard_raw_example_top/gen_blk_multi_ch[2].u_gtf_wiz_ip_top/inst/gtf_channel_inst/gtf_channel_inst/TXOUTCLK]

# create_generated_clock -name GTF_1_TXOUTCLK_3 \
#                        [get_pins gtf_top_1/gtfwizard_raw_example_top/gen_blk_multi_ch[3].u_gtf_wiz_ip_top/inst/gtf_channel_inst/gtf_channel_inst/TXOUTCLK]

# create_generated_clock -name GTF_1_RXOUTCLK_0 \
#                        [get_pins gtf_top_1/gtfwizard_raw_example_top/gen_blk_multi_ch[0].u_gtf_wiz_ip_top/inst/gtf_channel_inst/gtf_channel_inst/RXOUTCLK]

# create_generated_clock -name GTF_1_RXOUTCLK_1 \
#                        [get_pins gtf_top_1/gtfwizard_raw_example_top/gen_blk_multi_ch[1].u_gtf_wiz_ip_top/inst/gtf_channel_inst/gtf_channel_inst/RXOUTCLK]

# create_generated_clock -name GTF_1_RXOUTCLK_2 \
#                        [get_pins gtf_top_1/gtfwizard_raw_example_top/gen_blk_multi_ch[2].u_gtf_wiz_ip_top/inst/gtf_channel_inst/gtf_channel_inst/RXOUTCLK]

# create_generated_clock -name GTF_1_RXOUTCLK_3 \
#                        [get_pins gtf_top_1/gtfwizard_raw_example_top/gen_blk_multi_ch[3].u_gtf_wiz_ip_top/inst/gtf_channel_inst/gtf_channel_inst/RXOUTCLK]


# set_false_path  -from [get_clocks -of_objects [get_pins clk_wiz_300_to_161_inst/inst/mmcme4_adv_inst/CLKOUT0]] \
#                 -to   [list [get_clocks -of_objects [get_pins clk_wiz_300_to_161_inst/inst/mmcme4_adv_inst/CLKOUT1]] \
#                             [get_clocks GTF_1_TXOUTCLK_0] \
#                             [get_clocks GTF_1_TXOUTCLK_1] \
#                             [get_clocks GTF_1_TXOUTCLK_2] \
#                             [get_clocks GTF_1_TXOUTCLK_3] \
#                             [get_clocks GTF_1_RXOUTCLK_0] \
#                             [get_clocks GTF_1_RXOUTCLK_1] \
#                             [get_clocks GTF_1_RXOUTCLK_2] \
#                             [get_clocks GTF_1_RXOUTCLK_3] ]

# set_false_path  -from [get_clocks -of_objects [get_pins clk_wiz_300_to_161_inst/inst/mmcme4_adv_inst/CLKOUT1]] \
#                 -to   [list [get_clocks -of_objects [get_pins clk_wiz_300_to_161_inst/inst/mmcme4_adv_inst/CLKOUT0]] \
#                             [get_clocks GTF_1_TXOUTCLK_0] \
#                             [get_clocks GTF_1_TXOUTCLK_1] \
#                             [get_clocks GTF_1_TXOUTCLK_2] \
#                             [get_clocks GTF_1_TXOUTCLK_3] \
#                             [get_clocks GTF_1_RXOUTCLK_0] \
#                             [get_clocks GTF_1_RXOUTCLK_1] \
#                             [get_clocks GTF_1_RXOUTCLK_2] \
#                             [get_clocks GTF_1_RXOUTCLK_3] ]

# set_false_path  -from [get_clocks GTF_1_TXOUTCLK_0] \
#                 -to   [list [get_clocks GTF_1_TXOUTCLK_1] \
#                             [get_clocks GTF_1_TXOUTCLK_2] \
#                             [get_clocks GTF_1_TXOUTCLK_3] \
#                             [get_clocks GTF_1_RXOUTCLK_0] \
#                             [get_clocks GTF_1_RXOUTCLK_1] \
#                             [get_clocks GTF_1_RXOUTCLK_2] \
#                             [get_clocks GTF_1_RXOUTCLK_3] \
#                             [get_clocks -of_objects [get_pins clk_wiz_300_to_161_inst/inst/mmcme4_adv_inst/CLKOUT0]] \
#                             [get_clocks -of_objects [get_pins clk_wiz_300_to_161_inst/inst/mmcme4_adv_inst/CLKOUT1]] ]

# set_false_path  -from [get_clocks GTF_1_TXOUTCLK_1] \
#                 -to   [list [get_clocks GTF_1_TXOUTCLK_0] \
#                             [get_clocks GTF_1_TXOUTCLK_2] \
#                             [get_clocks GTF_1_TXOUTCLK_3] \
#                             [get_clocks GTF_1_RXOUTCLK_0] \
#                             [get_clocks GTF_1_RXOUTCLK_1] \
#                             [get_clocks GTF_1_RXOUTCLK_2] \
#                             [get_clocks GTF_1_RXOUTCLK_3] \
#                             [get_clocks -of_objects [get_pins clk_wiz_300_to_161_inst/inst/mmcme4_adv_inst/CLKOUT0]] \
#                             [get_clocks -of_objects [get_pins clk_wiz_300_to_161_inst/inst/mmcme4_adv_inst/CLKOUT1]] ]

# set_false_path  -from [get_clocks GTF_1_TXOUTCLK_2] \
#                 -to   [list [get_clocks GTF_1_TXOUTCLK_0] \
#                             [get_clocks GTF_1_TXOUTCLK_1] \
#                             [get_clocks GTF_1_TXOUTCLK_3] \
#                             [get_clocks GTF_1_RXOUTCLK_0] \
#                             [get_clocks GTF_1_RXOUTCLK_1] \
#                             [get_clocks GTF_1_RXOUTCLK_2] \
#                             [get_clocks GTF_1_RXOUTCLK_3] \
#                             [get_clocks -of_objects [get_pins clk_wiz_300_to_161_inst/inst/mmcme4_adv_inst/CLKOUT0]] \
#                             [get_clocks -of_objects [get_pins clk_wiz_300_to_161_inst/inst/mmcme4_adv_inst/CLKOUT1]] ]

# set_false_path  -from [get_clocks GTF_1_TXOUTCLK_3] \
#                 -to   [list [get_clocks GTF_1_TXOUTCLK_0] \
#                             [get_clocks GTF_1_TXOUTCLK_1] \
#                             [get_clocks GTF_1_TXOUTCLK_2] \
#                             [get_clocks GTF_1_RXOUTCLK_0] \
#                             [get_clocks GTF_1_RXOUTCLK_1] \
#                             [get_clocks GTF_1_RXOUTCLK_2] \
#                             [get_clocks GTF_1_RXOUTCLK_3] \
#                             [get_clocks -of_objects [get_pins clk_wiz_300_to_161_inst/inst/mmcme4_adv_inst/CLKOUT0]] \
#                             [get_clocks -of_objects [get_pins clk_wiz_300_to_161_inst/inst/mmcme4_adv_inst/CLKOUT1]] ]

# set_false_path  -from [get_clocks GTF_1_RXOUTCLK_0] \
#                 -to   [list [get_clocks GTF_1_TXOUTCLK_0] \
#                             [get_clocks GTF_1_TXOUTCLK_1] \
#                             [get_clocks GTF_1_TXOUTCLK_2] \
#                             [get_clocks GTF_1_TXOUTCLK_3] \
#                             [get_clocks GTF_1_RXOUTCLK_1] \
#                             [get_clocks GTF_1_RXOUTCLK_2] \
#                             [get_clocks GTF_1_RXOUTCLK_3] \
#                             [get_clocks -of_objects [get_pins clk_wiz_300_to_161_inst/inst/mmcme4_adv_inst/CLKOUT0]] \
#                             [get_clocks -of_objects [get_pins clk_wiz_300_to_161_inst/inst/mmcme4_adv_inst/CLKOUT1]] ]

# set_false_path  -from [get_clocks GTF_1_RXOUTCLK_1] \
#                 -to   [list [get_clocks GTF_1_TXOUTCLK_0] \
#                             [get_clocks GTF_1_TXOUTCLK_1] \
#                             [get_clocks GTF_1_TXOUTCLK_2] \
#                             [get_clocks GTF_1_TXOUTCLK_3] \
#                             [get_clocks GTF_1_RXOUTCLK_0] \
#                             [get_clocks GTF_1_RXOUTCLK_2] \
#                             [get_clocks GTF_1_RXOUTCLK_3] \
#                             [get_clocks -of_objects [get_pins clk_wiz_300_to_161_inst/inst/mmcme4_adv_inst/CLKOUT0]] \
#                             [get_clocks -of_objects [get_pins clk_wiz_300_to_161_inst/inst/mmcme4_adv_inst/CLKOUT1]] ]

# set_false_path  -from [get_clocks GTF_1_RXOUTCLK_2] \
#                 -to   [list [get_clocks GTF_1_TXOUTCLK_0] \
#                             [get_clocks GTF_1_TXOUTCLK_1] \
#                             [get_clocks GTF_1_TXOUTCLK_2] \
#                             [get_clocks GTF_1_TXOUTCLK_3] \
#                             [get_clocks GTF_1_RXOUTCLK_0] \
#                             [get_clocks GTF_1_RXOUTCLK_1] \
#                             [get_clocks GTF_1_RXOUTCLK_3] \
#                             [get_clocks -of_objects [get_pins clk_wiz_300_to_161_inst/inst/mmcme4_adv_inst/CLKOUT0]] \
#                             [get_clocks -of_objects [get_pins clk_wiz_300_to_161_inst/inst/mmcme4_adv_inst/CLKOUT1]] ]

# set_false_path  -from [get_clocks GTF_1_RXOUTCLK_3] \
#                 -to   [list [get_clocks GTF_1_TXOUTCLK_0] \
#                             [get_clocks GTF_1_TXOUTCLK_1] \
#                             [get_clocks GTF_1_TXOUTCLK_2] \
#                             [get_clocks GTF_1_TXOUTCLK_3] \
#                             [get_clocks GTF_1_RXOUTCLK_0] \
#                             [get_clocks GTF_1_RXOUTCLK_1] \
#                             [get_clocks GTF_1_RXOUTCLK_2] \
#                             [get_clocks -of_objects [get_pins clk_wiz_300_to_161_inst/inst/mmcme4_adv_inst/CLKOUT0]] \
#                             [get_clocks -of_objects [get_pins clk_wiz_300_to_161_inst/inst/mmcme4_adv_inst/CLKOUT1]] ]


################################################################################
#
#  GTF MAC delay_powergood internal-reset synchronizer false paths
#
################################################################################

set_false_path -from [get_pins -hierarchical -filter { NAME =~  "gtf_top_0/u_gtfwizard_0_example_gtfmac_top/i_gtfmac/gen_blk_multi_ch[*].u_gtf_wiz_ip_top/inst/delay_powergood_inst/gen_powergood_delay.intclk_rrst_n_r_reg[*]/C" } ] \
               -to   [get_pins -hierarchical -filter { NAME =~  "gtf_top_0/u_gtfwizard_0_example_gtfmac_top/i_gtfmac/gen_blk_multi_ch[*].u_gtf_wiz_ip_top/inst/delay_powergood_inst/gen_powergood_delay.wait_cnt_reg[*]/R" } ]

set_false_path -from [get_pins -hierarchical -filter { NAME =~  "gtf_top_0/u_gtfwizard_0_example_gtfmac_top/i_gtfmac/gen_blk_multi_ch[*].u_gtf_wiz_ip_top/inst/delay_powergood_inst/gen_powergood_delay.intclk_rrst_n_r_reg[*]/C" } ] \
               -to   [get_pins -hierarchical -filter { NAME =~  "gtf_top_0/u_gtfwizard_0_example_gtfmac_top/i_gtfmac/gen_blk_multi_ch[*].u_gtf_wiz_ip_top/inst/delay_powergood_inst/gen_powergood_delay.wait_cnt_reg[*]/D" } ]

set_false_path -from [get_pins -hierarchical -filter { NAME =~  "gtf_top_0/u_gtfwizard_0_example_gtfmac_top/i_gtfmac/gen_blk_multi_ch[*].u_gtf_wiz_ip_top/inst/delay_powergood_inst/gen_powergood_delay.intclk_rrst_n_r_reg[*]/C" } ] \
               -to   [get_pins -hierarchical -filter { NAME =~  "gtf_top_0/u_gtfwizard_0_example_gtfmac_top/i_gtfmac/gen_blk_multi_ch[*].u_gtf_wiz_ip_top/inst/delay_powergood_inst/gen_powergood_delay.wait_cnt_reg[*]/CE" } ]

set_false_path -from [get_pins -hierarchical -filter { NAME =~  "gtf_top_0/u_gtfwizard_0_example_gtfmac_top/i_gtfmac/gen_blk_multi_ch[*].u_gtf_wiz_ip_top/inst/delay_powergood_inst/gen_powergood_delay.int_pwr_on_fsm_reg/C" } ] \
               -to   [get_pins -hierarchical -filter { NAME =~  "gtf_top_0/u_gtfwizard_0_example_gtfmac_top/i_gtfmac/gen_blk_multi_ch[*].u_gtf_wiz_ip_top/inst/delay_powergood_inst/gen_powergood_delay.intclk_rrst_n_r_reg[*]/CE" } ]

set_false_path -from [get_pins -hierarchical -filter { NAME =~  "gtf_top_0/u_gtfwizard_0_example_gtfmac_top/i_gtfmac/gen_blk_multi_ch[*].u_gtf_wiz_ip_top/inst/delay_powergood_inst/gen_powergood_delay.int_pwr_on_fsm_reg/C" } ] \
               -to   [get_pins -hierarchical -filter { NAME =~  "gtf_top_0/u_gtfwizard_0_example_gtfmac_top/i_gtfmac/gen_blk_multi_ch[*].u_gtf_wiz_ip_top/inst/delay_powergood_inst/gen_powergood_delay.wait_cnt_reg[*]/CE" } ]

set_false_path -from [get_pins -hierarchical -filter { NAME =~  "gtf_top_0/u_gtfwizard_0_example_gtfmac_top/i_gtfmac/gen_blk_multi_ch[*].u_gtf_wiz_ip_top/inst/delay_powergood_inst/gen_powergood_delay.wait_cnt_reg[*]/C" } ] \
               -to   [get_pins -hierarchical -filter { NAME =~  "gtf_top_0/u_gtfwizard_0_example_gtfmac_top/i_gtfmac/gen_blk_multi_ch[*].u_gtf_wiz_ip_top/inst/delay_powergood_inst/gen_powergood_delay.wait_cnt_reg[*]/D" } ]

set_false_path -from [get_pins -hierarchical -filter { NAME =~  "gtf_top_0/u_gtfwizard_0_example_gtfmac_top/i_gtfmac/gen_blk_multi_ch[*].u_gtf_wiz_ip_top/inst/delay_powergood_inst/gen_powergood_delay.wait_cnt_reg[*]/C" } ] \
               -to   [get_pins -hierarchical -filter { NAME =~  "gtf_top_0/u_gtfwizard_0_example_gtfmac_top/i_gtfmac/gen_blk_multi_ch[*].u_gtf_wiz_ip_top/inst/delay_powergood_inst/gen_powergood_delay.int_pwr_on_fsm_reg/D" } ]

set_false_path -from [get_pins -hierarchical -filter { NAME =~  "gtf_top_0/u_gtfwizard_0_example_gtfmac_top/i_gtfmac/gen_blk_multi_ch[*].u_gtf_wiz_ip_top/inst/delay_powergood_inst/gen_powergood_delay.int_pwr_on_fsm_reg/C" } ] \
               -to   [get_pins -hierarchical -filter { NAME =~  "gtf_top_0/u_gtfwizard_0_example_gtfmac_top/i_gtfmac/gen_blk_multi_ch[*].u_gtf_wiz_ip_top/inst/delay_powergood_inst/gen_powergood_delay.pwr_on_fsm_reg/D" } ]

# double mac
set_false_path -from [get_pins -hierarchical -filter { NAME =~  "gtf_top_1/u_gtfwizard_0_example_gtfmac_top/i_gtfmac/gen_blk_multi_ch[*].u_gtf_wiz_ip_top/inst/delay_powergood_inst/gen_powergood_delay.intclk_rrst_n_r_reg[*]/C" } ] \
               -to   [get_pins -hierarchical -filter { NAME =~  "gtf_top_1/u_gtfwizard_0_example_gtfmac_top/i_gtfmac/gen_blk_multi_ch[*].u_gtf_wiz_ip_top/inst/delay_powergood_inst/gen_powergood_delay.wait_cnt_reg[*]/R" } ]

set_false_path -from [get_pins -hierarchical -filter { NAME =~  "gtf_top_1/u_gtfwizard_0_example_gtfmac_top/i_gtfmac/gen_blk_multi_ch[*].u_gtf_wiz_ip_top/inst/delay_powergood_inst/gen_powergood_delay.intclk_rrst_n_r_reg[*]/C" } ] \
               -to   [get_pins -hierarchical -filter { NAME =~  "gtf_top_1/u_gtfwizard_0_example_gtfmac_top/i_gtfmac/gen_blk_multi_ch[*].u_gtf_wiz_ip_top/inst/delay_powergood_inst/gen_powergood_delay.wait_cnt_reg[*]/D" } ]

set_false_path -from [get_pins -hierarchical -filter { NAME =~  "gtf_top_1/u_gtfwizard_0_example_gtfmac_top/i_gtfmac/gen_blk_multi_ch[*].u_gtf_wiz_ip_top/inst/delay_powergood_inst/gen_powergood_delay.intclk_rrst_n_r_reg[*]/C" } ] \
               -to   [get_pins -hierarchical -filter { NAME =~  "gtf_top_1/u_gtfwizard_0_example_gtfmac_top/i_gtfmac/gen_blk_multi_ch[*].u_gtf_wiz_ip_top/inst/delay_powergood_inst/gen_powergood_delay.wait_cnt_reg[*]/CE" } ]

set_false_path -from [get_pins -hierarchical -filter { NAME =~  "gtf_top_1/u_gtfwizard_0_example_gtfmac_top/i_gtfmac/gen_blk_multi_ch[*].u_gtf_wiz_ip_top/inst/delay_powergood_inst/gen_powergood_delay.int_pwr_on_fsm_reg/C" } ] \
               -to   [get_pins -hierarchical -filter { NAME =~  "gtf_top_1/u_gtfwizard_0_example_gtfmac_top/i_gtfmac/gen_blk_multi_ch[*].u_gtf_wiz_ip_top/inst/delay_powergood_inst/gen_powergood_delay.intclk_rrst_n_r_reg[*]/CE" } ]

set_false_path -from [get_pins -hierarchical -filter { NAME =~  "gtf_top_1/u_gtfwizard_0_example_gtfmac_top/i_gtfmac/gen_blk_multi_ch[*].u_gtf_wiz_ip_top/inst/delay_powergood_inst/gen_powergood_delay.int_pwr_on_fsm_reg/C" } ] \
               -to   [get_pins -hierarchical -filter { NAME =~  "gtf_top_1/u_gtfwizard_0_example_gtfmac_top/i_gtfmac/gen_blk_multi_ch[*].u_gtf_wiz_ip_top/inst/delay_powergood_inst/gen_powergood_delay.wait_cnt_reg[*]/CE" } ]

set_false_path -from [get_pins -hierarchical -filter { NAME =~  "gtf_top_1/u_gtfwizard_0_example_gtfmac_top/i_gtfmac/gen_blk_multi_ch[*].u_gtf_wiz_ip_top/inst/delay_powergood_inst/gen_powergood_delay.wait_cnt_reg[*]/C" } ] \
               -to   [get_pins -hierarchical -filter { NAME =~  "gtf_top_1/u_gtfwizard_0_example_gtfmac_top/i_gtfmac/gen_blk_multi_ch[*].u_gtf_wiz_ip_top/inst/delay_powergood_inst/gen_powergood_delay.wait_cnt_reg[*]/D" } ]

set_false_path -from [get_pins -hierarchical -filter { NAME =~  "gtf_top_1/u_gtfwizard_0_example_gtfmac_top/i_gtfmac/gen_blk_multi_ch[*].u_gtf_wiz_ip_top/inst/delay_powergood_inst/gen_powergood_delay.wait_cnt_reg[*]/C" } ] \
               -to   [get_pins -hierarchical -filter { NAME =~  "gtf_top_1/u_gtfwizard_0_example_gtfmac_top/i_gtfmac/gen_blk_multi_ch[*].u_gtf_wiz_ip_top/inst/delay_powergood_inst/gen_powergood_delay.int_pwr_on_fsm_reg/D" } ]

set_false_path -from [get_pins -hierarchical -filter { NAME =~  "gtf_top_1/u_gtfwizard_0_example_gtfmac_top/i_gtfmac/gen_blk_multi_ch[*].u_gtf_wiz_ip_top/inst/delay_powergood_inst/gen_powergood_delay.int_pwr_on_fsm_reg/C" } ] \
               -to   [get_pins -hierarchical -filter { NAME =~  "gtf_top_1/u_gtfwizard_0_example_gtfmac_top/i_gtfmac/gen_blk_multi_ch[*].u_gtf_wiz_ip_top/inst/delay_powergood_inst/gen_powergood_delay.pwr_on_fsm_reg/D" } ]


################################################################################
#
#  Frequency-monitor false paths -- system clk_wiz vs every async clock
#  (the freq counter is intentionally async to all of these)
#
################################################################################

set_false_path  -from [get_clocks -of_objects [get_pins clk_reset/clk_wiz_100mhz/inst/mmcme4_adv_inst/CLKOUT0]] \
                -to   [list [get_clocks synce_clk_127_p] \
                            [get_clocks synce_clk_128_p] \
                            [get_clocks synce_clk_129_p] \
                            [get_clocks synce_clk_130_p] \
                            [get_clocks synce_clk_131_p] \
                            [get_clocks synce_clk_65_1_p] \
                            [get_clocks synce_clk_65_2_p] \
                            [get_clocks GTF_0_RXOUTCLK_0] \
                            [get_clocks GTF_0_RXOUTCLK_1] \
                            [get_clocks GTF_0_RXOUTCLK_2] \
                            [get_clocks GTF_0_RXOUTCLK_3] \
                            [get_clocks GTF_0_TXOUTCLK_0] \
                            [get_clocks GTF_0_TXOUTCLK_1] \
                            [get_clocks GTF_0_TXOUTCLK_2] \
                            [get_clocks GTF_0_TXOUTCLK_3] \
                            [get_clocks GTF_1_RXOUTCLK_0] \
                            [get_clocks GTF_1_RXOUTCLK_1] \
                            [get_clocks GTF_1_RXOUTCLK_2] \
                            [get_clocks GTF_1_RXOUTCLK_3] \
                            [get_clocks GTF_1_TXOUTCLK_0] \
                            [get_clocks GTF_1_TXOUTCLK_1] \
                            [get_clocks GTF_1_TXOUTCLK_2] \
                            [get_clocks GTF_1_TXOUTCLK_3] \
                            [get_clocks -of_objects [get_pins {gtf_top_0/u_gtfwizard_0_example_gtfmac_top/i_gtfmac/gen_blk_multi_ch[0].u_bufg_gt_gtf_recov_clk/O}]] \
                            [get_clocks -of_objects [get_pins {gtf_top_1/u_gtfwizard_0_example_gtfmac_top/i_gtfmac/gen_blk_multi_ch[0].u_bufg_gt_gtf_recov_clk/O}]] ]

set_false_path  -from [get_clocks -of_objects [get_pins clk_reset/clk_wiz_100mhz/inst/mmcme4_adv_inst/CLKOUT1]] \
                -to   [list [get_clocks synce_clk_127_p] \
                            [get_clocks synce_clk_128_p] \
                            [get_clocks synce_clk_129_p] \
                            [get_clocks synce_clk_130_p] \
                            [get_clocks synce_clk_131_p] \
                            [get_clocks synce_clk_65_1_p] \
                            [get_clocks synce_clk_65_2_p] \
                            [get_clocks GTF_0_RXOUTCLK_0] \
                            [get_clocks GTF_0_RXOUTCLK_1] \
                            [get_clocks GTF_0_RXOUTCLK_2] \
                            [get_clocks GTF_0_RXOUTCLK_3] \
                            [get_clocks GTF_0_TXOUTCLK_0] \
                            [get_clocks GTF_0_TXOUTCLK_1] \
                            [get_clocks GTF_0_TXOUTCLK_2] \
                            [get_clocks GTF_0_TXOUTCLK_3] \
                            [get_clocks GTF_1_RXOUTCLK_0] \
                            [get_clocks GTF_1_RXOUTCLK_1] \
                            [get_clocks GTF_1_RXOUTCLK_2] \
                            [get_clocks GTF_1_RXOUTCLK_3] \
                            [get_clocks GTF_1_TXOUTCLK_0] \
                            [get_clocks GTF_1_TXOUTCLK_1] \
                            [get_clocks GTF_1_TXOUTCLK_2] \
                            [get_clocks GTF_1_TXOUTCLK_3] \
                            [get_clocks -of_objects [get_pins {gtf_top_0/u_gtfwizard_0_example_gtfmac_top/i_gtfmac/gen_blk_multi_ch[0].u_bufg_gt_gtf_recov_clk/O}]] \
                            [get_clocks -of_objects [get_pins {gtf_top_1/u_gtfwizard_0_example_gtfmac_top/i_gtfmac/gen_blk_multi_ch[0].u_bufg_gt_gtf_recov_clk/O}]] ]

set_false_path  -from [list [get_clocks synce_clk_127_p] \
                            [get_clocks synce_clk_128_p] \
                            [get_clocks synce_clk_129_p] \
                            [get_clocks synce_clk_130_p] \
                            [get_clocks synce_clk_131_p] \
                            [get_clocks synce_clk_65_1_p] \
                            [get_clocks synce_clk_65_2_p] \
                            [get_clocks GTF_0_RXOUTCLK_0] \
                            [get_clocks GTF_0_RXOUTCLK_1] \
                            [get_clocks GTF_0_RXOUTCLK_2] \
                            [get_clocks GTF_0_RXOUTCLK_3] \
                            [get_clocks GTF_0_TXOUTCLK_0] \
                            [get_clocks GTF_0_TXOUTCLK_1] \
                            [get_clocks GTF_0_TXOUTCLK_2] \
                            [get_clocks GTF_0_TXOUTCLK_3] \
                            [get_clocks GTF_1_RXOUTCLK_0] \
                            [get_clocks GTF_1_RXOUTCLK_1] \
                            [get_clocks GTF_1_RXOUTCLK_2] \
                            [get_clocks GTF_1_RXOUTCLK_3] \
                            [get_clocks GTF_1_TXOUTCLK_0] \
                            [get_clocks GTF_1_TXOUTCLK_1] \
                            [get_clocks GTF_1_TXOUTCLK_2] \
                            [get_clocks GTF_1_TXOUTCLK_3] \
                            [get_clocks -of_objects [get_pins {gtf_top_0/u_gtfwizard_0_example_gtfmac_top/i_gtfmac/gen_blk_multi_ch[0].u_bufg_gt_gtf_recov_clk/O}]] \
                            [get_clocks -of_objects [get_pins {gtf_top_1/u_gtfwizard_0_example_gtfmac_top/i_gtfmac/gen_blk_multi_ch[0].u_bufg_gt_gtf_recov_clk/O}]] ] \
                -to   [get_clocks -of_objects [get_pins clk_reset/clk_wiz_100mhz/inst/mmcme4_adv_inst/CLKOUT0]]

set_false_path  -from [list [get_clocks synce_clk_127_p] \
                            [get_clocks synce_clk_128_p] \
                            [get_clocks synce_clk_129_p] \
                            [get_clocks synce_clk_130_p] \
                            [get_clocks synce_clk_131_p] \
                            [get_clocks synce_clk_65_1_p] \
                            [get_clocks synce_clk_65_2_p] \
                            [get_clocks GTF_0_RXOUTCLK_0] \
                            [get_clocks GTF_0_RXOUTCLK_1] \
                            [get_clocks GTF_0_RXOUTCLK_2] \
                            [get_clocks GTF_0_RXOUTCLK_3] \
                            [get_clocks GTF_0_TXOUTCLK_0] \
                            [get_clocks GTF_0_TXOUTCLK_1] \
                            [get_clocks GTF_0_TXOUTCLK_2] \
                            [get_clocks GTF_0_TXOUTCLK_3] \
                            [get_clocks GTF_1_RXOUTCLK_0] \
                            [get_clocks GTF_1_RXOUTCLK_1] \
                            [get_clocks GTF_1_RXOUTCLK_2] \
                            [get_clocks GTF_1_RXOUTCLK_3] \
                            [get_clocks GTF_1_TXOUTCLK_0] \
                            [get_clocks GTF_1_TXOUTCLK_1] \
                            [get_clocks GTF_1_TXOUTCLK_2] \
                            [get_clocks GTF_1_TXOUTCLK_3] \
                            [get_clocks -of_objects [get_pins {gtf_top_0/u_gtfwizard_0_example_gtfmac_top/i_gtfmac/gen_blk_multi_ch[0].u_bufg_gt_gtf_recov_clk/O}]] \
                            [get_clocks -of_objects [get_pins {gtf_top_1/u_gtfwizard_0_example_gtfmac_top/i_gtfmac/gen_blk_multi_ch[0].u_bufg_gt_gtf_recov_clk/O}]] ] \
                -to   [get_clocks -of_objects [get_pins clk_reset/clk_wiz_100mhz/inst/mmcme4_adv_inst/CLKOUT1]]


################################################################################
#
#  GTF RAW reset synchronizer
#
################################################################################

# set_false_path -from [get_pins {gtf_top_1/gtfwizard_raw_example_top/gen_blk_multi_ch[0].reset_all_in_sync/arststages_ff_reg[4]/C}]
