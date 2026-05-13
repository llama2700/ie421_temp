set_property CONTROL.TRIGGER_CONDITION OR [get_hw_ilas -of_objects [get_hw_devices xcvu2p_0] -filter {CELL_NAME=~"gtf_top_0/u_gtfwizard_0_example_gtfmac_top/ILA_GEN[0].rx_mac_ila"}]
set_property CONTROL.TRIGGER_CONDITION OR [get_hw_ilas -of_objects [get_hw_devices xcvu2p_0] -filter {CELL_NAME=~"gtf_top_0/u_gtfwizard_0_example_gtfmac_top/ILA_GEN[0].tx_mac_ila"}]
set_property CONTROL.TRIGGER_CONDITION OR [get_hw_ilas -of_objects [get_hw_devices xcvu2p_0] -filter {CELL_NAME=~"gtf_top_1/u_gtfwizard_0_example_gtfmac_top/ILA_GEN[0].rx_mac_ila"}]
set_property CONTROL.TRIGGER_CONDITION OR [get_hw_ilas -of_objects [get_hw_devices xcvu2p_0] -filter {CELL_NAME=~"gtf_top_1/u_gtfwizard_0_example_gtfmac_top/ILA_GEN[0].tx_mac_ila"}]

set_property CONTROL.TRIGGER_POSITION 10 [get_hw_ilas -of_objects [get_hw_devices xcvu2p_0] -filter {CELL_NAME=~"gtf_top_0/u_gtfwizard_0_example_gtfmac_top/ILA_GEN[0].rx_mac_ila"}]
set_property CONTROL.TRIGGER_POSITION 10 [get_hw_ilas -of_objects [get_hw_devices xcvu2p_0] -filter {CELL_NAME=~"gtf_top_0/u_gtfwizard_0_example_gtfmac_top/ILA_GEN[0].tx_mac_ila"}]
set_property CONTROL.TRIGGER_POSITION 10 [get_hw_ilas -of_objects [get_hw_devices xcvu2p_0] -filter {CELL_NAME=~"gtf_top_1/u_gtfwizard_0_example_gtfmac_top/ILA_GEN[0].rx_mac_ila"}]
set_property CONTROL.TRIGGER_POSITION 10 [get_hw_ilas -of_objects [get_hw_devices xcvu2p_0] -filter {CELL_NAME=~"gtf_top_1/u_gtfwizard_0_example_gtfmac_top/ILA_GEN[0].tx_mac_ila"}]

set_property TRIGGER_COMPARE_VALUE eq1'b1 [get_hw_probes {gtf_top_0/u_gtfwizard_0_example_gtfmac_top/ILA_GEN[0].fifo_rx_axis_tvalid} -of_objects [get_hw_ilas -of_objects [get_hw_devices xcvu2p_0] -filter {CELL_NAME=~"gtf_top_0/u_gtfwizard_0_example_gtfmac_top/ILA_GEN[0].rx_mac_ila"}]]
set_property TRIGGER_COMPARE_VALUE eq1'b1 [get_hw_probes {gtf_top_0/u_gtfwizard_0_example_gtfmac_top/ILA_GEN[0].fifo_tx_axis_tvalid} -of_objects [get_hw_ilas -of_objects [get_hw_devices xcvu2p_0] -filter {CELL_NAME=~"gtf_top_0/u_gtfwizard_0_example_gtfmac_top/ILA_GEN[0].tx_mac_ila"}]]
set_property TRIGGER_COMPARE_VALUE eq1'b1 [get_hw_probes {gtf_top_1/u_gtfwizard_0_example_gtfmac_top/ILA_GEN[0].fifo_rx_axis_tvalid} -of_objects [get_hw_ilas -of_objects [get_hw_devices xcvu2p_0] -filter {CELL_NAME=~"gtf_top_1/u_gtfwizard_0_example_gtfmac_top/ILA_GEN[0].rx_mac_ila"}]]
set_property TRIGGER_COMPARE_VALUE eq1'b1 [get_hw_probes {gtf_top_1/u_gtfwizard_0_example_gtfmac_top/ILA_GEN[0].fifo_tx_axis_tvalid} -of_objects [get_hw_ilas -of_objects [get_hw_devices xcvu2p_0] -filter {CELL_NAME=~"gtf_top_1/u_gtfwizard_0_example_gtfmac_top/ILA_GEN[0].tx_mac_ila"}]]

run_hw_ila [get_hw_ilas -of_objects [get_hw_devices xcvu2p_0] -filter {CELL_NAME=~"gtf_top_0/u_gtfwizard_0_example_gtfmac_top/ILA_GEN[0].rx_mac_ila"}]
run_hw_ila [get_hw_ilas -of_objects [get_hw_devices xcvu2p_0] -filter {CELL_NAME=~"gtf_top_0/u_gtfwizard_0_example_gtfmac_top/ILA_GEN[0].tx_mac_ila"}]
run_hw_ila [get_hw_ilas -of_objects [get_hw_devices xcvu2p_0] -filter {CELL_NAME=~"gtf_top_1/u_gtfwizard_0_example_gtfmac_top/ILA_GEN[0].rx_mac_ila"}]
run_hw_ila [get_hw_ilas -of_objects [get_hw_devices xcvu2p_0] -filter {CELL_NAME=~"gtf_top_1/u_gtfwizard_0_example_gtfmac_top/ILA_GEN[0].tx_mac_ila"}]