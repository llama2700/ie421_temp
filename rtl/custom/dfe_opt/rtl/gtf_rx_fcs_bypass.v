/*

GTF RX FCS Bypass
Controls the hard MAC's FCS checking and deletion via direct GTF_CHANNEL ports.
Per UG1549, ctl_rx_check_fcs and ctl_rx_delete_fcs are direct ports on the
GTF_CHANNEL primitive, not DRP registers.

Disabling FCS check eliminates the up-to-9-cycle delay between last data byte
and RXAXISTERR assertion in 10G mode.

*/

`resetall
`timescale 1ns / 1ps
`default_nettype none

module gtf_rx_fcs_bypass (
    input  wire        clk,
    input  wire        rst,

    /*
     * Control (from register file)
     */
    input  wire        fcs_check_enable,   // 0 = disable FCS check (HFT default)
    input  wire        fcs_delete_enable,  // 0 = keep FCS bytes in frame
    input  wire        fcs_tx_ins_enable,  // 1 = MAC inserts FCS on TX

    /*
     * GTF_CHANNEL direct ports
     */
    output wire        ctl_rx_check_fcs,
    output wire        ctl_rx_delete_fcs,
    output wire        ctl_tx_fcs_ins_enable,

    /*
     * Status
     */
    output wire        fcs_check_active,
    output wire        fcs_delete_active,
    output wire        fcs_tx_ins_active
);

// Direct passthrough — these are asynchronous control ports on the GTF
assign ctl_rx_check_fcs      = fcs_check_enable;
assign ctl_rx_delete_fcs     = fcs_delete_enable;
assign ctl_tx_fcs_ins_enable = fcs_tx_ins_enable;

// Status readback
assign fcs_check_active  = fcs_check_enable;
assign fcs_delete_active = fcs_delete_enable;
assign fcs_tx_ins_active = fcs_tx_ins_enable;

endmodule

`resetall
