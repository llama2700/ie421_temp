/*
 * Shared parameters and constants for ll_eth_mac_pcs.
 *
 * 10GBASE-R 64b/66b block type encodings (IEEE 802.3 Clause 49),
 * sync header values, Ethernet preamble/SFD, and control characters.
 */

package ll_eth_cfg;

    // ---------------------------------------------------------------
    // 64b/66b sync headers
    // ---------------------------------------------------------------
    localparam [1:0]
        SYNC_DATA = 2'b10,
        SYNC_CTRL = 2'b01;

    // ---------------------------------------------------------------
    // 64b/66b block type field (descrambled_data[7:0] for SYNC_CTRL)
    // IEEE 802.3 Table 49-1
    // ---------------------------------------------------------------
    localparam [7:0]
        BLOCK_TYPE_CTRL    = 8'h1e,  // C7 C6 C5 C4 C3 C2 C1 C0 BT
        BLOCK_TYPE_START_4 = 8'h33,  // D7 D6 D5    C3 C2 C1 C0 BT
        BLOCK_TYPE_START_0 = 8'h78,  // D7 D6 D5 D4 D3 D2 D1    BT
        BLOCK_TYPE_TERM_0  = 8'h87,
        BLOCK_TYPE_TERM_1  = 8'h99,
        BLOCK_TYPE_TERM_2  = 8'haa,
        BLOCK_TYPE_TERM_3  = 8'hb4,
        BLOCK_TYPE_TERM_4  = 8'hcc,
        BLOCK_TYPE_TERM_5  = 8'hd2,
        BLOCK_TYPE_TERM_6  = 8'he1,
        BLOCK_TYPE_TERM_7  = 8'hff;

    // ---------------------------------------------------------------
    // 7-bit control character encodings (per XGMII)
    // ---------------------------------------------------------------
    localparam [6:0]
        CTRL_IDLE  = 7'h00,
        CTRL_ERROR = 7'h1e;

    // ---------------------------------------------------------------
    // Ethernet preamble and SFD
    // ---------------------------------------------------------------
    localparam [7:0]
        ETH_PRE = 8'h55,
        ETH_SFD = 8'hD5;

    // ---------------------------------------------------------------
    // Internal block type tags (shared between RX and TX)
    // Encoding: bit[3] = 1 for TERM types, bits[2:0] = lane position
    // ---------------------------------------------------------------
    localparam [3:0]
        TYPE_IDLE    = 4'd0,
        TYPE_ERROR   = 4'd1,
        TYPE_START_0 = 4'd2,
        TYPE_START_4 = 4'd3,
        TYPE_DATA    = 4'd4,
        TYPE_TERM_0  = 4'd8,
        TYPE_TERM_1  = 4'd9,
        TYPE_TERM_2  = 4'd10,
        TYPE_TERM_3  = 4'd11,
        TYPE_TERM_4  = 4'd12,
        TYPE_TERM_5  = 4'd13,
        TYPE_TERM_6  = 4'd14,
        TYPE_TERM_7  = 4'd15;

endpackage
