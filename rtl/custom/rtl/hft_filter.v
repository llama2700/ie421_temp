// ------------------------------------------------------------------------------
// HFT Packet Filter Module
// ------------------------------------------------------------------------------
// [ GTF MAC Output ] 
//        |
//        v
// [ 512-bit Data Buffer ]  <-- Cycle 0 (First 64 bytes)
//        |
//        |--- [ Parse Ethernet Header ]
//        |--- [ Parse IP Header ]
//        |--- [ Parse UDP Header ]
//        |
//        v
// [ Extract Destination Port ]  --- (e.g., 5001?) ---+
//        |                                          |
//        |                                          |
//     [ MATCH? ]                               [ NO MATCH? ]
//        |                                          |
//        v                                          v
//  [ PASS TO CORE ]                           [ DROP PACKET ]
//        |                                          |
//  m_axis_tvalid = 1                         m_axis_tvalid = 0
//  m_axis_tdata  = Data                      m_axis_tdata  = Don't Care
//        |                                          |
//        v                                          v
// [ DMA to Host RAM ]                        [ Packet Terminated ]
// ------------------------------------------------------------------------------

module hft_filter  #(
    parameter AXIS_RX_USER_WIDTH = 97,        // 96bit PTP time stamp + 1bit error sign
    parameter [15:0] UDP_DST_PORT = 16'd5000  // Destination UDP port, can be changed during instantiation
)
(
    input  wire        clk,
    input  wire        rst,

    // from MAC
    input  wire [511:0]                    s_axis_tdata,
    input  wire [63:0]                     s_axis_tkeep,
    input  wire                            s_axis_tvalid,
    input  wire                            s_axis_tlast,
    input  wire [AXIS_RX_USER_WIDTH-1:0]   s_axis_tuser,
    output wire                            s_axis_tready,

    // to Corundum
    output wire [511:0]                    m_axis_tdata,
    output wire [63:0]                     m_axis_tkeep,
    output wire                            m_axis_tvalid,
    output wire                            m_axis_tlast,
    output wire [AXIS_RX_USER_WIDTH-1:0]   m_axis_tuser,
    input  wire                            m_axis_tready
);
    // logic:
    // ---------------------------------------------------------------------------
    // header file extraction（all in first stamp 64 bit）
    // bit N: tdata[N*8+7 : N*8]，manually extract fields from the first 64 bytes of the frame, which contains Ethernet, IP and UDP headers
    // ---------------------------------------------------------------------------
    // bit 12-13: EtherType
    wire [15:0] ethertype    = {s_axis_tdata[103:96],  s_axis_tdata[111:104]};
    // bit 23:    IP Protocol
    wire [7:0]  ip_proto     =  s_axis_tdata[191:184];
    // bit 36-37: UDP target port
    wire [15:0] udp_dst_port = {s_axis_tdata[295:288], s_axis_tdata[303:296]};

    // ---------------------------------------------------------------------------
    // logic judge（combination logic，output in the first beat）
    // ---------------------------------------------------------------------------
    wire frame_match = (ethertype    == 16'h0800) &&  // IPv4
                    (ip_proto     == 8'h11)    &&  // UDP
                    (udp_dst_port == UDP_DST_PORT);

    // ---------------------------------------------------------------------------
    // track first beat of a frame，generate in_first_beat signal
    // ---------------------------------------------------------------------------
    reg in_first_beat;

    always @(posedge clk) begin
        if (rst) begin
            in_first_beat <= 1'b1;
        end else if (s_axis_tvalid) begin
            if (s_axis_tlast)
                in_first_beat <= 1'b1;  // this frame ends, next cycle is a new frame header
            else
                in_first_beat <= 1'b0;  // middle of frame, continue
        end
    end

    // ---------------------------------------------------------------------------
    // lock judge result，keep till frame end（latched logic，output in later beats）
    // ---------------------------------------------------------------------------
    reg frame_pass;

    always @(posedge clk) begin
        if (rst) begin
            frame_pass <= 1'b0;
        end else if (s_axis_tvalid && in_first_beat) begin
            frame_pass <= frame_match;  // fist beat latch the match result
        end
    end

    // which judge to use for output：first stamp use comb logic later use latched result
    wire current_pass = in_first_beat ? frame_match : frame_pass;

    // ---------------------------------------------------------------------------
    // output：data transfer，tvalid filter by current_pass
    // ---------------------------------------------------------------------------
    always @(posedge clk) begin
        if (rst) begin
            m_axis_tvalid <= 1'b0;
            m_axis_tdata  <= 512'b0;
            m_axis_tkeep  <= 64'b0;
            m_axis_tlast  <= 1'b0;
            m_axis_tuser  <= {AXIS_RX_USER_WIDTH{1'b0}};
        end else begin
            m_axis_tdata  <= s_axis_tdata;
            m_axis_tkeep  <= s_axis_tkeep;
            m_axis_tlast  <= s_axis_tlast;
            m_axis_tuser  <= s_axis_tuser;
            // only valid data that passes the filter will be sent to Corundum
            m_axis_tvalid <= s_axis_tvalid && current_pass;
        end
    end

    // MAC always push mode，does not support backpressure，alwaysready
    assign s_axis_tready = 1'b1;
    

endmodule