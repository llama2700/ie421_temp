module hft_filter 
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
    // logic
    

endmodule