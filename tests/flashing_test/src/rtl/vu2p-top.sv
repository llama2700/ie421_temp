`timescale 1ns/1ns

module top (
    input logic clk_sys_lvds_300_p,
    input logic clk_sys_lvds_300_n
);

    //////////////////////
    // Clocking
    //////////////////////
    logic clk_sys_lvds_300_ibuf;
    logic clk_sys_lvds_300;
    logic clk_sys_100;
    logic clk_sys_100_locked;
    logic clk_sys_100_reset;

    clk_wiz_0 mmcm0 (
        // Clock out ports
        .clk_out1(clk_sys_100),     // output clk_out1
        // Status and control signals
        .reset(1'b0), // input reset
        .locked(clk_sys_100_locked),       // output locked
    // Clock in ports
        .clk_in1_p(clk_sys_lvds_300_p),    // input clk_in1_p
        .clk_in1_n(clk_sys_lvds_300_n)    // input clk_in1_n
    );

    assign clk_sys_100_reset = ~clk_sys_100_locked;


    //////////////////////
    // Fun test stuff
    //////////////////////

    logic [6:0] counter = '0;

    always_ff @ (posedge clk_sys_100) begin
        counter <= counter + 1'b1;
    end

    ila_0 il0(
        .clk(clk_sys_100), // input wire clk
        .probe0(counter) // input wire [6:0] probe0
    );

endmodule