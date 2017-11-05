`timescale 1ns / 1ps
`define WORD_SIZE 16

module Register(
    input clk,
    input write,
    input [1:0] addr1,
    input [1:0] addr2,
    input [1:0] addr3,
    input [`WORD_SIZE-1:0] data3,
    output wire [`WORD_SIZE-1:0] data1,
    output wire [`WORD_SIZE-1:0] data2
    );
reg [`WORD_SIZE-1:0] register [3:0]; 

// setting output
always @(negedge clk) begin
    if(write == 1) register[addr3] = data3;  
end
assign data1 = register[addr1];
assign data2 = register[addr2];
  
endmodule
