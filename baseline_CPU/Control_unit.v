`include "opcodes.v"
`include "mapping.v"

`define RegWrite 11
`define MemtoReg 10:9
`define MemRead 8
`define MemWrite 7
`define RegDst 6:5
`define ALUOp 4:1
`define ALUSrc 0

module Control_unit (
  input wire [3:0] opcode,
  input wire [5:0] funct,
  input wire reset_n,
  output reg [`CTRL_BUS_SIZE-1:0] controlBus
);
always @(*) begin
    if({opcode, funct}==10'b1111011100)         controlBus = 12'b0xx00xx1111x;
    else if({opcode, funct}==10'b1111011001)    controlBus = 12'b0xx00xx1111x;
    else if({opcode, funct}==10'b1111011010)    controlBus = 12'b11000101111x;
    else if({opcode, funct}==10'b1111011101)    controlBus = 12'b0xx00xx1111x;
    else if(opcode==4'b1111)                    controlBus = 12'b100000111110;
    else if(opcode==4'b0100)                    controlBus = 12'b100000001001;
    else if(opcode==4'b0101)                    controlBus = 12'b100000001011;
    else if(opcode==4'b0110)                    controlBus = 12'b100000001101;
    else if(opcode==4'b0111)                    controlBus = 12'b101100001111;
    else if(opcode==4'b1000)                    controlBus = 12'b0xx01xx10001;
    else if(opcode==4'b0000)                    controlBus = 12'b0xx00xx1011x;
    else if(opcode==4'b0001)                    controlBus = 12'b0xx00xx0001x;
    else if(opcode==4'b0010)                    controlBus = 12'b0xx00xx0010x;
    else if(opcode==4'b0011)                    controlBus = 12'b0xx00xx0011x;
    else if(opcode==4'b1001)                    controlBus = 12'b0xx00xx1001x;
    else if(opcode==4'b1010)                    controlBus = 12'b11000101010x;
end
endmodule
/*
 MemtoReg = 2 means PC to Reg
 RegDest = 2 means rd = $2
RegWrite	MemtoReg1	MemtoReg0	MemRead	MemWrite	RegDst1	RegDst0	ALUOp3	ALUOp2	ALUOp1	ALUOp0	ALUSrc
R	1	0	0	0	0	0	1	1	1	1	1	0
ADI	1	0	0	0	0	0	0	0	1	0	0	1
ORI	1	0	0	0	0	0	0	0	1	0	1	1
LHI	1	0	0	0	0	0	0	0	1	1	0	1
WWD	0	x	x	0	0	x	x	1	1	1	1	x
LWD	1	0	1	1	0	0	0	0	1	1	1	1
SWD	0	x	x	0	1	x	x	1	0	0	0	1
BNE	0	x	x	0	0	x	x	1	0	1	1	x
BEQ	0	x	x	0	0	x	x	0	0	0	1	x
BGZ	0	x	x	0	0	x	x	0	0	1	0	x
BLZ	0	x	x	0	0	x	x	0	0	1	1	x
JMP	0	x	x	0	0	x	x	1	0	0	1	x
JAL	1	1	0	0	0	1	0	1	0	1	0	x
JPR	0	x	x	0	0	x	x	1	1	1	1	x
JRL	1	1	0	0	0	1	0	1	1	1	1	x
HLT	0	x	x	0	0	x	x	1	1	1	1	x
*/