module Hazard_Detection_unit(
    input [3:0] opcode,
    input [5:0] funct,
    input [1:0] IF_ID_rs,
    input [1:0] IF_ID_rt,
    input [1:0] ID_EX_rt,
    input ID_EX_MemRead,
    input BCEQ, // Branch Condition Equal 
    input BCGT, // Branch Condition Greater Than 
    input BCLT, // Branch Condition Less Than 
    output reg PCWrite, // PcWrite
    output reg IFFlush,
    output reg IF_ID_Stall,
    output reg ControlFlush,
    output reg [1:0] PCSrc
);

wire taken;
assign taken=  ((opcode == 0 && BCEQ == 0) ||   //BNE
                (opcode == 1 && BCEQ == 1) ||   //BEQ
                (opcode == 2 && BCGT == 1) ||   //BGZ
                (opcode == 3 && BCLT == 1))?    //BLZ
                1: 0;

always@(*) begin
    // default
    IFFlush = 0; PCWrite = 1; IF_ID_Stall=0; ControlFlush = 0; PCSrc=1;
    
    // 1 : load(stall)
    if(ID_EX_MemRead==1&&(ID_EX_rt==IF_ID_rt||ID_EX_rt==IF_ID_rs)) begin
        IF_ID_Stall=1;
        PCWrite = 0;
        ControlFlush = 1;
    end
    
    // 2 : branch(flush when branch is taken)
    if(taken==1) begin
        IFFlush = 1;
        PCWrite = 1;    
        ControlFlush = 1;
        PCSrc=0;
    end        
    
    //3 : jump(flush)
    //JMP
    if(opcode == 9) begin
        IFFlush = 1;
        PCWrite = 1;
        ControlFlush = 1;
        PCSrc=2;
    end
    //JAL
    if(opcode == 10) begin
        IFFlush = 1;
        PCWrite = 1;
        ControlFlush = 0;   // to write back $pc to $2
        PCSrc=2;
    end
    //JPR
    if(opcode == 15 && funct == 25) begin
        IFFlush = 1;
        PCWrite = 1;
        ControlFlush = 1;
        PCSrc=3;
    end
    //JRL
    if(opcode == 15 && funct == 26) begin
        IFFlush = 1;
        PCWrite = 1;
        ControlFlush = 0;  // to write back $pc to $2
        PCSrc=3;
    end
end
endmodule
