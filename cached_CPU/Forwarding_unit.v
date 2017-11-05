module Forwarding_unit(
    input wire [1:0] IF_ID_rs,
    input wire [1:0] IF_ID_rt,
    input wire [1:0] ID_EX_rs,
    input wire [1:0] ID_EX_rt,
    input wire [1:0] ID_EX_rd,
    input wire [1:0] EX_MEM_rd,
    input wire [1:0] MEM_WB_rd,
    input wire ID_EX_RegWrite,
    input wire EX_MEM_RegWrite,
    input wire MEM_WB_RegWrite,
    output reg [1:0] forwardA,
    output reg [1:0] forwardB,
    output reg [1:0] forwardRs,
    output reg [1:0] forwardRt
);
    always @(*) begin
        //forwarding for ALUInputA in EX stage
        if(EX_MEM_RegWrite && EX_MEM_rd == ID_EX_rs) forwardA = 2'b10;              //from EX
        else if(MEM_WB_RegWrite && MEM_WB_rd == ID_EX_rs) forwardA = 2'b01;         //from MEM
        else forwardA=0;
        
        //forwarding for ALUInputB in EX stage
        if(EX_MEM_RegWrite && EX_MEM_rd == ID_EX_rt) forwardB=2'b10;                //from EX
        else if(MEM_WB_RegWrite && MEM_WB_rd == ID_EX_rt) forwardB = 2'b01;         //from MEM
        else forwardB=0;
        
        //forwarding for Rs in ID stage         
        if(ID_EX_RegWrite && ID_EX_rd == IF_ID_rs) forwardRs = 2'b11;               //from ID
        else if(EX_MEM_RegWrite && EX_MEM_rd == IF_ID_rs) forwardRs=2'b10;          //from EX
        else if(MEM_WB_RegWrite && MEM_WB_rd == IF_ID_rs) forwardRs = 2'b01;        //from MEM
        else forwardRs=0;
        
        //forwarding for Rd in ID stage
        if(ID_EX_RegWrite && ID_EX_rd == IF_ID_rt) forwardRt = 2'b11;               //from ID
        else if(EX_MEM_RegWrite && EX_MEM_rd == IF_ID_rt) forwardRt=2'b10;          //from EX
        else if(MEM_WB_RegWrite && MEM_WB_rd == IF_ID_rt) forwardRt = 2'b01;        //from MEM
        else forwardRt=0;
    end        
    
endmodule
