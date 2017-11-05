`define WORD_SIZE 16    // data and address word size
`include "mapping.v"

module Datapath(
  output wire i_readM,                       // i_read from memory  
  output wire i_writeM,                       // i_write from memory
  output wire [`WORD_SIZE-1:0] i_address,    // i_current address for data
  output wire d_readM,                       // d_read from memory  
  output wire d_writeM,                       // d_write from memory
  output wire [`WORD_SIZE-1:0] d_address,    // d_current address for data
  inout wire [`WORD_SIZE-1:0] i_data,              // data being isnput or output
  inout [`WORD_SIZE-1:0] d_data,              // data being isnput or output
  input reset_n,                            // active-low RESET signal
  input clk,                                // clock signal
  
  // for debuging/testing purpose
  output reg [`WORD_SIZE-1:0] num_inst,   // number of instruction during execution
  output wire [`WORD_SIZE-1:0] output_port, // this will be used for a "WWD" instruction
  output is_halted                      // this means cpu is halted
  
);

//
reg [1:0] memCount;

// latchs
reg [`LATCH_SIZE-1:0] latchs;

// IF declaration
wire [`WORD_SIZE-1:0] nextPC;

// ID declaration
wire [`CTRL_BUS_SIZE-1:0] controlBus;   // control signals
wire IF_Flush;
wire [1:0] PCSrc;                       // PC control signal
wire [3:0] ALUCtrl;                     // ALU control signal
wire [`CTRL_BUS_SIZE-1:0] nextID_EX_Ctrls;

wire [`WORD_SIZE-1:0] inst;

wire [1:0] ID_forwardRs;
wire [1:0] ID_forwardRt;
wire [`WORD_SIZE-1:0] ID_forwardedRs;
wire [`WORD_SIZE-1:0] ID_forwardedRt;

wire PCWrite;
wire ControlFlush;
wire IF_ID_Stall;

wire BCEQ;
wire BCGT;
wire BCLT;

wire [`WORD_SIZE-1:0] RFOutA;
wire [`WORD_SIZE-1:0] RFOutB;

// sign-extended immediate
wire [`WORD_SIZE-1:0] extendImm;

// EX declaration

wire [`WORD_SIZE-1:0] ALUInputA;
wire [`WORD_SIZE-1:0] ALUInputB;
wire [`WORD_SIZE-1:0] EX_forwardedRt;
wire [`WORD_SIZE-1:0] EX_ALUOut;
wire [5:0] EX_funct;
wire [1:0] EX_rd;
wire [1:0] EX_forwardA;
wire [1:0] EX_forwardB;


//MEM declaration
wire [`WORD_SIZE-1:0] DMemOut;

//WB declaration
wire [`WORD_SIZE-1:0] WB_RFWriteData;


// Datapath module declaration
Control_unit ctrl_unit(.opcode(latchs[`IF_ID_opcode]), .funct(latchs[`IF_ID_funct]), .reset_n(reset_n), .controlBus(controlBus));
Register register(.clk(clk), .write(latchs[`MEM_WB_RegWrite]), .addr1(latchs[`IF_ID_rs]), .addr2(latchs[`IF_ID_rt]), .addr3(latchs[`MEM_WB_rd]), .data3(WB_RFWriteData), .data1(RFOutA), .data2(RFOutB));
Hazard_Detection_unit hazard_detection_unit(.opcode(latchs[`IF_ID_opcode]), .funct(latchs[`IF_ID_funct]), 
        .IF_ID_rs(latchs[`IF_ID_rs]), .IF_ID_rt(latchs[`IF_ID_rt]), .ID_EX_rt(latchs[`ID_EX_rt]), .ID_EX_MemRead(latchs[`ID_EX_MemRead]), 
        .BCEQ(BCEQ), .BCGT(BCGT), .BCLT(BCLT), .PCWrite(PCWrite), .IFFlush(IF_Flush), .IF_ID_Stall(IF_ID_Stall), .ControlFlush(ControlFlush), .PCSrc(PCSrc));
Forwarding_unit forwarding_unit(.IF_ID_rs(latchs[`IF_ID_rs]), .IF_ID_rt(latchs[`IF_ID_rt]), .ID_EX_rs(latchs[`ID_EX_rs]), 
                                .ID_EX_rt(latchs[`ID_EX_rt]), .ID_EX_rd(EX_rd), .EX_MEM_rd(latchs[`EX_MEM_rd]), .MEM_WB_rd(latchs[`MEM_WB_rd]), 
                                .ID_EX_RegWrite(latchs[`ID_EX_RegWrite]), .EX_MEM_RegWrite(latchs[`EX_MEM_RegWrite]), .MEM_WB_RegWrite(latchs[`MEM_WB_RegWrite]), 
                                .forwardA(EX_forwardA), .forwardB(EX_forwardB), .forwardRs(ID_forwardRs), .forwardRt(ID_forwardRt));
ALUctrl_unit aluctrl_unit(.ALUOp(latchs[`ID_EX_ALUOp]), .funct(EX_funct), .ALUCtrl(ALUCtrl));
ALU alu(.A(ALUInputA), .B(ALUInputB), .ALUCtrl(ALUCtrl), .C(EX_ALUOut), .output_port(output_port), .isHLT(is_halted));

// IF wiring
assign i_address = latchs[`IF_pc];
assign i_readM = PCWrite;
assign i_writeM = 0;
assign nextPC=(PCSrc<=1) ? 
    ( (PCSrc==0) ?  extendImm+latchs[`IF_ID_pc]: latchs[`IF_pc]+1 ) :       //0: branch offset, 1: pc+1
    ( (PCSrc==2) ?  { latchs[`IF_ID_pc]>>12,latchs[`IF_ID_target] } :       //2: jump offset,   3: jump register
                    { (ID_forwardRs==0)?   RFOutA :                                                                         //no forwarding
                      (ID_forwardRs==1)?   WB_RFWriteData :                                                                 //from WB
                      (ID_forwardRs==2)?   ((latchs[`EX_MEM_MemtoReg]==2)? latchs[`EX_MEM_pc] : latchs[`EX_MEM_ALUOut] ) :  //from MEM 
                                           (latchs[`ID_EX_MemtoReg]==2)? latchs[`ID_EX_pc]:EX_ALUOut } ) ;                  //from EX
                                           
// ID wiring
assign extendImm = (latchs[`IF_ID_imm]>>7 == 1) ? {8'b11111111,latchs[`IF_ID_imm]} : {8'b00000000,latchs[`IF_ID_imm]}; // sign-extend
assign nextID_EX_Ctrls = (ControlFlush ==0) ? controlBus : 12'bz;

assign ID_forwardedRs=(ID_forwardRs==0)? RFOutA                 //no forwarding
            : (ID_forwardRs==1)? WB_RFWriteData                 //from WB
            : (ID_forwardRs==2)? latchs[`EX_MEM_ALUOut]         //from MEM 
            : EX_ALUOut;                                        //from EX
            
assign ID_forwardedRt=(ID_forwardRt==0)? RFOutB                 //no forwarding
            : (ID_forwardRt==1)? WB_RFWriteData                 //from WB
            : (ID_forwardRt==2)? latchs[`EX_MEM_ALUOut]         //from MEM 
            : EX_ALUOut;                                        //from EX
            
assign BCEQ = (ID_forwardedRs==ID_forwardedRt)? 1: 0;
assign BCGT = (ID_forwardedRs[`WORD_SIZE-1]==0&&ID_forwardedRs!=16'b0)? 1:0;
assign BCLT = (ID_forwardedRs[`WORD_SIZE-1]==1)? 1:0;

// EX wiring
assign ALUInputA =  (EX_forwardA == 0)? latchs[`ID_EX_rsData] : 
                    (EX_forwardA ==1)? WB_RFWriteData : { (latchs[`EX_MEM_MemtoReg]==2)? latchs[`EX_MEM_pc]: latchs[`EX_MEM_ALUOut] };
assign EX_forwardedRt =    (EX_forwardB == 0)? latchs[`ID_EX_rtData] : 
                        (EX_forwardB ==1)? WB_RFWriteData : (latchs[`EX_MEM_MemtoReg]==2)? latchs[`EX_MEM_pc]: latchs[`EX_MEM_ALUOut];
assign ALUInputB = (latchs[`ID_EX_ALUSrc]==0)? (EX_forwardedRt) : latchs[`ID_EX_imm];
assign EX_funct = latchs[`ID_EX_imm];
assign EX_rd = (latchs[`ID_EX_RegDst]==0)? latchs[`ID_EX_rt] : (latchs[`ID_EX_RegDst]==1)? latchs[`ID_EX_rd] : 2;

// MEM wiring
assign d_address = latchs[`EX_MEM_ALUOut];
assign d_writeM = latchs[`EX_MEM_MemWrite];
assign d_readM = latchs[`EX_MEM_MemRead];
assign d_data = (d_writeM == 1)? (latchs[`EX_MEM_MemWriteData]) : 16'bz;
assign DMemOut = (d_readM ==1)? d_data: 16'bz;

// WB wiring
assign WB_RFWriteData = (latchs[`MEM_WB_MemtoReg]==0)? latchs[`MEM_WB_ALUOut] : (latchs[`MEM_WB_MemtoReg]==1)? latchs[`MEM_WB_DMemOut] : latchs[`MEM_WB_pc];

// check reset_n
always @(negedge reset_n) begin
    num_inst<=-1;
    latchs=0;
    latchs[`IF_pc] = -1;
    memCount=0;
end

always @(negedge clk) begin
    if(reset_n==1&&is_halted==0&&memCount[1:0]==2'b00) begin
        if(latchs[`ID_EX_incNum]==1) num_inst = num_inst + 1;
    end
end
always@(negedge clk) begin
    if(reset_n==1&&is_halted==0) begin
        if(i_readM==1 || d_readM==1 || d_writeM==1) begin
            if(memCount[1:0]==2'b10) begin
                memCount=0;
            end
            else begin 
                memCount = memCount + 1;
            end
        end
    end
end
always@(posedge clk) begin
    if(reset_n==1&&is_halted==0&&memCount==0) begin
        if(PCWrite == 1) latchs[`IF_pc] <= nextPC;
        
        // IF/ID
        if(IF_ID_Stall == 0&&IF_Flush==0) begin
            latchs[`IF_ID_pc] <=latchs[`IF_pc] + 1; 
            latchs[`IF_ID_inst] <= i_data;
        end
        else if(IF_Flush==1) begin
            latchs[`IF_ID_inst] <= 16'bz;
        end
        if(IF_Flush==1) begin 
            latchs[`IF_ID_incNum]<=0;
            latchs[`ID_EX_incNum] <= latchs[`IF_ID_incNum];
        end
        else if(IF_ID_Stall==1) begin
            latchs[`IF_ID_incNum]<=1;
            latchs[`ID_EX_incNum]<=0;
        end
        else begin 
            latchs[`IF_ID_incNum]<=1;
            latchs[`ID_EX_incNum] <= latchs[`IF_ID_incNum];
        end
        
        // ID/EX
        latchs[`ID_EX_controlBus] <= nextID_EX_Ctrls;
        latchs[`ID_EX_pc] <= latchs[`IF_ID_pc];
        latchs[`ID_EX_rsData] <= RFOutA;
        latchs[`ID_EX_rtData] <= RFOutB;
        latchs[`ID_EX_imm] <= extendImm;
        latchs[`ID_EX_rs] <= latchs[`IF_ID_rs];
        latchs[`ID_EX_rt] <= latchs[`IF_ID_rt];
        latchs[`ID_EX_rd] <= latchs[`IF_ID_rd];
        
        // EX/MEM
        latchs[`EX_MEM_controlWB] <= latchs[`ID_EX_controlWB];
        latchs[`EX_MEM_controlMEM] <= latchs[`ID_EX_controlMEM];    
        latchs[`EX_MEM_pc] <= latchs[`ID_EX_pc];
        latchs[`EX_MEM_ALUOut] <= EX_ALUOut;
        latchs[`EX_MEM_MemWriteData] <= ((EX_forwardB == 0)? latchs[`ID_EX_rtData] : (EX_forwardB ==1)? WB_RFWriteData : latchs[`EX_MEM_ALUOut]);
        latchs[`EX_MEM_rd] <= EX_rd;
        
        // MEM/WB
        latchs[`MEM_WB_controlWB] <= latchs[`EX_MEM_controlWB];
        latchs[`MEM_WB_pc] <= latchs[`EX_MEM_pc];
        latchs[`MEM_WB_DMemOut] <= DMemOut;
        latchs[`MEM_WB_ALUOut] <= latchs[`EX_MEM_ALUOut];
        latchs[`MEM_WB_rd] <= latchs[`EX_MEM_rd];
    end
end

always @(posedge clk) begin
    if(reset_n==1&&is_halted==0&&memCount[1:0]==0) begin
        if(IF_Flush==1) begin 
            latchs[`IF_ID_inst] = 0;
        end
    end
end

endmodule