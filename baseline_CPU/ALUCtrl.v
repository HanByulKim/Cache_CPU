
module ALUctrl_unit(
    input [3:0] ALUOp,
    input [5:0] funct,
    output reg [3:0] ALUCtrl
    );
    always @(*) begin
        case(ALUOp)
            15: begin
                case(funct)
                    0:  ALUCtrl=4'b0000; //ADD
                    1:  ALUCtrl=4'b0001; //SUB
                    2:  ALUCtrl=4'b0010; //AND
                    3:  ALUCtrl=4'b0011; //ORR
                    4:  ALUCtrl=4'b0100; //NOT
                    5:  ALUCtrl=4'b0101; //TCP
                    6:  ALUCtrl=4'b0110; //SHL
                    7:  ALUCtrl=4'b0111; //SHR
                    28: ALUCtrl=4'b1101; //WWD
                    29: ALUCtrl = 4'b1111;//HLT
                endcase
            end
            4: ALUCtrl = 4'b0000; //ADI
            5: ALUCtrl = 4'b0011; //ORI            
            6: ALUCtrl = 4'b1000; //LHI
            7: ALUCtrl = 4'b0000; //LWD
            8: ALUCtrl = 4'b0000; //SWD
            default: ALUCtrl = 4'b0000; //JMP JAL JPR JRL
        endcase
    end
    
endmodule
