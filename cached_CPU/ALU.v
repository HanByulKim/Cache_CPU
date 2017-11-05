
`define WORD_SIZE 16
module ALU(
    input [15:0] A,
    input [15:0] B,
    input wire [3:0] ALUCtrl,
    output reg [15:0] C,
    output reg [`WORD_SIZE-1:0] output_port,
    output reg isHLT
    );
    
    always @* begin
      isHLT=0;
      #10   // to avoid timing problem
      case(ALUCtrl)
          4'b0000: C = A + B;               // ADD or ADI or LWD or SWD
          4'b0001: begin                    // SUB
            C = A+~B+1;          
          end
          4'b0010: C = A & B;               // AND
          4'b0011: C = A | B;               // ORR or ORI
          4'b0100: C = ~A;                  // NOT
          4'b0101: C = ~A + 1;              // TCP
          4'b0110: C = A<<1;                // SHL
          4'b0111: C = A>>1;                // SHR
          4'b1000: C = {B,8'b00000000};     // LHI
          4'b1101: output_port = A;         // WWD
          4'b1111: isHLT=1;                 // HLT
      endcase
    end
    
endmodule
