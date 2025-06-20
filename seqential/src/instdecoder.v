module instruction_decoder (
    input [31:0] instruction,
    output [4:0] read_reg1,
    output [4:0] read_reg2,
    output [4:0] write_reg,
    output [31:0] imm,
    output [6:0] opcode,
    output [2:0] funct3,
    output [6:0] funct7
);

assign opcode = instruction[6:0];
assign funct3 = instruction[14:12];
assign funct7 = instruction[31:25];

reg [4:0] rs1, rs2, rd,parth;
reg [31:0] imm_val;

always @(*) begin
    // Default assignments
    rs1 = 5'b0;
    rs2 = 5'b0;
    rd = 5'b0;
    imm_val = 32'b0;
    parth = 5'b0;
    case (opcode)
        // R-type (Register-Register ALU operations)
        7'b0000000: begin
            rs1 = instruction[19:15];
            rs2 = instruction[24:20];
            rd = instruction[11:7];
        end
        
        // I-type (Load instructions only)
        7'b0100011: begin
            rs1 = instruction[19:15];
            rd = instruction[11:7];
            imm_val = {{20{instruction[31]}}, instruction[31:20]};
            parth= 5'b1;
        end
        
        // S-type (Store instructions)
        7'b0101011: begin
            rs1 = instruction[19:15];
            rs2 = instruction[24:20];
            imm_val = {{20{instruction[31]}}, instruction[31:25], instruction[11:7]};
        end
        
        // B-type (Branch instructions)
        7'b0000100: begin
            rs1 = instruction[19:15];
            rs2 = instruction[24:20];
            imm_val = {{19{instruction[31]}}, instruction[31], instruction[7], instruction[30:25], instruction[11:8], 1'b0};
        end
        
        default: begin
        end
    endcase
end

assign read_reg1 = rs1;
assign read_reg2 = rs2;
assign write_reg = rd;
assign imm = imm_val;

endmodule
