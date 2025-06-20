`include "alu_processor.v"
`include "ifblock.v"
`include "instdecoder.v"
`include "memblock.v"
`include "writeback.v"
`include "control_unit.v"
`include "alu_control.v"

// Immediate Extension Block
module immediate_extension(
    input [31:0] imm_in,
    output [63:0] imm_out
);
    // Sign extend the immediate value to 64 bits
    assign imm_out = {{32{imm_in[31]}}, imm_in};
endmodule

module processor(
    input clk,
    input reset
);

    // Program Counter
    reg [63:0] PC;
    
    // Wires
    wire [31:0] instruction;
    wire [63:0] reg_data1, reg_data2, alu_result, mem_data, wb_data;
    wire [31:0] imm_ext;
    wire [63:0] imm_ext_64;  // 64-bit sign-extended immediate
    wire [3:0] alu_control;
    wire [6:0] opcode, funct7;
    wire [2:0] funct3;
    wire reg_write, mem_write, mem_read, alu_src, mem_to_reg, branch;
    wire [1:0] alu_op;
    wire zero, PCsrc;
    wire [4:0] read_reg1, read_reg2, write_reg;
    wire [63:0] pc_branch_target;  // PC branch target address

    // PC Update Logic with shifted immediate
    assign pc_branch_target = PC + imm_ext_64;

    always @(posedge clk or posedge reset) begin
        if (reset)
            PC <= 0;
        else
            PC <= (PCsrc) ? pc_branch_target : (PC + 1);
    end

    // Instruction Fetch Stage
    instruction_memory IF (
        .PC(PC),
        .instruction(instruction)
    );

    // Instruction Decode Stage
    instruction_decoder ID (
        .instruction(instruction), 
        .read_reg1(read_reg1),
        .read_reg2(read_reg2),
        .write_reg(write_reg),
        .imm(imm_ext),
        .opcode(opcode),
        .funct3(funct3),
        .funct7(funct7)
    );

    // Immediate Extension Block
    immediate_extension IMM_EXT (
        .imm_in(imm_ext),
        .imm_out(imm_ext_64)
    );

    // Control Unit
    ControlUnit CU (
        .Op(opcode[5:0]),
        .RegWrite(reg_write),
        .MemtoReg(mem_to_reg),
        .MemRead(mem_read),
        .MemWrite(mem_write),
        .Branch(branch),
        .ALUSrc(alu_src),
        .ALUOp(alu_op)
    );

    // Register File
    register_file RF (
        .clk(clk),
        .reset(reset),
        .read_reg1(read_reg1),
        .read_reg2(read_reg2),
        .write_reg(write_reg),
        .write_data(wb_data),
        .regWrite(reg_write),
        .read_data1(reg_data1),
        .read_data2(reg_data2)
    );

    // ALU Control
    alu_control ALU_CTRL (
        .ALUOp(alu_op),
        .func3(funct3),
        .func7(funct7),
        .ALUControl(alu_control)
    );

    // Execute Stage - ALU with sign-extended immediate
    wire [63:0] ALU_B;
    assign ALU_B = (alu_src) ? imm_ext_64 : reg_data2;  // Use sign-extended immediate

    ALU EX (
        .control(alu_control), 
        .A(reg_data1), 
        .B(ALU_B),  
        .result(alu_result), 
        .Zero(zero)
    );

    // Memory Stage
    data_memory MEM (
        .clk(clk), 
        .memWrite(mem_write), 
        .memRead(mem_read), 
        .reset(reset),
        .address(alu_result), 
        .write_data(reg_data2), 
        .read_data(mem_data)
    );

    // Branch Control
    assign PCsrc = branch & zero;

    // Simple mux for writeback
    assign wb_data = mem_to_reg ? mem_data : alu_result;

endmodule

