`include "alu_processor.v"
`include "ifblock.v"
`include "instdecoder.v"
`include "memblock.v"
`include "writeback.v"
`include "control_unit.v"
`include "alu_control.v"
`include "hazard_detection_unit.v"
`include "forwardingunit.v"

module immediate_extension(
    input [31:0] imm_in,
    output reg [63:0] imm_out
);
    always @* imm_out = {{32{imm_in[31]}}, imm_in};
endmodule


module IF_ID_reg(
    input clk, reset,hazard_stall,flush,
     input            if_id_WriteEnable,
    input [63:0] IF_pc,
    input [31:0] IF_instruction,
    output reg [63:0] ID_pc,
    output reg [31:0] ID_instruction
);
    always @(posedge clk or posedge reset) begin
           if (reset) begin
      ID_pc <= 64'b0;
      ID_instruction <= 32'b0;
    end else if (if_id_WriteEnable) begin
     ID_pc <= IF_pc;
      ID_instruction <= IF_instruction;
      //
    end
    if (flush) begin
      // IF_ID_PC <= 64'b0;
      ID_instruction <= 32'b0;
    end

    end
endmodule

module ID_EX_reg(
    input clk, reset,hazard_stall,flush,
    // Control signals
    input ID_RegWrite, ID_MemtoReg, ID_MemRead, ID_MemWrite, ID_ALUSrc, ID_Branch,
    input [1:0] ID_ALUOp,
    // Data inputs
    input [63:0] ID_pc,
    input [4:0] ID_rs1, ID_rs2, 
    input [63:0] ID_reg1, ID_reg2, // Fixed width to 64 bits
    input [63:0] ID_imm,
    input [4:0] ID_rd,
    input [2:0] ID_funct3,
    input [6:0] ID_funct7,
    // Outputs
    output reg EX_RegWrite, EX_MemtoReg, EX_MemRead, EX_MemWrite, EX_ALUSrc, EX_Branch,
    output reg [1:0] EX_ALUOp,
    output reg [4:0] EX_rs1, EX_rs2, 
    output reg [63:0] EX_pc, EX_reg1, EX_reg2, EX_imm,
    output reg [4:0] EX_rd,
    output reg [2:0] EX_funct3,
    output reg [6:0] EX_funct7
);
    always @(posedge clk or posedge reset) begin
    if(reset ||hazard_stall ||flush) begin
        EX_RegWrite  <= 0;
        EX_MemtoReg  <= 0;
        EX_MemRead   <= 0;
        EX_MemWrite  <= 0;
        EX_ALUSrc    <= 0;
        EX_Branch    <= 0;
        EX_ALUOp     <= 0;
        EX_pc        <= 0;
        EX_reg1      <= 0;
        EX_reg2      <= 0;
        EX_imm       <= 0;
        EX_rd        <= 0;
        EX_funct3    <= 0;
        EX_funct7    <= 0;
        EX_rs1       <= 0;
        EX_rs2       <= 0;
    end else if (~hazard_stall) begin
        EX_RegWrite  <= ID_RegWrite;
        EX_MemtoReg  <= ID_MemtoReg;
        EX_MemRead   <= ID_MemRead;
        EX_MemWrite  <= ID_MemWrite;
        EX_ALUSrc    <= ID_ALUSrc;
        EX_Branch    <= ID_Branch;
        EX_ALUOp     <= ID_ALUOp;
        EX_pc        <= ID_pc;
        EX_reg1      <= ID_reg1;
        EX_reg2      <= ID_reg2;
        EX_imm       <= ID_imm;
        EX_rd        <= ID_rd;
        EX_funct3    <= ID_funct3;
        EX_funct7    <= ID_funct7;
        EX_rs1       <= ID_rs1;  // Properly assigned 5-bit register
        EX_rs2       <= ID_rs2;  // Properly assigned 5-bit register
    end
end

endmodule

module EX_MEM_reg(
    input clk, reset,flush,
    input [63:0] actual_target_EX,
    input EX_RegWrite, EX_MemtoReg, EX_MemRead, EX_MemWrite,EX_Branch,
    input [63:0] EX_alu_result, 
    input [63:0] EX_reg2, // Fixed width to 64 bits
    input [4:0] EX_rd,
    input EX_Zero,
    input branch_taken_EX,
    output reg MEM_RegWrite, MEM_MemtoReg, MEM_MemRead, MEM_MemWrite,MEM_Branch,branch_taken_MEM,
    output reg [63:0] MEM_alu_result, MEM_reg2,
    output reg [4:0] MEM_rd,
    output reg [63:0] actual_target_MEM,
    output reg MEM_Zero
);
    always @(posedge clk or posedge reset) begin
        if(reset||flush) begin {MEM_RegWrite, MEM_MemtoReg, MEM_MemRead, MEM_MemWrite, MEM_alu_result, MEM_reg2, MEM_rd, MEM_Zero} <= 8'b0;
           branch_taken_MEM <=branch_taken_EX;
           MEM_Branch <= EX_Branch;
      actual_target_MEM <= actual_target_EX;
        end
        else begin
            {MEM_RegWrite, MEM_MemtoReg, MEM_MemRead, MEM_MemWrite} <= 
            {EX_RegWrite, EX_MemtoReg, EX_MemRead, EX_MemWrite};
            {MEM_alu_result, MEM_reg2, MEM_rd, MEM_Zero} <= 
            {EX_alu_result, EX_reg2, EX_rd, EX_Zero};
             actual_target_MEM <= actual_target_EX;
              branch_taken_MEM <= branch_taken_EX;
               MEM_Branch <= EX_Branch;
        end
    end
endmodule

module MEM_WB_reg(
    input clk, reset,
    input MEM_RegWrite, MEM_MemtoReg,
    input [63:0] MEM_read_data, MEM_alu_result,
    input [4:0] MEM_rd,
    output reg WB_RegWrite, WB_MemtoReg,
    output reg [63:0] WB_read_data, WB_alu_result,
    output reg [4:0] WB_rd
);
    always @(posedge clk or posedge reset) begin
        if(reset) {WB_RegWrite, WB_MemtoReg, WB_read_data, WB_alu_result, WB_rd} <= 5'b0;
        else begin
            {WB_RegWrite, WB_MemtoReg} <= {MEM_RegWrite, MEM_MemtoReg};
            {WB_read_data, WB_alu_result, WB_rd} <= {MEM_read_data, MEM_alu_result, MEM_rd};
        end
    end
endmodule

module processor_pipeline0(
    input clk,
    input reset
);
    // Pipeline registers
    wire [63:0] IF_pc, ID_pc, EX_pc;
    wire [31:0] IF_instruction, ID_instruction;
     wire if_id_WriteEnable;
  wire pc_WriteEnable;
    // ID stage signals
    wire [63:0] reg1, reg2;
    wire [31:0] imm_ext;
    wire [63:0] imm_ext_64;
    wire [1:0] alu_op;
    wire [6:0] opcode;
    wire [5:0] opcode_trimmed; // Added to handle the 6-bit opcode for ControlUnit
    wire [6:0] funct7;
    wire [2:0] funct3;
    wire reg_write, mem_write, mem_read, alu_src, mem_to_reg, branch;
    wire [4:0] rs1, rs2, rd;
    
    // EX stage signals
    wire EX_RegWrite, EX_MemtoReg, EX_MemRead, EX_MemWrite, EX_ALUSrc, EX_Branch;
    wire [4:0] EX_rs1, EX_rs2;

    wire [1:0] EX_ALUOp;
    wire [63:0] EX_reg1, EX_reg2; // Explicitly defined as 64-bit wires
    wire [63:0] EX_imm;
    wire [4:0] EX_rd;
    wire [2:0] EX_funct3;
    wire [6:0] EX_funct7;
    wire [63:0] alu_result;
    wire zero;
    
    // MEM stage signals
    wire MEM_RegWrite, MEM_MemtoReg, MEM_MemRead, MEM_MemWrite,MEM_Branch,branch_taken_MEM;
    wire [63:0] MEM_alu_result, MEM_reg2;
    wire [4:0] MEM_rd;
    wire MEM_Zero;
    wire [63:0] mem_data;
    
    // WB stage signals
    wire WB_RegWrite, WB_MemtoReg;
    wire [63:0] WB_read_data, WB_alu_result;
    wire [4:0] WB_rd;
    wire [63:0] wb_data;
    wire [1:0] forwardA, forwardB;
    wire hazard_stall;
    // PC Logic
    reg [63:0] PC;
    reg flush;
     wire [63:0] actual_target_EX = EX_pc + (EX_imm); // Calculate the actual target address for the branch instruction
    wire [63:0] pc_next;
    wire        branch_taken_EX;
     assign pc_next  = (branch_taken_MEM) ? actual_target_MEM : PC+1;
     assign branch_taken_EX = EX_Branch && zero;

    // IF Stage
    instruction_memory IM(
        .PC(PC),
        .instruction(IF_instruction)
    );

    hazard_detection_unit HDU (
        .MemRead_EX(EX_MemRead),
        .Rd_EX(EX_rd),
        .structural_hazard(1'b0),  // Assume no structural hazard unless signaled externally.
        .branch(EX_Branch),
        .IFID_inst(ID_instruction),
        .stall(hazard_stall),
         .if_id_WriteEnable(if_id_WriteEnable),
      .pc_WriteEnable(pc_WriteEnable)
    );
        always @(posedge clk or posedge reset) begin
        if(reset) PC <= 0;
        else if (pc_WriteEnable) PC <= pc_next;
    end

    // IF/ID Pipeline
    IF_ID_reg IF_ID(
        .hazard_stall(hazard_stall),
        .flush(flush),
        .clk(clk),
        .reset(reset),
         .if_id_WriteEnable(if_id_WriteEnable),
        .IF_pc(PC),
        .IF_instruction(IF_instruction),
        .ID_pc(ID_pc),
        .ID_instruction(ID_instruction)
    );

    // ID Stage
    instruction_decoder ID(
        .instruction(ID_instruction),
        .read_reg1(rs1),
        .read_reg2(rs2),
        .write_reg(rd),
        .imm(imm_ext),
        .opcode(opcode),
        .funct3(funct3),
        .funct7(funct7)
    );

    // Trim opcode to 6 bits for ControlUnit
    assign opcode_trimmed = opcode[5:0];

    immediate_extension IMM_EXT(
        .imm_in(imm_ext),
        .imm_out(imm_ext_64)
    );

    ControlUnit CU(
        .Op(opcode_trimmed), // Using the trimmed 6-bit opcode
        .RegWrite(reg_write),
        .MemtoReg(mem_to_reg),
        .MemRead(mem_read),
        .MemWrite(mem_write),
        .Branch(branch),
        .ALUSrc(alu_src),
        .ALUOp(alu_op)
    );

    register_file RF(
        .clk(clk),
        .reset(reset),
        .read_reg1(rs1),
        .read_reg2(rs2),
        .write_reg(WB_rd),
        .write_data(wb_data),
        .regWrite(WB_RegWrite),
        .read_data1(reg1),
        .read_data2(reg2)
    );

    // ID/EX Pipeline
    ID_EX_reg ID_EX(
        .clk(clk),
        .hazard_stall(hazard_stall),
        .flush(flush),
        .reset(reset),
        .ID_RegWrite(reg_write),
        .ID_rs1(rs1),
        .ID_rs2(rs2),
        .ID_MemtoReg(mem_to_reg),
        .ID_MemRead(mem_read),
        .ID_MemWrite(mem_write),
        .ID_ALUSrc(alu_src),
        .ID_Branch(branch),
        .ID_ALUOp(alu_op),
        .ID_pc(ID_pc),
        .ID_reg1(reg1),
        .ID_reg2(reg2),
        .ID_imm(imm_ext_64),
        .ID_rd(rd),
        .ID_funct3(funct3),
        .ID_funct7(funct7),
        .EX_RegWrite(EX_RegWrite),
        .EX_MemtoReg(EX_MemtoReg),
        .EX_MemRead(EX_MemRead),
        .EX_MemWrite(EX_MemWrite),
        .EX_ALUSrc(EX_ALUSrc),
        .EX_Branch(EX_Branch),
        .EX_ALUOp(EX_ALUOp),
        .EX_pc(EX_pc),
        .EX_reg1(EX_reg1),
        .EX_reg2(EX_reg2),
        .EX_imm(EX_imm),
        .EX_rd(EX_rd),
        .EX_funct3(EX_funct3),
        .EX_funct7(EX_funct7),
        .EX_rs1(EX_rs1),
        .EX_rs2(EX_rs2)
    );

    // ALU Control
    wire [3:0] alu_control_signal;
    alu_control ALU_CTRL(
        .ALUOp(EX_ALUOp),
        .func3(EX_funct3),
        .func7(EX_funct7),
        .ALUControl(alu_control_signal)
    );

    ForwardingUnit forward(
        .RS_1(EX_rs1),
        .RS_2(EX_rs2),
        .rdMem( MEM_rd),
        .rdWb(WB_rd),
        .regWrite_Wb(WB_RegWrite),
        .regWrite_Mem(MEM_RegWrite),
        .Forward_A(forwardA),
        .Forward_B(forwardB)
    );

    wire [63:0] ALU_A;
    assign ALU_A = (forwardA == 2'b10) ? MEM_alu_result :
                   (forwardA == 2'b01) ? WB_read_data :
                   EX_reg1;
                   
    // Select ALU operand B using forwarding and ALUSrc mux
    wire [63:0] forwarded_B;
    assign forwarded_B = (forwardB == 2'b10) ? MEM_alu_result :
                         (forwardB == 2'b01) ? WB_read_data :
                         EX_reg2;

    // EX Stage
    wire [63:0] alu_b = EX_ALUSrc ? EX_imm : forwarded_B;
    ALU alu(
        .control(alu_control_signal),
        .A(ALU_A),
        .B(alu_b),
        .result(alu_result),
        .Zero(zero)
    );
  wire [63:0] actual_target_MEM;
    // EX/MEM Pipeline
    EX_MEM_reg EX_MEM(
        .clk(clk),
        .reset(reset),
        .flush(flush),
        .EX_RegWrite(EX_RegWrite),
        .EX_MemtoReg(EX_MemtoReg),
        .EX_MemRead(EX_MemRead),
        .EX_MemWrite(EX_MemWrite),
        .EX_alu_result(alu_result),
        .branch_taken_EX  (branch_taken_EX),
        .EX_reg2(EX_reg2),
        .EX_rd(EX_rd),
        .EX_Zero(zero),
        .MEM_RegWrite(MEM_RegWrite),
        .MEM_MemtoReg(MEM_MemtoReg),
        .MEM_MemRead(MEM_MemRead),
        .MEM_MemWrite(MEM_MemWrite),
        .MEM_alu_result(MEM_alu_result),
        .MEM_reg2(MEM_reg2),
        .MEM_rd(MEM_rd),
        .MEM_Zero(MEM_Zero),
        .branch_taken_MEM (branch_taken_MEM),
        .actual_target_MEM(actual_target_MEM),
        .MEM_Branch(MEM_Branch),
        .EX_Branch(EX_Branch),
        .actual_target_EX(actual_target_EX)
    );

    // MEM Stage
    data_memory DM(
        .clk(clk),
        .reset(reset),
        .memWrite(MEM_MemWrite),
        .memRead(MEM_MemRead),
        .address(MEM_alu_result),
        .write_data(MEM_reg2),
        .read_data(mem_data)
    );

    // MEM/WB Pipeline
    MEM_WB_reg MEM_WB(
        .clk(clk),
        .reset(reset),
        .MEM_RegWrite(MEM_RegWrite),
        .MEM_MemtoReg(MEM_MemtoReg),
        .MEM_read_data(mem_data),
        .MEM_alu_result(MEM_alu_result),
        .MEM_rd(MEM_rd),
        .WB_RegWrite(WB_RegWrite),
        .WB_MemtoReg(WB_MemtoReg),
        .WB_read_data(WB_read_data),
        .WB_alu_result(WB_alu_result),
        .WB_rd(WB_rd)
    );

    // WB Stage
    assign wb_data = WB_MemtoReg ? WB_read_data : WB_alu_result;


  always @(*) begin
    flush = 0;
    if (branch_taken_MEM) begin
      flush = 1;
    end
  end



endmodule

// module processor_tb;
//     reg clk, reset;
//     processor_pipeline uut(clk, reset);

//     initial begin
//         clk = 0;
//         reset = 1;
//         #10 reset = 0;
//         forever #5 clk = ~clk;
//     end
    
//     initial begin
//         #200;
//         $finish;
//     end
// endmodule