module hazard_detection_unit (
  // From the EX stage (or earlier) indicating a load instruction:
  input  wire        MemRead_EX,
  // Destination register of the instruction in the EX stage:
  input  wire [4:0]  Rd_EX,
  // Structural hazard signal (e.g. resource conflict like memory port contention)
  input  wire        structural_hazard,
  // Branch control signal (e.g. if a branch is determined as taken)
  input  wire        branch,
  // Instruction in the IF/ID stage to check for register dependencies:
  input  wire [31:0] IFID_inst,
  
  // Output control signals to stall or flush the pipeline:
  output reg         stall,

    output reg       if_id_WriteEnable,
    output reg       pc_WriteEnable
);

  // Extract source registers from the IF/ID instruction
  // (Assuming RISC–V, where bits [19:15] and [24:20] are rs1 and rs2)
  wire [4:0] rs1_ID;
  wire [4:0] rs2_ID;
  
  assign rs1_ID = IFID_inst[19:15];
  assign rs2_ID = IFID_inst[24:20];

  always @(*) begin
    // Default: no stall or flush
    stall = 1'b0;
    // flush = 1'b0;
      if_id_WriteEnable = 1;
    pc_WriteEnable = 1;
    // Data Hazard: load-use hazard detection.
    // If the instruction in the EX stage is a load and its destination register
    // is needed by the instruction in the IF/ID stage, then stall.
    if (MemRead_EX && ((Rd_EX == rs1_ID) || (Rd_EX == rs2_ID))) begin
      stall = 1'b1;
       if_id_WriteEnable = 0;
      pc_WriteEnable = 0;
    end
   

    // Structural Hazard: if a resource conflict is detected,
    // stall the pipeline regardless of other hazards.
    // if (structural_hazard) begin
    //   stall = 1'b1;
    // end

    // Control Hazard: if a branch instruction is taken,
    // then flush the instruction in the IF/ID stage.
    // (Often, a stall might also be inserted to give time for branch resolution.)
    
  end

endmodule
