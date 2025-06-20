module instruction_memory (
    input [63:0] PC,
    output  [31:0] instruction
);
    reg [31:0] memory[0:63];
    initial begin
    $readmemb("instructions.txt", memory);
  end
   assign instruction=memory[PC[63:0]];
endmodule