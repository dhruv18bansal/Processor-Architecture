module data_memory (
    input clk,
    input memWrite,
    input memRead,
    input reset,
    input [63:0] address,
    input [63:0] write_data,
    output reg [63:0] read_data
);
    reg [63:0] memory [0:1023];
    integer i;

    // Initialize memory on reset
    always @(posedge clk) begin
        if (reset) begin
            for (i = 0; i < 1024; i = i + 1) begin
                memory[i] <= 64'b0;
            end
            memory[2] <= 64'b1111; // Example initialization
        end 
    end

    // Read logic (combinational)
    always @(*) begin
        if (memRead) 
            read_data = memory[address]; // Use blocking assignment (=)
        else
            read_data = 64'b0; // Or set to high-impedance (64'bz)
    end

    // Write logic (sequential)
    always @(posedge clk) begin
        if (memWrite) 
            memory[address] <= write_data;
    end

endmodule