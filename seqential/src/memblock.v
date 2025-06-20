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
     always @(posedge clk) begin
        if (reset) begin
            for (i = 0; i < 1024; i = i + 1) begin
                memory[i] <= 64'b0;
            end
        end 
       
    end
     always @(*) begin
        read_data = (memRead == 0) ? 64'b0 : memory[address];  
    end
    // always @(*) begin
    //     if (memRead) 
    //         read_data <= memory[address];
    // end
    
    always @(posedge clk) begin
        if (memWrite) 
            memory[address] = write_data;
    end
    
endmodule