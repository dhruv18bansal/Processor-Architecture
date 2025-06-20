module register_file (
    input clk,
    input reset,
    input regWrite,
    input [4:0] read_reg1,
    input [4:0] read_reg2,
    input [4:0] write_reg,
    input [63:0] write_data,
    output reg [63:0] read_data1,
    output reg [63:0] read_data2
);
    reg [63:0] registers [0:31];
    integer i;

    // Synchronous Reset and Write
    always @(posedge clk) begin
        if (reset) begin
            for (i = 0; i < 32; i = i + 1) begin
                registers[i] <= 64'b0;
            end
            registers[3] <= {62'b0, 2'b11};  // Initialize specific registers
            registers[2] <= {62'b0, 2'b10};
        end 
        else if (regWrite && write_reg != 0) begin
            registers[write_reg] <= write_data;
        end
    end

    // Combinational Read
    always @(*) begin
        read_data1 = (read_reg1 == 0) ? 64'b0 : registers[read_reg1];  
    end

    always @(*) begin
        read_data2 = (read_reg2 == 0) ? 64'b0 : registers[read_reg2];
    end

endmodule

module mux(
    input [63:0] in0,
    input [63:0] in1,
    input select,
    output [63:0] out
);
    assign out = select ? in1 : in0;
endmodule