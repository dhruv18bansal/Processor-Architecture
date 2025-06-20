module alu_control(
    input  [1:0] ALUOp,
    input  [2:0] func3,
    input  [6:0] func7,  
    output reg [3:0] ALUControl
);
    wire func7_5 = func7[5];
    
    always @(*) begin
        case (ALUOp)
            2'b00: ALUControl = 4'b0010;
            2'b01: ALUControl = 4'b0110;
            2'b10: begin
                case (func3)
                    3'b000: begin
                        if (func7_5 == 1'b1)
                            ALUControl = 4'b0110;
                        else
                            ALUControl = 4'b0010;
                    end
                    3'b111: ALUControl = 4'b0000;
                    3'b110: ALUControl = 4'b0001;
                    default: ALUControl = 4'b1111;
                endcase
            end
            default: ALUControl = 4'b1111;
        endcase
    end
endmodule


