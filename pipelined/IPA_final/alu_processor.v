module full_adder(input A, B, cin, output sum, cout);
  assign sum = A ^ B ^ cin;
  assign cout = (A & B) | (B & cin) | (A & cin);
endmodule

module bitwise_not(input [63:0] A, output [63:0] Y);
  assign Y = ~A;
endmodule

module adder(input signed [63:0] A, B, input cin, output [63:0] sum, output cout);
  wire [63:0] carry;
  genvar i;
  generate
    for (i = 0; i < 64; i = i + 1) begin: FA
      if (i == 0)
        full_adder FA (A[i], B[i], cin, sum[i], carry[i]);
      else
        full_adder FA (A[i], B[i], carry[i-1], sum[i], carry[i]);
    end
  endgenerate
  assign cout = carry[63];
endmodule

module subtractor_64bit(input signed [63:0] A, B, output [63:0] diff, output cout);
  wire [63:0] B_neg;
  bitwise_not B_INV (B, B_neg);
  adder ADD (A, B_neg, 1'b1, diff, cout);
endmodule

module bitwise_and(input [63:0] A, B, output [63:0] Y);
  genvar i;
  generate
    for (i = 0; i < 64; i = i + 1) begin: gen_and_gates
      and a1 (Y[i], A[i], B[i]);
    end
  endgenerate
endmodule

module bitwise_or(input [63:0] A, B, output [63:0] Y);
  genvar i;
  generate
    for (i = 0; i < 64; i = i + 1) begin: gen_or_gates
      or o1 (Y[i], A[i], B[i]);
    end
  endgenerate
endmodule



module ALU(
    input signed [63:0] A, B,
    input [3:0] control,
    output reg signed [63:0] result,
    output reg Zero
);
  wire [63:0] sum, diff, and_out, or_out;
  wire cout;
  
  adder ADD (A, B, 1'b0, sum, cout);
  subtractor_64bit SUB (A, B, diff, cout);
  bitwise_and AND_ (A, B, and_out);
  bitwise_or OR_ (A, B, or_out);
  
  always @(*) begin
    case (control)
      4'b0010: result = sum;       
      4'b0110: result = diff;      
      4'b0000: result = and_out;   
      4'b0001: result = or_out;     
      default: result = 64'b0;
    endcase
    
    Zero = (result == 64'b0) ? 1'b1 : 1'b0;
  end
endmodule

