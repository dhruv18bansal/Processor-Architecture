module ControlUnit(
    input [5:0] Op,  
    output RegWrite, MemtoReg, MemRead, MemWrite, Branch, ALUSrc,
    output [1:0] ALUOp
);
    wire R_format, lw, sw, beq;
    
    // Here we are decoding the Op fields that we are given
    and (R_format, ~Op[5], ~Op[4], ~Op[3], ~Op[2], ~Op[1], ~Op[0]); // 000000 ----- THIS WE HAVE TAKEN FOR R-Type
    and (lw, Op[5], ~Op[4], ~Op[3], ~Op[2], Op[1], Op[0]);         //  100011 ----- THIS WE HAVE TAKEN FOR LW-Type
    and (sw, Op[5], ~Op[4], Op[3], ~Op[2], Op[1], Op[0]);          //  101011 ----- THIS WE HAVE TAKEN FOR SW-Type
    and (beq, ~Op[5], ~Op[4], ~Op[3], Op[2], ~Op[1], ~Op[0]);      //  000100 ----- THIS WE HAVE TAKEN FOR BEQ-Type
    
    assign RegWrite = R_format | lw;        
    assign MemtoReg = lw;                   
    assign MemRead = lw;                   
    assign MemWrite = sw;                   
    assign Branch = beq;                  
    assign ALUSrc = lw | sw;               
    assign ALUOp[1] = R_format;          
    assign ALUOp[0] = beq;               
    
endmodule

