`timescale 1ns / 1ps
//`include "bhuvi.v"  // Ensure that final.v includes the processor module and all necessary modules

module final_tb;
    reg clk, reset;
    
    // Instantiate the processor
    processor_pipeline0 uut (
        .clk(clk),
        .reset(reset)
    );
    
    initial begin
        // Initialize signals
        #6;
        clk=1;
    // Clock generation: 10ns period => 100MHz clock
        forever #5 clk = ~clk;
    
        // Apply reset for 10ns, then release it
        
        
    end
    
    // Monitor key signals for debugging:
    // We are accessing some internal signals using hierarchical references.
    // Adjust the names if your module hierarchy differs.
    // initial begin
    //     $monitor("Time=%0t | PC=%h | IF_inst_reg=%h | ID_EX_rd=%h", 
    //              $time, uut.PC, uut.IFID_inst_reg, uut.ID_EX_rd);
    // end
    
    // Dump waveforms for analysis in a waveform viewer.
    initial begin
        $dumpfile("final_tb.vcd");
        $dumpvars(0, final_tb);

        reset = 1;
        uut.PC=0;
        #10;
        reset=0;
        // Run simulation for a while
        #200;
        
        // End simulation
        $finish;
    end

endmodule
