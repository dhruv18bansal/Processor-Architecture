// `timescale 1ns / 1ps
`include "processortop.v"

module processor_tb;

  // Inputs
  reg clk;
  reg reset;

  // Instantiate the CPU
  processor uut (
      .clk  (clk),
      .reset(reset)
  );
  
  // Clock generation
  initial begin
    #6;
    clk = 1;
    forever #5 clk = ~clk;  // 10ns period clock
  end

  // Initialize and dump waveforms
  initial begin
    // Create waveform file
    $dumpfile("processor_sim.vcd");
    $dumpvars(0, processor_tb);  // Dump all variables
    // reset = 0;
    // #5;
    // Initialize inputs
    reset = 1;
    #10;  // Wait for first clock edge (10ns)
    reset = 0;  // Release reset after 10ns
    #200;
    $finish;
  end

endmodule