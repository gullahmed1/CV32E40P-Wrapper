// Copyright 2018 ETH Zurich and University of Bologna.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.

// Wrapper for a cv32e40p core and optional FPU
// Contributor: Davide Schiavone <davide@openhwgroup.org>
module cv32e40p_wrapper #(
    parameter PULP_XPULP       = 0, // PULP ISA Extension (incl. custom CSRs and hardware loop, excl. p.elw)
    parameter PULP_CLUSTER = 0,  // PULP Cluster interface (incl. p.elw)
    parameter FPU = 0,  // Floating Point Unit (interfaced via APU interface)
    parameter FPU_ADDMUL_LAT = 0,  // Floating-Point ADDition/MULtiplication computing lane pipeline registers number
    parameter FPU_OTHERS_LAT = 0,  // Floating-Point COMParison/CONVersion computing lanes pipeline registers number
    parameter PULP_ZFINX = 0,  // Float-in-General Purpose registers
    parameter NUM_MHPMCOUNTERS = 1
) (
    // Clock and Reset
    /*input logic clk_i,
    input logic rst_ni,*/
input rst, 
input clk,						// 1 button to reset, clock signal as input
//output alu_z;						// An LED turned ON if result is zero
output reg[7:0] Anode_Activate,		// Anodes to control 7-segments
output reg [6:0] LED_out,			// output result to be sent on 7-segments

   /*  input logic pulp_clock_en_i,  // PULP clock enable (only used if PULP_CLUSTER = 1)
    input logic scan_cg_en_i,  // Enable all clock gates for testing

   // Core ID, Cluster ID, debug mode halt address and boot address are considered more or less static
    input logic [31:0] boot_addr_i,
    input logic [31:0] mtvec_addr_i,
    input logic [31:0] dm_halt_addr_i,
    input logic [31:0] hart_id_i,
    input logic [31:0] dm_exception_addr_i, */

    // Instruction memory interface
    output logic        instr_req_o,
    input  logic        instr_gnt_i,
    input  logic        instr_rvalid_i,
    output logic [31:0] instr_addr_o,
    input  logic [31:0] instr_rdata_i,

    // Data memory interface
    output logic        data_req_o,
    input  logic        data_gnt_i,
    input  logic        data_rvalid_i,
    output logic        data_we_o,
    output logic [ 3:0] data_be_o,
    output logic [31:0] data_addr_o,
    output logic [31:0] data_wdata_o,
    input  logic [31:0] data_rdata_i,

  /*  // Interrupt inputs
    input  logic [31:0] irq_i,  // CLINT interrupts + CLINT extension interrupts
    output logic        irq_ack_o,
    output logic [ 4:0] irq_id_o,

    // Debug Interface
    input  logic debug_req_i,
    output logic debug_havereset_o,
    output logic debug_running_o,
    output logic debug_halted_o, */

    // CPU Control Signals
    input  logic fetch_enable_i,
  //  output logic core_sleep_o
);
wire [31:0] pc;
wire new_clk;
wire [31:0] instruction,out;
main f4 (.clk(clk),.rst(rst),.Result(out));

	// ALL modules will be called in this file. Code will be executed and results will be shown on 7-segment display
// Code segment for BCD to 7-segment Decoder. Keep this code as it is
reg [31:0] counter;		// A 32 bit flashing counter
reg toggle;			// A variable to toggle between two 7-segments 
wire [7:0]Result;
assign Result[7:0]= 12;//out[7:0];
wire [4:0]first_four_bits = out[3:0];
wire [4:0]next_four_bits = out[7:4];
// first_four_bits = out[3:0];
// next_four_bits = out[7:4];

always @(posedge clk)
    begin
        if(!rst) begin
            if(counter>=100000) begin
                 counter <= 0;
				 toggle = ~toggle; end
            else begin
                counter <= counter + 1;
				end
		end
		else begin
		toggle<=0;
		  counter<=0;
		end
    end 
    // anode activating signals for 8 segments, digit period of 1ms
    // decoder to generate anode signals 
    always @(*)
    begin
        
            case(toggle)
            1'b0: begin
                Anode_Activate = 8'b01111111; 
                // activate SEGMENT1 and Deactivate all others
                  end
            1'b1: begin
                Anode_Activate = 8'b10111111; 
                // activate LED2 and Deactivate all others    
                   end
            endcase


    end
    // Cathode patterns of the 7-segment 1 LED display 
    always @(*)
    begin
	if (toggle) begin
        case(out)				// First 4 bits of Result from ALU will be displayed on 1st segment
        32'b0000: LED_out = 7'b0000001; // "0"     
        32'b0001: LED_out = 7'b1001111; // "1" 
        32'b0010: LED_out = 7'b0010010; // "2" 
        32'b0011: LED_out = 7'b0000110; // "3" 
        32'b0100: LED_out = 7'b1001100; // "4" 
        32'b0101: LED_out = 7'b0100100; // "5" 
        32'b0110: LED_out = 7'b0100000; // "6" 
        32'b0111: LED_out = 7'b0001111; // "7" 
        32'b1000: LED_out = 7'b0000000; // "8"     
        32'b1001: LED_out = 7'b0000100; // "9"
		32'b1010: LED_out = 7'b0001000; // "A"     
        32'b1011: LED_out = 7'b1100000; // "b"     
        32'b1100: LED_out = 7'b0110001; // "C"     
        32'b1101: LED_out = 7'b1000010; // "d"     
        32'b1110: LED_out = 7'b0110000; // "E"     
        32'b1111: LED_out = 7'b0111000; // "F"     
        
        //default: LED_out = 7'b0000001; // "0"
        endcase

	end
    

	// Cathode patterns of the 7-segment 2 LED display
if(!toggle) begin	
        case(out[7:4])			// Next 4 bits of Result from ALU will be displayed on 2nd segment
        4'b0000: LED_out = 7'b0000001; // "0"     
        4'b0001: LED_out = 7'b1001111; // "1" 
        4'b0010: LED_out = 7'b0010010; // "2" 
        4'b0011: LED_out = 7'b0000110; // "3" 
        4'b0100: LED_out = 7'b1001100; // "4" 
        4'b0101: LED_out = 7'b0100100; // "5" 
        4'b0110: LED_out = 7'b0100000; // "6" 
        4'b0111: LED_out = 7'b0001111; // "7" 
        4'b1000: LED_out = 7'b0000000; // "8"     
        4'b1001: LED_out = 7'b0000100; // "9"
		4'b1010: LED_out = 7'b0001000; // "A"     
        4'b1011: LED_out = 7'b1100000; // "b"     
        4'b1100: LED_out = 7'b0110001; // "C"     
        4'b1101: LED_out = 7'b1000010; // "d"     
        4'b1110: LED_out = 7'b0110000; // "E"     
        4'b1111: LED_out = 7'b0111000; // "F"     
        
        default: LED_out = 7'b0000011; // "0"
        endcase

    end
end

  import cv32e40p_apu_core_pkg::*;

 /* // Core to FPU
  logic                              apu_req;
  logic [   APU_NARGS_CPU-1:0][31:0] apu_operands;
  logic [     APU_WOP_CPU-1:0]       apu_op;
  logic [APU_NDSFLAGS_CPU-1:0]       apu_flags;

  // FPU to Core
  logic                              apu_gnt;
  logic                              apu_rvalid;
  logic [                31:0]       apu_rdata;
  logic [APU_NUSFLAGS_CPU-1:0]       apu_rflags;*/

  // Instantiate the Core
  cv32e40p_core #(
      .PULP_XPULP      (PULP_XPULP),
      .PULP_CLUSTER    (PULP_CLUSTER),
      .FPU             (FPU),
      .FPU_ADDMUL_LAT  (FPU_ADDMUL_LAT),
      .FPU_OTHERS_LAT  (FPU_OTHERS_LAT),
      .PULP_ZFINX      (PULP_ZFINX),
      .NUM_MHPMCOUNTERS(NUM_MHPMCOUNTERS)
  ) core_i (
      .clk_i (clk),
      .rst_ni(rst),
/*
      .pulp_clock_en_i(pulp_clock_en_i),
      .scan_cg_en_i   (scan_cg_en_i),

      .boot_addr_i        (boot_addr_i),
      .mtvec_addr_i       (mtvec_addr_i),
      .dm_halt_addr_i     (dm_halt_addr_i),
      .hart_id_i          (hart_id_i),
      .dm_exception_addr_i(dm_exception_addr_i), */

      .instr_req_o   (instr_req_o),
      .instr_gnt_i   (instr_gnt_i),
      .instr_rvalid_i(instr_rvalid_i),
      .instr_addr_o  (instr_addr_o),
      .instr_rdata_i (instr_rdata_i),

      .data_req_o   (data_req_o),
      .data_gnt_i   (data_gnt_i),
      .data_rvalid_i(data_rvalid_i),
      .data_we_o    (data_we_o),
      .data_be_o    (data_be_o),
      .data_addr_o  (data_addr_o),
      .data_wdata_o (data_wdata_o),
      .data_rdata_i (data_rdata_i),
/*
      .apu_req_o     (apu_req),
      .apu_gnt_i     (apu_gnt),
      .apu_operands_o(apu_operands),
      .apu_op_o      (apu_op),
      .apu_flags_o   (apu_flags),
      .apu_rvalid_i  (apu_rvalid),
      .apu_result_i  (apu_rdata),
      .apu_flags_i   (apu_rflags),

      .irq_i    (irq_i),
      .irq_ack_o(irq_ack_o),
      .irq_id_o (irq_id_o),

      .debug_req_i      (debug_req_i),
      .debug_havereset_o(debug_havereset_o),
      .debug_running_o  (debug_running_o),
      .debug_halted_o   (debug_halted_o), */

      .fetch_enable_i(fetch_enable_i),
     // .core_sleep_o  (core_sleep_o)
  );
/*
  generate
    if (FPU) begin : fpu_gen
      // Instantiate the FPU wrapper
      cv32e40p_fp_wrapper #(
          .FPU_ADDMUL_LAT(FPU_ADDMUL_LAT),
          .FPU_OTHERS_LAT(FPU_OTHERS_LAT)
      ) fp_wrapper_i (
          .clk_i         (clk_i),
          .rst_ni        (rst_ni),
          .apu_req_i     (apu_req),
          .apu_gnt_o     (apu_gnt),
          .apu_operands_i(apu_operands),
          .apu_op_i      (apu_op),
          .apu_flags_i   (apu_flags),
          .apu_rvalid_o  (apu_rvalid),
          .apu_rdata_o   (apu_rdata),
          .apu_rflags_o  (apu_rflags)
      );
    end else begin : no_fpu_gen
      // Drive FPU output signals to 0
      assign apu_gnt    = '0;
      assign apu_rvalid = '0;
      assign apu_rdata  = '0;
      assign apu_rflags = '0;
    end
  endgenerate */

endmodule
