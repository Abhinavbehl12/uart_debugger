// Copyright 2024 Dolphin Design
// SPDX-License-Identifier: Apache-2.0 WITH SHL-2.1
//
// Licensed under the Solderpad Hardware License v 2.1 (the "License");
// you may not use this file except in compliance with the License, or,
// at your option, the Apache License version 2.0.
// You may obtain a copy of the License at
//
// https://solderpad.org/licenses/SHL-2.1/
//
// Unless required by applicable law or agreed to in writing, any work
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/////////////////////////////////////////////////////////////////////////////
//                                                                         //
// Contributors: Pascal Gouedo, Dolphin Design <pascal.gouedo@dolphin.fr>  //
//                                                                         //
// Description:  Top level module of CV32E40P instantiating the Core and   //
//               an optional CVFPU with its clock gating cell.             //
//                                                                         //
/////////////////////////////////////////////////////////////////////////////

module cv32e40p_fpga_top #(
    parameter COREV_PULP = 0, // PULP ISA Extension (incl. custom CSRs and hardware loop, excl. cv.elw)
    parameter COREV_CLUSTER = 0,  // PULP Cluster interface (incl. cv.elw)
    parameter FPU = 0,  // Floating Point Unit (interfaced via APU interface)
    parameter FPU_ADDMUL_LAT = 0,  // Floating-Point ADDition/MULtiplication computing lane pipeline registers number
    parameter FPU_OTHERS_LAT = 0,  // Floating-Point COMParison/CONVersion computing lanes pipeline registers number
    parameter ZFINX = 0,  // Float-in-General Purpose registers
    parameter DEBUGGER_INSTANCE = 0,  // Adding debugger instance into the RISC-V
    parameter NUM_MHPMCOUNTERS = 1
) (
    // Clock and Reset
    input   wire clk_25mhz,
    input   wire clk_i,
    input   wire rst_i,

 /*    input   wire pulp_clock_en_i,  // PULP clock enable (only used if COREV_CLUSTER = 1)
    input   wire scan_cg_en_i,  // Enable all clock gates for testing

    // Core ID, Cluster ID, debug mode halt address and boot address are considered more or less static
    input   wire [31:0] boot_addr_i,
    input   wire [31:0] mtvec_addr_i,
    input   wire [31:0] dm_halt_addr_i,
    input   wire [31:0] hart_id_i,
    input   wire [31:0] dm_exception_addr_i,

    // Instruction memory interface
    output   wire        instr_req_o,
    input    wire        instr_gnt_i,
    input    wire        instr_rvalid_i,
    output   wire [31:0] instr_addr_o,
    input    wire [31:0] instr_rdata_i,

    // Data memory interface
    output   wire        data_req_o,
    input    wire        data_gnt_i,
    input    wire        data_rvalid_i,
    output   wire        data_we_o,
    output   wire [ 3:0] data_be_o,
    output   wire [31:0] data_addr_o,
    output   wire [31:0] data_wdata_o,
    input    wire [31:0] data_rdata_i,

    // Interrupt inputs
    input    wire [31:0] irq_i,  // CLINT interrupts + CLINT extension interrupts
    output   wire        irq_ack_o,
    output   wire [ 4:0] irq_id_o, 

    // Debug Interface
    input    wire debug_req_i,
    output   wire debug_havereset_o,
    output   wire debug_running_o,
    output   wire debug_halted_o,

    // CPU Control Signals
    input    wire fetch_enable_i,
    output   wire core_sleep_o, */
	
	//UART interface
	input uart_rx ,
	output uart_tx ,
	output uart_busy,
	
	//LED interface
	
	output user_led0 ,
	output user_led1 ,
	output user_led2 ,
	output user_led3 ,
	output user_led4 ,
	output user_led5 ,
	output user_led6 
	
);

   // wire clk_25mhz;
    reg rst_ni;
  // Core to FPU
    wire                              apu_busy;
    wire                              apu_req;
    //wire [   APU_NARGS_CPU-1:0][31:0] apu_operands;
    //wire [     APU_WOP_CPU-1:0]       apu_op;
    //wire [APU_NDSFLAGS_CPU-1:0]       apu_flags;

  // FPU to Core
    wire                              apu_gnt;
    wire                              apu_rvalid;
    wire [                31:0]       apu_rdata;
    //wire [APU_NUSFLAGS_CPU-1:0]       apu_rflags;

    wire apu_clk_en, apu_clk;

	//REGISTER INTERFACE - COnnected to debugger module
    wire [7:0]reg_addr;   
    wire [31:0]reg_wr_data;
    wire  [31:0]reg_rd_data;
    wire reg_wr_en  ;
    wire reg_rd_en ;
    wire  reg_rd_done;
    wire [31:0]reg_rd_data_debugger;
    wire [31:0]reg_rd_data_risc;
    wire reg_rd_done_debugger;
    wire reg_rd_done_risc;
  // signals connecting core to memory
    wire                         instr_req;
    wire                         instr_gnt;
    wire                         instr_rvalid;
    wire [31:0]                  instr_addr;
    wire [31:0]                  instr_rdata;
    wire risc_rd_wr_cmd;
    wire fpga_rd_wr_cmd;
    wire debugger_rd_wr_cmd;
    wire start_test;
    wire pll_locked;
 localparam  BOOT_ADDR         = 'h00;
 localparam  DM_HALTADDRESS    = 32'h1A11_0800;
 localparam  HART_ID           = 32'h0000_0000;
// assign rst_ni = rst_i;
 
  always @(posedge clk_25mhz or negedge rst_i) begin
	    if (rst_i) begin
	        rst_ni <= 1'd1; 
        end else begin
             if(!pll_locked)
                 rst_ni <= 1'b1;
             else 
                 rst_ni <= 1'b0;
        end
  end       
 
  // Instantiate the Core
  cv32e40p_core #(
      .COREV_PULP      (COREV_PULP),
      .COREV_CLUSTER   (COREV_CLUSTER),
      .FPU             (FPU),
      .FPU_ADDMUL_LAT  (FPU_ADDMUL_LAT),
      .FPU_OTHERS_LAT  (FPU_OTHERS_LAT),
      .ZFINX           (ZFINX),
      .NUM_MHPMCOUNTERS(NUM_MHPMCOUNTERS)
  ) core_i (
      .clk_i (clk_i),
      .rst_ni(rst_ni),

      .pulp_clock_en_i(1'b0),
      .scan_cg_en_i   (1'b0),

         .boot_addr_i            ( BOOT_ADDR             ),
         .dm_halt_addr_i         ( DM_HALTADDRESS        ),
         .hart_id_i              ( HART_ID               ),
      .mtvec_addr_i       ('d0),
      .dm_exception_addr_i('d0),

      // .instr_req_o   (instr_req_o),
      // .instr_gnt_i   (instr_gnt_i),
      // .instr_rvalid_i(instr_rvalid_i),
      // .instr_addr_o  (instr_addr_o),
      // .instr_rdata_i (instr_rdata_i),
      .instr_req_o            ( instr_req             ),
      .instr_gnt_i            ( instr_gnt             ),
      .instr_rvalid_i         ( instr_rvalid          ),
      .instr_addr_o           ( instr_addr            ),
      .instr_rdata_i          ( instr_rdata           ),

      .data_req_o   ( ),
      // .data_gnt_i   (data_gnt_i),
      .data_gnt_i   (1'b0),
      .data_rvalid_i(1'b0),
      .data_we_o    (),
      .data_be_o    (),
      .data_addr_o  (),
      .data_wdata_o (),
      // .data_rdata_i (data_rdata_i),
      .data_rdata_i ('d0),

      .apu_req_o              (                       ),
      .apu_gnt_i              ( 1'b0                  ),
      .apu_operands_o         (                       ),
      .apu_op_o               (                       ),
      .apu_flags_o            (                       ),
      .apu_rvalid_i           ( 1'b0                  ),
      .apu_result_i           ( {32{1'b0}}            ),
      .apu_flags_i            ( {5{1'b0}}             ),

      .irq_i    ({32{1'b0}}),
      .irq_ack_o(),
      .irq_id_o (),

      .debug_req_i      (),
      .debug_havereset_o(),
      .debug_running_o  (),
      .debug_halted_o   (),

      .fetch_enable_i(1'b1),//start the instruction fetch
      .core_sleep_o  ()
	  
	  //Register interface for debugger
      ,.reg_addr          (reg_addr   )
      ,.reg_wr_data       (reg_wr_data)
      ,.reg_rd_data       (reg_rd_data_risc)
      ,.reg_wr_en         (risc_rd_wr_cmd && reg_wr_en  )
      ,.reg_rd_en         (risc_rd_wr_cmd && reg_rd_en  )
      ,.reg_rd_done       (reg_rd_done_risc)
  );
assign reg_rd_data = reg_rd_done_risc ? reg_rd_data_risc : (reg_rd_done_debugger ?  reg_rd_data_debugger : 32'd0);
assign reg_rd_done = reg_rd_done_risc | reg_rd_done_debugger;
//////////////////////////////////////////////////////////////
//UART Debugger instance
//////////////////////////////////////////////////////////////

reg_intf_via_uart u_reg_intf_via_uart (
.core_clk           (clk_25mhz)//25 Mhz clock
,.core_rst          (!rst_ni)
,.uart_rx           (uart_rx)
,.uart_tx           (uart_tx)
,.uart_busy         (uart_busy)
,.reg_addr          (reg_addr   )
,.reg_wr_data       (reg_wr_data)
,.reg_rd_data       (reg_rd_data)
,.reg_wr_en         (reg_wr_en  )
,.reg_rd_en         (reg_rd_en  )
,.reg_rd_done       (reg_rd_done)
,.start_test        (start_test)
,.test_command      ()
,.test_result       ()
,.test_result_valid ()
,.debug_bus_u       ()
,.risc_rd_wr_cmd    (risc_rd_wr_cmd)
,.fpga_rd_wr_cmd    (fpga_rd_wr_cmd)
,.debugger_rd_wr_cmd(debugger_rd_wr_cmd)
);
//////////////////////////////////////////////////////////////
//UART Memory instance
//////////////////////////////////////////////////////////////

mem_intf_via_uart  #(
.ADDR_WIDTH (8)
)u_mem_intf_via_uart(
.core_clk           (clk_i)//25 Mhz clock
,.core_clk_25Mhz    (clk_25mhz)//25 Mhz clock
,.rst_n             (rst_ni)
,.reg_addr          (reg_addr   )
,.reg_wr_data       (reg_wr_data)
,.reg_rd_data       (reg_rd_data_debugger)
,.reg_wr_en         (debugger_rd_wr_cmd && reg_wr_en  )
,.reg_rd_en         (debugger_rd_wr_cmd && reg_rd_en  )
,.reg_rd_done       (reg_rd_done_debugger)
,.start_test        (start_test),

.instr_req_i    ( instr_req        ),
.instr_addr_i   ( instr_addr[8-1:0]),
.instr_rdata_o  ( instr_rdata      ),
.instr_rvalid_o ( instr_rvalid     ),
.instr_gnt_o    ( instr_gnt        )

);
//PLL Instance
 /*clk_wiz_0 clock_25mhz_pll
   (
    // Clock out ports
    .clk_out1(clk_25mhz),     // output clk_out1
    // Status and control signals
    .reset(!rst_ni), // input reset
    .locked(pll_locked),       // output locked
   // Clock in ports
    .clk_in1(clk_i));      // input clk_in1
*/


endmodule
