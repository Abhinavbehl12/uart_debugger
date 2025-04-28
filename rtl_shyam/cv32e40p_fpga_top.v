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

///////////////////////////////////////////////////////////////////////////
//TXD 26 D Out Asynchronous data output (UART Transmit)
//RXD 25 D In Asynchronous data input (UART Receive)
//CTS 23* D In Clear To Send control input (active low)
//RTS 24* D Out Ready to Send control output (active low)
//DSR 27* D in Data Set Ready control input (active low)
//DTR 28* D Out Data Terminal Ready control output (active low)
//DCD 1* D In Data Carrier Detect control input (active low)
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
    input pll_locked,

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
	
	output reg user_led0 ,
	output user_led1 ,
	output reg user_led2 ,
	output user_led3 ,
	output user_led4 ,
	output user_led5 ,
	output user_led6 ,
	
	//DEbug interface
	//output [31:0]debug_bus,
	
	//Push_buttn interface
	input pb0 ,
	input pb1 ,
	input pb2 ,
	input pb3 ,

	//SW interface
	input sw0 ,
	input sw1 ,
	input sw2 ,
	input sw3 ,
	input sw4 ,
	input sw5 ,
	input sw6 ,
	input sw7 
	
);

//    wire clk_25mhz;
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
//    wire pll_locked;
    wire [4:0]reg_intf_cur_state;
    wire [31:0]debug_bus_u;
    wire [31:0] debug_bus__reg_int;
    wire uart_tx_int;
    wire [31:0] debug_bus;
 localparam  BOOT_ADDR         = 'h00;
 localparam  DM_HALTADDRESS    = 32'h1A11_0800;
 localparam  HART_ID           = 32'h0000_0000;
// assign rst_ni = rst_i;
 
  always @(posedge clk_25mhz or negedge rst_i) begin
	    if (rst_i) begin
	        rst_ni <= 1'd0; 
	        user_led0 <= 1'd1; 
        end else begin
             if(!pll_locked)
                 rst_ni <= 1'b0;
             else 
                 rst_ni <= 1'b1;
                 
            user_led0 <= sw0;     
            if(sw5)
               user_led0 <= reg_intf_cur_state[0];
        end
  end       
 
 assign user_led1 = sw5 ? reg_intf_cur_state[1] :sw1;
 
 
  always @(posedge clk_i or negedge rst_i) begin
	    if (rst_i) begin
	        user_led2 <= 1'd1; 
        end else begin
                 
            user_led2 <= sw0;   
            if(sw5)
               user_led2 <= reg_intf_cur_state[2];  
        end
  end    
  
 assign user_led3 = sw5 ? reg_intf_cur_state[3] : pll_locked ;
 assign user_led4 = sw5 ? reg_intf_cur_state[4]: rst_ni;
 assign user_led5 = sw5 ? start_test : uart_rx;
  
  
  assign uart_tx = sw1 ? uart_tx_int : uart_rx;
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
,.uart_tx           (uart_tx_int)
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
,.debug_bus_u       (debug_bus_u)
,.debug_bus__reg_int(debug_bus__reg_int)
,.risc_rd_wr_cmd    (risc_rd_wr_cmd)
,.fpga_rd_wr_cmd    (fpga_rd_wr_cmd)
,.o_cur_state       (reg_intf_cur_state)
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


assign debug_bus = sw6? {debug_bus_u}: //uart_controller
                   sw7?{rst_ni,uart_rx,30'hDEAD_DEAD}:
                       debug_bus__reg_int ;//reg_intf_via_uart

//UART controller
/*assign    debug_bus_u     =  rxbitcnt &      //20:17
                             rx_reg   &      //16:9
                             toprx    &      //8
                             rx_2d    &      //7
                             top16_rx &      //6
                             rxrdyi   &      //5
                             clrdiv   &      //4
                             rxerr    &      //3
                             rxfsm;          //2:0 
                             
                             
     assign debug_bus__reg_int            =                                        
                         core_rst &     //0                    
                         uart_rx &      //1               
                         current_state & //6:2                          
                         current_state_recieve &   //11:7            
                         risc_rd_wr_cmd &  //12                    
                         fpga_rd_wr_cmd &  //13            
                         debugger_rd_wr_cmd &  //14                  
                         req_cmd_value &   //22:15                 
                         req_cmd_addr &//30:23
                         //63-31
                         32'd0
                         ;  
                             
                             
                             */
                             
                             
//reg_intf_via_uart                             
                                                    
                       
 
// assign pll_locked = 1'b1;                      

ila_0 debug_ila (
	.clk(clk_25mhz), // input wire clk


	.probe0(debug_bus) // input wire [31:0] probe0
);

endmodule
