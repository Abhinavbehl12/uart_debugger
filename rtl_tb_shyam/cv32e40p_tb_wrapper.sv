// Copyright 2018 Robert Balas <balasr@student.ethz.ch>
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.

// Wrapper for a CV32E40P testbench, containing CV32E40P, Memory and stdout peripheral
// Contributor: Robert Balas <balasr@student.ethz.ch>
// Module renamed from riscv_wrapper to cv32e40p_tb_wrapper because (1) the
// name of the core changed, and (2) the design has a cv32e40p_wrapper module.
//
// SPDX-License-Identifier: Apache-2.0 WITH SHL-0.51

module cv32e40p_tb_wrapper
    #(parameter // Parameters used by TB
                INSTR_RDATA_WIDTH = 32,
                RAM_ADDR_WIDTH    = 5,
                BOOT_ADDR         = 'h04,
                DM_HALTADDRESS    = 32'h1A11_0800,
                HART_ID           = 32'h0000_0000,
                // Parameters used by DUT
                PULP_XPULP        = 0,
                PULP_CLUSTER      = 0,
                FPU               = 0,
                PULP_ZFINX        = 0,
                RISC_TOP          = 0,
                NUM_MHPMCOUNTERS  = 1
    )
    (input logic         clk_i,
     input logic         rst_ni,

     input logic         fetch_enable_i,
     output logic        tests_passed_o,
     output logic        tests_failed_o,
     output logic [31:0] exit_value_o,
     output logic        exit_valid_o);

    // signals connecting core to memory
    logic                         instr_req;
    logic                         instr_gnt;
    logic                         instr_rvalid;
    logic [31:0]                  instr_addr;
    logic [INSTR_RDATA_WIDTH-1:0] instr_rdata;

    logic                         data_req;
    logic                         data_gnt;
    logic                         data_rvalid;
    logic [31:0]                  data_addr;
    logic                         data_we;
    logic [3:0]                   data_be;
    logic [31:0]                  data_rdata;
    logic [31:0]                  data_wdata;

    // signals to debug unit
    logic                         debug_req;

    // irq signals (not used)
    logic [0:31]                  irq;
    logic [0:4]                   irq_id_in;
    logic                         irq_ack;
    logic [0:4]                   irq_id_out;
    logic                         irq_sec;

logic uart_rx  ;
logic uart_rx2  ;
logic uart_tx  ;
logic uart_busy;
logic clk_25mhz;

    // interrupts (only timer for now)
    assign irq_sec     = '0;

    
    cv32e40p_fpga_top #(
                 .COREV_PULP       (PULP_XPULP),
                 .COREV_CLUSTER     (PULP_CLUSTER),
                 .FPU              (FPU),
                 .ZFINX       (PULP_ZFINX),
                 .NUM_MHPMCOUNTERS (NUM_MHPMCOUNTERS)
                )
    cv32e40p_fpga_top_i
        (
         .clk_25mhz              ( clk_25mhz                 ),
         .clk_i                  ( clk_i                 ),
         .rst_i                 ( !rst_ni                ),


	//UART interface
	    .uart_rx    (uart_rx2  )
	    // .uart_tx    (uart_tx  ),
	    // .uart_busy  (uart_busy)
       );
    // end 	
// endgenerate	
////////////////////////////////////
//DRIVE uart
////////////////////////////////////
//drive_uart drive_uart_1(
//.clk_25mhz  (clk_25mhz),
//.u_uart_rx   (uart_rx)
//);
drive_uart drive_uart_2(
 .clk_25mhz  (clk_25mhz),
.u_uart_rx   (uart_rx2)
);

endmodule // cv32e40p_tb_wrapper
