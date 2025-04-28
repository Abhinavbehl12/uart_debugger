// Copyright 2018 ETH Zurich and University of Bologna.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.

////////////////////////////////////////////////////////////////////////////////
// Engineer:       Matthias Baer - baermatt@student.ethz.ch                   //
//                                                                            //
// Additional contributions by:                                               //
//                 Igor Loi - igor.loi@unibo.it                               //
//                 Andreas Traber - atraber@student.ethz.ch                   //
//                 Sven Stucki - svstucki@student.ethz.ch                     //
//                 Michael Gautschi - gautschi@iis.ee.ethz.ch                 //
//                 Davide Schiavone - pschiavo@iis.ee.ethz.ch                 //
//                                                                            //
// Design Name:    Top level module                                           //
// Project Name:   RI5CY                                                      //
// Language:       SystemVerilog                                              //
//                                                                            //
// Description:    Top level module of the RISC-V core.                       //
//                 added APU, FPU parameter to include the APU_dispatcher     //
//                 and the FPU                                                //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

module cv32e40p_core
  import cv32e40p_apu_core_pkg::*;
#(
    parameter COREV_PULP =  0,  // PULP ISA Extension (incl. custom CSRs and hardware loop, excl. cv.elw)
    parameter COREV_CLUSTER = 0,  // PULP Cluster interface (incl. cv.elw)
    parameter FPU = 0,  // Floating Point Unit (interfaced via APU interface)
    parameter FPU_ADDMUL_LAT = 0,  // Floating-Point ADDition/MULtiplication lane pipeline registers number
    parameter FPU_OTHERS_LAT = 0,  // Floating-Point COMParison/CONVersion lanes pipeline registers number
    parameter ZFINX = 0,  // Float-in-General Purpose registers
    parameter NUM_MHPMCOUNTERS = 1
) (
    // Clock and Reset
    input logic clk_i,
    input logic rst_ni,

    input logic pulp_clock_en_i,  // PULP clock enable (only used if COREV_CLUSTER = 1)
    input logic scan_cg_en_i,  // Enable all clock gates for testing

    // Core ID, Cluster ID, debug mode halt address and boot address are considered more or less static
    input logic [31:0] boot_addr_i,
    input logic [31:0] mtvec_addr_i,
    input logic [31:0] dm_halt_addr_i,
    input logic [31:0] hart_id_i,
    input logic [31:0] dm_exception_addr_i,

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

    // CVFPU interface
    output logic                              apu_busy_o,
    // handshake signals
    output logic                              apu_req_o,
    input  logic                              apu_gnt_i,
    // request channel
    output logic [   APU_NARGS_CPU-1:0][31:0] apu_operands_o,
    output logic [     APU_WOP_CPU-1:0]       apu_op_o,
    output logic [APU_NDSFLAGS_CPU-1:0]       apu_flags_o,
    // response channel
    input  logic                              apu_rvalid_i,
    input  logic [                31:0]       apu_result_i,
    input  logic [APU_NUSFLAGS_CPU-1:0]       apu_flags_i,

    // Interrupt inputs
    input  logic [31:0] irq_i,  // CLINT interrupts + CLINT extension interrupts
    output logic        irq_ack_o,
    output logic [ 4:0] irq_id_o,

    // Debug Interface
    input  logic debug_req_i,
    output logic debug_havereset_o,
    output logic debug_running_o,
    output logic debug_halted_o,
//REGISTER INTERFACE - COnnected to debugger module
 input [7:0]reg_addr   
,input [31:0]reg_wr_data
,output  [31:0]reg_rd_data
,input reg_wr_en  
,input reg_rd_en  
,output  reg_rd_done,
    // CPU Control Signals
    input  logic fetch_enable_i,
    output logic core_sleep_o
);

  import cv32e40p_pkg::*;

  // Unused parameters and signals (left in code for future design extensions)
  localparam PULP_SECURE = 0;
  localparam N_PMP_ENTRIES = 16;
  localparam USE_PMP = 0;  // if PULP_SECURE is 1, you can still not use the PMP
  localparam A_EXTENSION = 0;
  localparam DEBUG_TRIGGER_EN = 1;

  // PULP bus interface behavior
  // If enabled will allow non-stable address phase signals during waited instructions requests and
  // will re-introduce combinatorial paths from instr_rvalid_i to instr_req_o and from from data_rvalid_i
  // to data_req_o
  localparam PULP_OBI = 0;

  // Unused signals related to above unused parameters
  // Left in code (with their original _i, _o postfixes) for future design extensions;
  // these used to be former inputs/outputs of RI5CY

  logic [5:0] data_atop_o;  // atomic operation, only active if parameter `A_EXTENSION != 0`
  logic       irq_sec_i;
  logic       sec_lvl_o;

  localparam N_HWLP = 2;
  localparam APU = (FPU == 1) ? 1 : 0;

  // IF/ID signals
  logic        instr_valid_id;
  logic [31:0] instr_rdata_id;  // Instruction sampled inside IF stage
  logic        is_compressed_id;
  logic        illegal_c_insn_id;
  logic        is_fetch_failed_id;

  logic        clear_instr_valid;
  logic        pc_set;

  logic [ 3:0] pc_mux_id;  // Mux selector for next PC
  logic [ 2:0] exc_pc_mux_id;  // Mux selector for exception PC
  logic [ 4:0] m_exc_vec_pc_mux_id;  // Mux selector for vectored IRQ PC
  logic [ 4:0] u_exc_vec_pc_mux_id;  // Mux selector for vectored IRQ PC
  logic [ 4:0] exc_cause;

  logic [ 1:0] trap_addr_mux;

  logic [31:0] pc_if;  // Program counter in IF stage
  logic [31:0] pc_id;  // Program counter in ID stage

  // ID performance counter signals
  logic        is_decoding;

  logic        useincr_addr_ex;  // Active when post increment
  logic        data_misaligned;

  logic        mult_multicycle;

  // Jump and branch target and decision (EX->IF)
  logic [31:0] jump_target_id, jump_target_ex;
  logic               branch_in_ex;
  logic               branch_decision;
  logic        [ 1:0] ctrl_transfer_insn_in_dec;

  logic               ctrl_busy;
  logic               if_busy;
  logic               lsu_busy;

  logic        [31:0] pc_ex;  // PC of last executed branch or cv.elw

  // ALU Control
  logic               alu_en_ex;
  alu_opcode_e        alu_operator_ex;
  logic        [31:0] alu_operand_a_ex;
  logic        [31:0] alu_operand_b_ex;
  logic        [31:0] alu_operand_c_ex;
  logic        [ 4:0] bmask_a_ex;
  logic        [ 4:0] bmask_b_ex;
  logic        [ 1:0] imm_vec_ext_ex;
  logic        [ 1:0] alu_vec_mode_ex;
  logic alu_is_clpx_ex, alu_is_subrot_ex;
  logic        [                 1:0]       alu_clpx_shift_ex;

  // Multiplier Control
  mul_opcode_e                              mult_operator_ex;
  logic        [                31:0]       mult_operand_a_ex;
  logic        [                31:0]       mult_operand_b_ex;
  logic        [                31:0]       mult_operand_c_ex;
  logic                                     mult_en_ex;
  logic                                     mult_sel_subword_ex;
  logic        [                 1:0]       mult_signed_mode_ex;
  logic        [                 4:0]       mult_imm_ex;
  logic        [                31:0]       mult_dot_op_a_ex;
  logic        [                31:0]       mult_dot_op_b_ex;
  logic        [                31:0]       mult_dot_op_c_ex;
  logic        [                 1:0]       mult_dot_signed_ex;
  logic                                     mult_is_clpx_ex;
  logic        [                 1:0]       mult_clpx_shift_ex;
  logic                                     mult_clpx_img_ex;

  // FPU
  logic                                     fs_off;
  logic        [            C_RM-1:0]       frm_csr;
  logic        [         C_FFLAG-1:0]       fflags_csr;
  logic                                     fflags_we;
  logic                                     fregs_we;

  // APU
  logic                                     apu_en_ex;
  logic        [APU_NDSFLAGS_CPU-1:0]       apu_flags_ex;
  logic        [     APU_WOP_CPU-1:0]       apu_op_ex;
  logic        [                 1:0]       apu_lat_ex;
  logic        [   APU_NARGS_CPU-1:0][31:0] apu_operands_ex;
  logic        [                 5:0]       apu_waddr_ex;

  logic        [                 2:0][ 5:0] apu_read_regs;
  logic        [                 2:0]       apu_read_regs_valid;
  logic                                     apu_read_dep;
  logic                                     apu_read_dep_for_jalr;
  logic        [                 1:0][ 5:0] apu_write_regs;
  logic        [                 1:0]       apu_write_regs_valid;
  logic                                     apu_write_dep;

  logic                                     perf_apu_type;
  logic                                     perf_apu_cont;
  logic                                     perf_apu_dep;
  logic                                     perf_apu_wb;

  // Register Write Control
  logic        [                 5:0]       regfile_waddr_ex;
  logic                                     regfile_we_ex;
  logic        [                 5:0]       regfile_waddr_fw_wb_o;  // From WB to ID
  logic                                     regfile_we_wb;
  logic                                     regfile_we_wb_power;
  logic        [                31:0]       regfile_wdata;

  logic        [                 5:0]       regfile_alu_waddr_ex;
  logic                                     regfile_alu_we_ex;

  logic        [                 5:0]       regfile_alu_waddr_fw;
  logic                                     regfile_alu_we_fw;
  logic                                     regfile_alu_we_fw_power;
  logic        [                31:0]       regfile_alu_wdata_fw;

  // CSR control
  logic                                     csr_access_ex;
  csr_opcode_e                              csr_op_ex;
  logic [23:0] mtvec, utvec;
  logic        [ 1:0] mtvec_mode;
  logic        [ 1:0] utvec_mode;

  csr_opcode_e        csr_op;
  csr_num_e           csr_addr;
  csr_num_e           csr_addr_int;
  logic        [31:0] csr_rdata;
  logic        [31:0] csr_wdata;
  PrivLvl_t           current_priv_lvl;

  // Data Memory Control:  From ID stage (id-ex pipe) <--> load store unit
  logic               data_we_ex;
  logic        [ 5:0] data_atop_ex;
  logic        [ 1:0] data_type_ex;
  logic        [ 1:0] data_sign_ext_ex;
  logic        [ 1:0] data_reg_offset_ex;
  logic               data_req_ex;
  logic               data_load_event_ex;
  logic               data_misaligned_ex;

  logic               p_elw_start;  // Start of cv.elw load (when data_req_o is sent)
  logic               p_elw_finish;  // Finish of cv.elw load (when data_rvalid_i is received)

  logic        [31:0] lsu_rdata;

  // stall control
  logic               halt_if;
  logic               id_ready;
  logic               ex_ready;

  logic               id_valid;
  logic               ex_valid;
  logic               wb_valid;

  logic               lsu_ready_ex;
  logic               lsu_ready_wb;

  logic               apu_ready_wb;

  // Signals between instruction core interface and pipe (if and id stages)
  logic               instr_req_int;  // Id stage asserts a req to instruction core interface

  // Interrupts
  logic m_irq_enable, u_irq_enable;
  logic csr_irq_sec;
  logic [31:0] mepc, uepc, depc;
  logic [             31:0]       mie_bypass;
  logic [             31:0]       mip;

  logic                           csr_save_cause;
  logic                           csr_save_if;
  logic                           csr_save_id;
  logic                           csr_save_ex;
  logic [              5:0]       csr_cause;
  logic                           csr_restore_mret_id;
  logic                           csr_restore_uret_id;
  logic                           csr_restore_dret_id;
  logic                           csr_mtvec_init;

  // HPM related control signals
  logic [             31:0]       mcounteren;

  // debug mode and dcsr configuration
  logic                           debug_mode;
  logic [              2:0]       debug_cause;
  logic                           debug_csr_save;
  logic                           debug_single_step;
  logic                           debug_ebreakm;
  logic                           debug_ebreaku;
  logic                           trigger_match;
  logic                           debug_p_elw_no_sleep;

  // Hardware loop controller signals
  logic [       N_HWLP-1:0][31:0] hwlp_start;
  logic [       N_HWLP-1:0][31:0] hwlp_end;
  logic [       N_HWLP-1:0][31:0] hwlp_cnt;

  logic [             31:0]       hwlp_target;
  logic                           hwlp_jump;

  // Performance Counters
  logic                           mhpmevent_minstret;
  logic                           mhpmevent_load;
  logic                           mhpmevent_store;
  logic                           mhpmevent_jump;
  logic                           mhpmevent_branch;
  logic                           mhpmevent_branch_taken;
  logic                           mhpmevent_compressed;
  logic                           mhpmevent_jr_stall;
  logic                           mhpmevent_imiss;
  logic                           mhpmevent_ld_stall;
  logic                           mhpmevent_pipe_stall;

  logic                           perf_imiss;

  // Wake signal
  logic                           wake_from_sleep;

  // PMP signals
  logic [N_PMP_ENTRIES-1:0][31:0] pmp_addr;
  logic [N_PMP_ENTRIES-1:0][ 7:0] pmp_cfg;

  logic                           data_req_pmp;
  logic [             31:0]       data_addr_pmp;
  logic                           data_gnt_pmp;
  logic                           data_err_pmp;
  logic                           data_err_ack;
  logic                           instr_req_pmp;
  logic                           instr_gnt_pmp;
  logic [             31:0]       instr_addr_pmp;
  logic                           instr_err_pmp;

  // Mux selector for vectored IRQ PC
  assign m_exc_vec_pc_mux_id = (mtvec_mode == 2'b0) ? 5'h0 : exc_cause;
  assign u_exc_vec_pc_mux_id = (utvec_mode == 2'b0) ? 5'h0 : exc_cause;

  // PULP_SECURE == 0
  assign irq_sec_i = 1'b0;

  // APU master signals
  assign apu_flags_o = apu_flags_ex;

  //////////////////////////////////////////////////////////////////////////////////////////////
  //   ____ _            _      __  __                                                   _    //
  //  / ___| | ___   ___| | __ |  \/  | __ _ _ __   __ _  __ _  ___ _ __ ___   ___ _ __ | |_  //
  // | |   | |/ _ \ / __| |/ / | |\/| |/ _` | '_ \ / _` |/ _` |/ _ \ '_ ` _ \ / _ \ '_ \| __| //
  // | |___| | (_) | (__|   <  | |  | | (_| | | | | (_| | (_| |  __/ | | | | |  __/ | | | |_  //
  //  \____|_|\___/ \___|_|\_\ |_|  |_|\__,_|_| |_|\__,_|\__, |\___|_| |_| |_|\___|_| |_|\__| //
  //                                                     |___/                                //
  //////////////////////////////////////////////////////////////////////////////////////////////

  logic clk;
  logic fetch_enable;

 
 
  // Tracer signal
  assign wb_valid = lsu_ready_wb;


  //  CSR access
  assign csr_addr = csr_addr_int;
  assign csr_wdata = alu_operand_a_ex;
  assign csr_op = csr_op_ex;

  assign csr_addr_int = csr_num_e'(csr_access_ex ? alu_operand_b_ex[11:0] : '0);

  //  Floating-Point registers write
  assign fregs_we     = (FPU == 1 & ZFINX == 0) ? ((regfile_alu_we_fw && regfile_alu_waddr_fw[5]) ||
                                                   (regfile_we_wb     && regfile_waddr_fw_wb_o[5]))
                                                : 1'b0;



endmodule
