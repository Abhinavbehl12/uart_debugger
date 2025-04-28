

module mem_intf_via_uart #(
parameter ADDR_WIDTH =8,
parameter MAX_INST_COUNT =2,//will start counting from 0
parameter DELAY_READ_INST =4
)(
input   core_clk
,input   core_clk_25Mhz
,input   rst_n


//REGISTER INTERFACE - COnnected to reg_intf module
,input [7:0]reg_addr   
,input [31:0]reg_wr_data
,output reg [31:0]reg_rd_data
,input reg_wr_en  
,input reg_rd_en  
,output reg reg_rd_done

//DEbugger interface
,input start_test

,output reg start_test_risc_clk,

// Instruction memory interface
     input                           instr_req_i,
     input [ADDR_WIDTH-1:0]         instr_addr_i,
     output [31:0]                  instr_rdata_o,
     output                         instr_rvalid_o,
     output                         instr_gnt_o,
	 

// Data memory interface	 
     input                          data_req_i,
     input [31:0]                   data_addr_i,
     input                          data_we_i,
     input [3:0]                    data_be_i,
     input [31:0]                   data_wdata_i,
     output [31:0]                  data_rdata_o,
     output                         data_rvalid_o,
     output                         data_gnt_o	 
);

reg start_test_d ;
reg start_test_2d;
reg start_test_3d;

reg [ADDR_WIDTH-1:0] mem[31:0];
reg [31:0]instr_rdata_reg;
reg instr_rvalid_reg;
reg instr_gnt_reg;
reg test_started;
reg stop_test;
reg [7:0]count;
reg [7:0]inst_count;

  //-----------------------------------------------------------------------------
  //-- READ : Read Registers for Debugger
  //-----------------------------------------------------------------------------
    always @(posedge core_clk_25Mhz or negedge rst_n) begin
	    if (~rst_n) begin
	        reg_rd_data <= 32'd0; 
	        reg_rd_done <= 1'b0; 
        end else begin
	        if(reg_rd_en)begin
	            reg_rd_data <= mem[reg_addr[4:0]]; 
	            reg_rd_done <= 1'b1; 
			end else begin
	            reg_rd_data <= 32'd0; 
	            reg_rd_done <= 1'b0; 
			end 
        end
	end 

  //-----------------------------------------------------------------------------
  //-- WRITE : WRITE Registers for Debugger
  //-----------------------------------------------------------------------------
    always @(posedge core_clk_25Mhz or negedge rst_n) begin
	    if (~rst_n) begin
	        //mem <= 'd0; 
        end else begin
	        if(reg_wr_en)begin
	            mem[reg_addr[4:0]] <= reg_wr_data; 
			end 
        end
	end 
	
  //-----------------------------------------------------------------------------
  //-- start_test logic
  //-----------------------------------------------------------------------------
    always @(posedge core_clk or negedge rst_n) begin
	    if (~rst_n) begin
	        start_test_d <= 1'b0; 
	        start_test_2d <= 1'b0; 
	        start_test_3d <= 1'b0; 
        end else begin
	        start_test_d <= start_test; 
	        start_test_2d <= start_test_d; 
	        start_test_3d <= start_test_2d; 
	        start_test_risc_clk <= start_test_3d; 
        end
	end 
  //-----------------------------------------------------------------------------
  //-- INSTRUCTION logic
  //-----------------------------------------------------------------------------
    always @(posedge core_clk or negedge rst_n) begin
	    if (~rst_n) begin
	        instr_rdata_reg <= 'd0;
			instr_rvalid_reg<= 'd0;
			instr_gnt_reg   <= 'd0;
			test_started    <= 1'd0;
			count           <= 8'd0;
			inst_count      <= 8'd0;
			stop_test      <= 1'd0;
			
        end else begin
		    if(start_test_3d)begin
			    test_started <= 1'b1;
			end else if (stop_test) begin
			    test_started <= 1'b0;
			    stop_test <= 1'b0;
			
			end 
			
			if(test_started)begin
			    if(instr_req_i && count == DELAY_READ_INST && inst_count == 8'd0)begin
				    count<= 8'd0;
					instr_gnt_reg <= 1'b1;
					inst_count <= inst_count+8'd1;
					instr_rvalid_reg<= 1'b0;
					instr_rdata_reg <= 32'd0;
				end else if(instr_req_i && count == DELAY_READ_INST && inst_count <= MAX_INST_COUNT)begin
				    count<= 8'd0;
					instr_gnt_reg    <= 1'b1;
					instr_rvalid_reg <= 1'b1;
					instr_rdata_reg  <= mem[inst_count-8'd1];
					inst_count       <= inst_count+8'd1;
				end else if( count == DELAY_READ_INST && inst_count > MAX_INST_COUNT)begin
				    count<= 8'd0;
					instr_gnt_reg <= 1'b0;
					instr_rvalid_reg <= 1'b1;
					instr_rdata_reg  <= mem[inst_count-8'd1];
					inst_count    <= 8'd0;
					stop_test  <= 1'b1;
				end else begin
                    count<=count+8'd1;
					instr_gnt_reg <= 1'b0;
					instr_rvalid_reg<= 1'b0;
					instr_rdata_reg <= 32'd0;
				end 	
			end 
        end
	end 

	assign  instr_rdata_o  =instr_rdata_reg;
	assign  instr_rvalid_o =instr_rvalid_reg;
	assign  instr_gnt_o    =instr_gnt_reg   ;

  


endmodule