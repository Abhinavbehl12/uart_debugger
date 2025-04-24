// Set Parameter CLKS_PER_BIT as follows:
// CLKS_PER_BIT == (Frequency of i_Clock)/(Frequency of UART)
// Example: 25 MHz Clock, 115200 baud UART
// (25000000)/(115200) == 217


module reg_intf_via_uart #(
parameter LOOPBACK_ADDRESS = 300,
parameter ENABLE_PARITY    = 0,
parameter MAKE_PARITY_ODD  = 0,
parameter BAUD_RATE        = 115200,
parameter CORE_CLK_FREQ    = 25000000 //25 MHZ

)(
input   core_clk
,input   core_rst

//UART INTERFACE - Connected to RS232
,input uart_rx
,output uart_tx
,output uart_busy

//REGISTER INTERFACE - COnnected to debugger module
,output [7:0]reg_addr   
,output [31:0]reg_wr_data
,input  [31:0]reg_rd_data
,output reg_wr_en  
,output reg_rd_en  
,input  reg_rd_done

,output risc_rd_wr_cmd
,output fpga_rd_wr_cmd
,output debugger_rd_wr_cmd

//DEbugger interface
,output reg start_test
,output [31:0] test_command
,input [31:0] test_result
,input  test_result_valid

//DEBUG interface
,output [20:0] debug_bus_u
);


reg[7:0] u_data_in       ;
reg u_data_valid_in ;
wire[7:0] u_data_out      ;
wire u_data_valid_out;
wire u_overflow      ;
wire u_parity_error  ;
wire u_rx     ;
wire u_tx     ;
wire u_tx_busy;


localparam IDLE=5'd0;
localparam REQ_CMD_VALUE_WORD1=5'd1;
localparam REQ_ADDR_WORD1=5'd2;
localparam REQ_DATA=5'd3;

localparam RSP_CMD_VALUE_WORD1=5'd1;
localparam RSP_ADDR_WORD1=5'd2;
localparam RSP_DATA=5'd3;

localparam WRITE_DATA_CMD=8'h7f;
wire read_command      ;
wire write_command     ;
wire rsp_cmd_pkt_length;
wire [7:0]rsp_cmd_value     ;
reg [31:0]rsp_cmd_data      ;

reg [4:0]current_state ;
reg [4:0]current_state_d ;
reg [15:0]length_counter;
reg [31:0]timer_cnt     ;
reg [7:0]req_cmd_value     ;
reg [7:0]req_cmd_addr     ;
reg [7:0]req_cmd_data     ;
reg valid_cmd_rcvd;
reg valid_cmd_rcvd_d;
reg valid_cmd_rcvd_2d;
reg req_cmd_rcvd_d;
reg req_cmd_rcvd;
reg req_cmd_data_valid;

reg [4:0]current_state_recieve  ;
reg [4:0]current_state_recieve_d;
reg [31:0]timer_cnt_rsp          ;
reg read_byte_ack_int      ;
reg response_pending       ;
reg reg_rd_en_int          ;
reg [4:0]length_counter_r          ;
reg response_pending_3d;       
reg response_pending_2d;       
reg response_pending_d ;       
reg rsp_read_done ;       
reg state_machne_active ;       




reg [1:0]word_cnt     ;  
reg req_32_bit_valid  ;
reg [31:0]byte_swap_32;   
reg [31:0]reg_rd_data_d  ;   
reg [31:0]req_32_bit_data;   


 uart_controller  #(
.ENABLE_PARITY   (ENABLE_PARITY  ),
.MAKE_PARITY_ODD (MAKE_PARITY_ODD),
.BAUD_RATE       (BAUD_RATE      ),
.CORE_CLK_FREQ   (CORE_CLK_FREQ  )  
) uart_controller_i(
.core_clk(core_clk),
.core_rst(core_rst),

//DATA INTERFACE - Connected to DEBUGGER/REGISTER interface
.data_in        (u_data_in       ),
.data_valid_in  (u_data_valid_in ),
.data_out       (u_data_out      ),
.data_valid_out (u_data_valid_out),
.overflow       (u_overflow      ),
.parity_error   (u_parity_error  ),

//UART INTERFACE - Connected to RS232
.rx     (uart_rx     ),
.tx     (uart_tx     ),
.tx_busy(u_tx_busy),

//DEBUG interface
.debug_bus_u ()

); 
//FSM to Understand Recieve DATA FROM UART 
always@(posedge core_clk)begin
        if (core_rst) begin
            current_state               <=     IDLE;
            length_counter              <=     16'd0;
            timer_cnt                   <=     32'd0;
            req_cmd_rcvd_d              <=     1'b0;
            req_cmd_data_valid          <=     1'b0;
            req_cmd_rcvd                <=     1'b0;
            req_cmd_value               <= 8'd0;
            req_cmd_addr                <= 8'd0;
            req_cmd_data                <= 8'd0;
			valid_cmd_rcvd              <= 'b0;
        end else begin     
            req_cmd_data_valid          <=     1'b0;
            req_cmd_rcvd_d              <=     req_cmd_rcvd;
            req_cmd_rcvd                <=     1'b0;
            current_state_d             <=     current_state;
			if(rsp_read_done)
			    req_cmd_value               <= 8'd0;
			
			
            if (current_state_d != current_state) begin        
                timer_cnt               <=     'd0;
            end else begin                       
                timer_cnt               <=     timer_cnt + 1;
            end
            if (timer_cnt == 'hFFFFFFFF) begin
                current_state                      <=     IDLE; //Reset the state machine after sometime if state do not move ahead
            end else if (u_data_valid_out==1'b1) begin
                current_state                      <=     IDLE;
                case (current_state) 
                 IDLE :
                 begin
                    if (u_data_out == 8'hcc) begin  
                        current_state              <=     REQ_CMD_VALUE_WORD1;
                    end
                    // req_cmd_value(7 : 0) <= u_data_out;
                    // req_cmd_value <= 8'd0;
                    length_counter <= 16'd4;//4 bytes of data to be rcvd
                end     
                 REQ_CMD_VALUE_WORD1 :
                 begin
                    current_state             <=     REQ_ADDR_WORD1;
                    req_cmd_value[7:0] <= u_data_out;
					valid_cmd_rcvd <= 'b1;
                 end 
                 REQ_ADDR_WORD1 :
                 begin
                    current_state            <=     REQ_DATA;
                    req_cmd_addr[7:0] <= u_data_out;
					valid_cmd_rcvd <= 'b1;
                 end 
                 REQ_DATA :
                 begin
                    timer_cnt            <=     'd0;
                    current_state        <=     REQ_DATA;
                    if (write_command == 1'b1) begin
                        req_cmd_data         <=     u_data_out;
                        req_cmd_data_valid   <=     1'b1;
                    end
                    length_counter      <=     length_counter - 1;
                    if (length_counter == 1 )begin
                        current_state              <=     IDLE;
                        req_cmd_rcvd    <=     1'b1;
					    valid_cmd_rcvd <= 'b0;
                    end
                 end 
                 default :
                    current_state                  <=     IDLE;
                endcase
            end
        end
    end 

//FSM to SEND DATA VIA UART     
always@(posedge core_clk)begin
        if (core_rst) begin
            current_state_recieve                        <=     IDLE;
            current_state_recieve_d                      <=     IDLE;
            timer_cnt_rsp               <=     'd0;
            read_byte_ack_int           <=    1'b0;
            response_pending            <=    1'b0;
            reg_rd_en_int               <=    1'b0;
            length_counter_r            <=    5'd0;
			response_pending_3d         <=    1'b0;
			response_pending_2d         <=    1'b0;
			response_pending_d          <=    1'b0;
			rsp_read_done               <=    1'd0;
            state_machne_active         <=    1'b0;
            u_data_in                   <=    8'h00;
            rsp_cmd_data                <=     32'd0;
        end else begin
            reg_rd_en_int               <=     1'b0;
            u_data_valid_in             <=     1'b0;
            read_byte_ack_int           <=     1'b0;
            state_machne_active         <=     1'b0;
            current_state_recieve_d     <=     current_state_recieve;
			response_pending_3d         <= response_pending_2d | response_pending_d | response_pending;
			response_pending_2d         <= response_pending_d;
			response_pending_d          <= response_pending;
            if (reg_rd_done == 1'b1) begin
                response_pending        <=    1'b0;
                rsp_cmd_data            <=  reg_rd_data;
            end
            
			if (current_state_recieve_d != current_state_recieve) begin
                timer_cnt_rsp           <=     'd0;
            end else begin
                timer_cnt_rsp           <=     timer_cnt_rsp + 1;
            end
            
			if (timer_cnt_rsp =='hFFFFFFFF) begin
                current_state_recieve                    <=     IDLE;
			    rsp_read_done               <=    1'd0;
            end else if ((u_tx_busy == 1'b0) && (response_pending_3d == 1'b0) && (u_data_valid_in == 1'b0)) begin
			
                state_machne_active                    <=    1'b1;
                case (current_state_recieve)
                 IDLE :
                 begin
				    rsp_read_done <= 1'd0;
                    if (((req_cmd_rcvd_d == 1'b1) && (write_command == 1'b1) && (req_cmd_addr[7:0] == LOOPBACK_ADDRESS) ) | 
                    // if (((req_cmd_rcvd_d == 1'b1) && (write_command == 1'b1)  ) | 
                       (read_command == 1'b1)) begin
                        current_state_recieve  <=     RSP_CMD_VALUE_WORD1;
                        u_data_in              <=     'hCC;
                        u_data_valid_in        <=     1'b1;
                        length_counter_r       <=     5'd5;
				        // rsp_read_done          <=     1'd0;
                    end
					
                 end
                 RSP_CMD_VALUE_WORD1 :
                 begin
                        current_state_recieve  <=    RSP_ADDR_WORD1;
                        u_data_in              <=    rsp_cmd_value;
                        u_data_valid_in        <=    1'b1;
                 end
                 RSP_ADDR_WORD1 :
                 begin
                    current_state_recieve     <=    RSP_DATA;
                    u_data_in                 <=    reg_addr;
                    u_data_valid_in           <=    1'b1;
                    length_counter_r          <=    length_counter_r - 5'd1;
                    reg_rd_en_int             <=    1'b1;
                    response_pending          <=    1'b1;
                 end
                 RSP_DATA :
                 begin
                    // if (length_counter_r == 5'd4 ) begin //send read signal only once
                        // reg_rd_en_int    <=    1'b0;
                        // response_pending <=    1'b0;
                        // length_counter_r    <=    length_counter_r - 5'd1;
                    // end
                    if (length_counter_r > 5'd0) begin
                        timer_cnt_rsp           <=    'd0;
                        current_state_recieve   <=    RSP_DATA;
                        u_data_in               <=    rsp_cmd_data[31:24];
                        rsp_cmd_data            <=    {rsp_cmd_data[23 : 0] , 8'd0};
                        u_data_valid_in         <=    1'b1;
                        length_counter_r        <=    length_counter_r - 5'd1;
				        rsp_read_done <= 1'd1;
                    end else if(length_counter_r == 5'd0)begin
                        current_state_recieve                 <=    IDLE;
                    end
                 end
                 default :
                        current_state_recieve                 <=    IDLE;
                endcase
            end
        end
    end 

//concatenating byte data to 32 bit data
always@(posedge core_clk)begin
        if (core_rst) begin
            word_cnt                     <=     2'd0;
            req_32_bit_valid             <=     1'b0;
            valid_cmd_rcvd_d             <=     1'b0;
            valid_cmd_rcvd_2d             <=     1'b0;
            byte_swap_32                 <=     32'd0;
            reg_rd_data_d                <=     32'd0;
            req_32_bit_data              <=     32'd0;
            start_test                   <=     1'b0;
        end else begin
            valid_cmd_rcvd_d             <=     valid_cmd_rcvd;
			valid_cmd_rcvd_2d            <= valid_cmd_rcvd_d;
            req_32_bit_valid             <=     1'b0;
            start_test                   <=     1'b0;
			if(!valid_cmd_rcvd_d )begin
                word_cnt                <=     'd0;
			end 
            if (reg_rd_done == 1'b1) begin
                reg_rd_data_d           <=  reg_rd_data;
                //rsp_cmd_data            <=  reg_rd_data;
            end
            if (req_cmd_data_valid == 1'b1) begin
                req_32_bit_data         <=     {req_32_bit_data[23 : 0] , req_cmd_data};
                byte_swap_32            <=     {req_cmd_data , byte_swap_32[31 : 8]};
                if (write_command == 1'b1) begin
                    word_cnt                <=     word_cnt + 1;
                end    
                if (word_cnt == 2'b11) begin
                    req_32_bit_valid        <=     1'b1;
					if(req_cmd_value==8'h7F)begin
					    start_test <= 1'b1;
					end 
                end
            end 
        end
end 

    assign risc_rd_wr_cmd         =  (valid_cmd_rcvd_2d && req_cmd_value[3:0] == 4'd6) ? 1'b1   : 1'b0;
    assign fpga_rd_wr_cmd         =  (valid_cmd_rcvd_2d && req_cmd_value[3:0] == 4'd4) ? 1'b1   : 1'b0;
    assign debugger_rd_wr_cmd     =  (valid_cmd_rcvd_2d && req_cmd_value[3:0] == 4'd2) ? 1'b1   : 1'b0;
	
    assign read_command         =  (valid_cmd_rcvd && req_cmd_value[7] == 1'b1) ? 1'b1   : 1'b0;
    assign write_command        =  (valid_cmd_rcvd && req_cmd_value[7] == 1'b0) ? 1'b1   : 1'b0;
    assign rsp_cmd_pkt_length   =  (read_command == 1'b1)?req_32_bit_data[15:0] + 12 : 'b0010;
    assign rsp_cmd_value        =  valid_cmd_rcvd && req_cmd_value;
    
    // assign rsp_cmd_data        =   reg_rd_data_d;
                                                                
    // ------------Register write
    // assign reg_wr_en_int        =    (write_command == 1'b1) ? (req_cmd_rcvd_d | req_32_bit_valid)   :1'b0;
    assign reg_wr_en            =    req_32_bit_valid  ;
    assign reg_rd_en            =    reg_rd_en_int;
    assign reg_addr             =    req_cmd_addr;
    assign reg_wr_data          =    req_32_bit_data;
    // assign reg_wr_en            =    reg_wr_en_int;
	
	// assign uart_tx = response_pending_3d ?  1'dz : uart_tx ;
	// assign uart_tx =  uart_tx ;
	assign uart_busy = response_pending_3d  ;
    // assign debug_bus            =    zeros(192 : 0) &   --255:63
                        // debug_bus_u &                  --62:42
                        // RST &                          --41
                        // UART_RX &                      --40
                        // CS &                           --39:35
                        // length_counter &               --34:19
                        // tx_busy &                       --18
                        // u_data_valid_in &                     --17
                        // u_data_in &                          --16:9
                        // DoutValid &                    --8
                        // u_data_out;                          --7:0
endmodule