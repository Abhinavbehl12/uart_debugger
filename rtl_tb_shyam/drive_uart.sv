
module drive_uart
   (
   output reg u_uart_rx,
   output clk_25mhz
   
   );

 // Testbench uses a 25 MHz clock
  // Want to interface to 115200 baud UART
  // 25000000 / 115200 = 217 Clocks Per Bit.
  localparam c_CLOCK_PERIOD_NS = 40000;
  localparam c_CLKS_PER_BIT    = 217;
  localparam c_BIT_PERIOD      = 115200;
  localparam FILE_READ         = 1;
  
  reg r_Clock = 0;
  reg r_TX_DV = 0;
  wire w_TX_Active, w_UART_Line;
  wire w_TX_Serial;
  reg [7:0] r_TX_Byte = 0;
  wire [7:0] w_RX_Byte;
  reg core_reset;
  wire [7:0]u_reg_addr   ;
  wire [31:0]u_reg_wr_data;
  reg [31:0]u_reg_rd_data;
  wire u_reg_wr_en  ;
  wire u_reg_rd_en  ;
  reg u_reg_rd_done;
  
  reg [2:0]current_state;
  reg [3:0] count;
  reg [7:0] data_8_bit;
  reg  start;
  // reg  u_uart_rx;
  wire  u_uart_tx;
  reg  uart_busy=0;
  
  reg enable_div16_rx;
reg toptx       ;
reg [7:0]clkdiv       ;
reg top16_tx       ;
reg [7:0]div16_tx       ;
  
  reg send_start_info;
  
  localparam IDLE = 3'd0;
  localparam SEND_ADDR = 3'd1;
  localparam SEND_DATA_SYMB0 = 3'd2;
  localparam SEND_DATA_SYMB1 = 3'd3;
  localparam SEND_DATA_SYMB2 = 3'd4;
  localparam SEND_DATA_SYMB3 = 3'd5;
  localparam STOP = 3'd6;
  localparam STOP2 = 3'd7;
  localparam REQ_CMD_VALUE = 3'd7;
  

  
  always
    #(c_CLOCK_PERIOD_NS/2) r_Clock <= !r_Clock;
assign clk_25mhz = r_Clock;

parameter BAUD_RATE        = 115200;
parameter CORE_CLK_FREQ    = 25000000; //25 MHZ
wire [7:0]divi_tx;
wire [7:0]divi_rx;
wire [7:0]divisor;
    // // //////////////////////////
    // // Baud rate selection
    // // //////////////////////////
assign    divi_tx =  8'b00001001;   //  //9 (10 - 1) counting from 0 //9 bits of data needed to be sent
assign    divi_rx =  8'b00000100;   //  //4 ( 5 - 1) counting from 0
assign    divisor =  (CORE_CLK_FREQ/(10*BAUD_RATE) - 1);
    
reg [55:0] RAM[15:0] ;
integer wait_10;
integer i;
integer index;
initial
$readmemh("/home/shyam/Desktop/mounted_nfsserver/Abhinav_behl/uart_debugger/rtl_tb/uart_input.txt", RAM);
initial begin for(i=0;i<15;i=i+1)             
$display ("RAM[%d]=%h",i,RAM[i]); 
    end  


always@(posedge r_Clock)begin
    if (!start) begin
        top16_tx         <=     1'b0;
        div16_tx         <=     8'd0;
    end else begin
        top16_tx         <=     1'b0;
        if (div16_tx == divisor) begin
            div16_tx     <=     8'd0;
            top16_tx     <=     1'b1;
        end else
            div16_tx     <=     div16_tx + 1;
        
    end
end
  
    // // //////////////////////////
    // // Tx Clock Generation
    // // //////////////////////////

always@(posedge r_Clock)begin
    if (!start) begin
        toptx              <=     1'b0;
        clkdiv             <=     8'd0;
    end else begin
        toptx              <=     1'b0;
        if (top16_tx==1'b1) begin
        clkdiv             <=     clkdiv + 1;
            if (clkdiv == divi_tx) begin
                toptx      <=     1'b1;
                clkdiv     <=     8'd0;
            end 
        end 
    end 
end 


   
   
always @(posedge r_Clock or  start)begin
    if(start==0)begin
        current_state <= IDLE;
        data_8_bit    <= 8'hcc;
        count         <= 4'd8;
        u_uart_rx     <= 1'dz;
    end else begin
    end 


end    
always @(posedge r_Clock or  start)begin
    if(u_reg_rd_en==1)begin
	    u_reg_rd_data <= 32'hAAAADDDD;
		u_reg_rd_done <= 1'b1;
    end else begin
	    u_reg_rd_data <= 32'h0;
		u_reg_rd_done <= 1'b0;
    end 


end 
always @(posedge r_Clock or  start)begin
    if(start==0)begin
        current_state <= IDLE;
		if(FILE_READ == 1)
        data_8_bit<= RAM[0][55:48];
		else 
        data_8_bit    <= 8'hcc;
        count         <= 4'd9;
        u_uart_rx     <= 1'dz;
		index         <= 'd0;
		wait_10         <= 'd0;
                send_start_info <= 1'b0;
    end else begin
        if(toptx==1'b1)begin
        
		//send_start_info         <= 'd1;
        if(send_start_info && !uart_busy)begin
            if(count==8'd9)begin
                u_uart_rx <= 1'b0;//IDLE bit 
                count<=count-1'b1;
                 
            end else if(count==8'd8)begin
                u_uart_rx <= 1'b1;//START bit 
                count<=count-1'b1;
                 
            end else begin
            u_uart_rx <= data_8_bit[0];
		    end 
        end else if (!uart_busy) begin
            u_uart_rx <= data_8_bit[0];
		end else 
            u_uart_rx <= 1'bz;
        
        
        if(!uart_busy)begin
            case(current_state)
            IDLE : begin
                send_start_info <= 1'b1;
                if(count<=7 && count!=0)begin
                    // u_uart_rx <= data_8_bit[0];//MAGIC BITS 'hCC
                    data_8_bit<= {1'b0,data_8_bit[7:1]};
                    count<=count-1'b1;
                    send_start_info <= 1'b0;
                    
                end else if (count==0)begin
                    // u_uart_rx <= data_8_bit[0];//MAGIC BITS 'hCC
                    count<= 4'd9;
                    send_start_info <= 1'b1;
                    // data_8_bit  <= 8'h06; //WRITE COMMAND
		    		if(FILE_READ == 1)
                    data_8_bit<= RAM[index][47:40];
		    		else 
                    data_8_bit  <= 8'h86; //READ COMMAND
                    current_state <= REQ_CMD_VALUE;
                end
            end     
            REQ_CMD_VALUE : begin
                if(count<=7 && count!=0)begin
                    // u_uart_rx <= data_8_bit[0];
                    data_8_bit<= {1'b0,data_8_bit[7:1]};
                    count<=count-1'b1;
                    send_start_info <= 1'b0;
                    
                end else if (count==0)begin
                    count<= 4'd9;
                    send_start_info <= 1'b1;
		    		if(FILE_READ == 1)
                    data_8_bit<= RAM[index][39:32];
		    		else 
                    data_8_bit  <= 8'hA2; //ADDRESS VAL
                    current_state <= SEND_ADDR;
                end
            end 
            SEND_ADDR : begin
                if(count<=7 && count!=0)begin
                    // u_uart_rx <= data_8_bit[0];
                    data_8_bit<= {1'b0,data_8_bit[7:1]};
                    count<=count-1'b1;
                    send_start_info <= 1'b0;
                    
                end else if (count==0)begin
                    count<= 4'd9;
                    send_start_info <= 1'b1;
		    		if(FILE_READ == 1)
                    data_8_bit<= RAM[index][31:24];
		    		else 
                    data_8_bit  <= 8'hDE; //DATA_SYMBOL0
                    current_state <= SEND_DATA_SYMB0;
                end
            end 
            SEND_DATA_SYMB0 : begin
                if(count<=7 && count!=0)begin
                    // u_uart_rx <= data_8_bit[0];
                    data_8_bit<= {1'b0,data_8_bit[7:1]};
                    count<=count-1'b1;
                    send_start_info <= 1'b0;
                    
                end else if (count==0)begin
                    count<= 4'd9;
                    send_start_info <= 1'b1;
		    		if(FILE_READ == 1)
                    data_8_bit<= RAM[index][23:16];
		    		else 
                    data_8_bit  <= 8'hAD; //DATA_SYMBOL1
                    current_state <= SEND_DATA_SYMB1;
                end
            end 
            SEND_DATA_SYMB1 : begin
                if(count<=7 && count!=0)begin
                    // u_uart_rx <= data_8_bit[0];
                    data_8_bit<= {1'b0,data_8_bit[7:1]};
                    count<=count-1'b1;
                    send_start_info <= 1'b0;
                    
                end else if (count==0)begin
                    count<= 4'd9;
                    send_start_info <= 1'b1;
		    		if(FILE_READ == 1)
                    data_8_bit<= RAM[index][15:8];
		    		else 
                    data_8_bit  <= 8'hDE; //DATA_SYMBOL2
                    current_state <= SEND_DATA_SYMB2;
                end
            end 
            SEND_DATA_SYMB2 : begin
                if(count<=7 && count!=0)begin
                    // u_uart_rx <= data_8_bit[0];
                    data_8_bit<= {1'b0,data_8_bit[7:1]};
                    count<=count-1'b1;
                    send_start_info <= 1'b0;
                    
                end else if (count==0)begin
                    count<= 4'd9;
                    send_start_info <= 1'b1;
		    		if(FILE_READ == 1)
                    data_8_bit<= RAM[index][7:0];
		    		else 
                    data_8_bit  <= 8'hAD; //DATA_SYMBOL3
                    current_state <= SEND_DATA_SYMB3;
                end
            end 
            SEND_DATA_SYMB3 : begin
		        
                if(count<=7 && count!=0)begin
                    // u_uart_rx <= data_8_bit[0];
                    data_8_bit<= {1'b0,data_8_bit[7:1]};
                    count<=count-1'b1;
                    send_start_info <= 1'b0;
                    
                end else if (count==0)begin
		    	    
                    count<= 4'd9;
                    send_start_info <= 1'b0;
		    		if(FILE_READ == 1)
                    data_8_bit<= RAM[index][55:48];
		    		else 
                    data_8_bit  <= 8'hCC; //magic
		    		
		    		if(FILE_READ ==1 && index <16)begin
                        current_state <= STOP;
		    		    index <= index + 'd1;
					    data_8_bit <= 'hzz;
		    		end else 
                        current_state <= STOP2;
                end
            end     
            STOP : begin
			    if(wait_10 == 'd20)begin
                    current_state <= IDLE;
					wait_10 <= 'd0;
                    data_8_bit<= RAM[index][55:48];
				end else 
				    wait_10 <= wait_10+1;
			
			
			
			end 
            STOP2 : begin
                
            // $finish();
		    	
            end         
            
            default : begin
                    count<= 4'd9;
                    data_8_bit  <= 8'hCC; //magic
                    current_state <= IDLE;
            end         
            endcase
		end 
        end 
    end 
end 



 initial begin
 
    start <= 1'b0;
    #100us
    start <= 1'b1;


  end 
  
  
  // Main Testing:
  initial
    begin
      // Tell UART to send a command (exercise TX)
      @(posedge r_Clock);
      @(posedge r_Clock);
      r_TX_DV   <= 1'b1;
      r_TX_Byte <= 8'h3F;
      @(posedge r_Clock);
      r_TX_DV <= 1'b0;

      // Check that the correct command was received
      // @(posedge w_RX_DV);
      if (w_RX_Byte == 8'h3F)
        $display("Test Passed - Correct Byte Received");
      else
        $display("Test Failed - Incorrect Byte Received");
 
  end 
         
  

always @(posedge r_Clock or  start)begin
  
      if(index == 'd16)begin
        // $finish();
        $display("index reached");
      end
    end
  initial 
  begin
    // Required to dump signals to EPWave
    $dumpfile("dump.vcd");
    $dumpvars(0);
  end
endmodule // drive_uart
