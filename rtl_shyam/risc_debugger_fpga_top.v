
module risc_debugger_fpga_top  (
        input    clk_i,
        input   wire rst_i,
        //input clk_25mhz,

	//UART1 interface
	input uart_rx ,
	output uart_tx ,
	output uart_busy,
	


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
	input sw7 ,

	//DEbug interface
	//output [31:0]debug_bus,
	//LED interface
	output user_led0 ,
	output user_led1 ,
	output user_led2 ,
	output user_led3 ,
	output user_led4 ,
	output user_led5 ,
	output user_led6 
	
);
wire clk_25mhz;
wire pll_locked;
   cv32e40p_fpga_top 
    cv32e40p_fpga_top_i
        (

         .clk_i                  ( clk_i     ),
         .rst_i                  ( rst_i     ),
         .clk_25mhz              ( clk_25mhz ),
         .pll_locked (pll_locked),


	//UART interface
	    .uart_rx    (uart_rx  ),
	    .uart_tx    (uart_tx  ),
	    .uart_busy    (uart_busy  ),//LED7
	    .user_led0    (user_led0  ),
	    .user_led1    (user_led1  ),
	    .user_led2    (user_led2  ),
	    .user_led3    (user_led3  ),
	    .user_led4    (user_led4  ),
	    .user_led5    (user_led5  ),
	    .user_led6    (user_led6  ),
	    
	    
	   // .debug_bus    (debug_bus  ),
	    
	    .pb0    (pb0  ),
	    .pb1    (pb1  ),
	    .pb2    (pb2  ),
	    .pb3    (pb3  ),
	    
	    
	    .sw0    (sw0  ),
	    .sw1    (sw1  ),
	    .sw2    (sw2  ),
	    .sw3    (sw3  ),
	    .sw4    (sw4  ),
	    .sw5    (sw5  ),
	    .sw6    (sw6  ),
	    .sw7    (sw7  )
	    

       ); 
//PLL Instance
 clock_25mhz_pll i_clock_25mhz_pll
   (
    // Clock out ports
    .clk_out1(clk_25mhz),     // output clk_out1
    // Status and control signals
    .reset(rst_i), // input reset
    .locked(pll_locked),       // output locked
   // Clock in ports
    .clk_in1(clk_i));      // input clk_in1

endmodule
