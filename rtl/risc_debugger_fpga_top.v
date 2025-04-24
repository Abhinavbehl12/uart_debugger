
module risc_debugger_fpga_top  (
        input   wire clk_i,
        input   wire rst_i,
        input clk_25mhz,

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

	//LED interface
	output user_led0 ,
	output user_led1 ,
	output user_led2 ,
	output user_led3 ,
	output user_led4 ,
	output user_led5 ,
	output user_led6 
	
);

   cv32e40p_fpga_top 
    cv32e40p_fpga_top_i
        (

         .clk_i                  ( clk_i     ),
         .rst_i                  ( rst_i     ),
         .clk_25mhz              ( clk_25mhz ),


	//UART interface
	    .uart_rx    (uart_rx  ),
	    .uart_tx    (uart_tx  )

       ); 

endmodule
