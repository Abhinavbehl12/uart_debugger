// Set Parameter CLKS_PER_BIT as follows:
// CLKS_PER_BIT == (Frequency of i_Clock)/(Frequency of UART)
// Example: 25 MHz Clock, 115200 baud UART
// (25000000)/(115200) == 217


module uart_controller #(
parameter ENABLE_PARITY    = 0,
parameter MAKE_PARITY_ODD  = 0,
parameter BAUD_RATE        = 115200,
parameter CORE_CLK_FREQ    = 25000000 //25 MHZ

)(
input   core_clk,
input   core_rst,

//DATA INTERFACE - Connected to DEBUGGER/REGISTER interface
input [7:0]   data_in,
input   data_valid_in,
output reg [7:0]   data_out,
output   data_valid_out,
output   overflow,
output   parity_error,

//UART INTERFACE - Connected to RS232
input rx,
output tx,
output tx_busy,

//DEBUG interface
output [31:0] debug_bus_u
);

reg rx_d ;
reg rx_2d;
reg rx_3d;
reg enable_div16_rx;
reg top16_rx       ;
reg [7:0]div16_rx       ;
reg top16_tx       ;
reg [7:0]div16_tx       ;
reg toptx ;
reg toprx;
reg [7:0]clkdiv;
reg [7:0]rxdiv;
reg [8:0]tx_reg  ;
reg [7:0]txbitcnt;
reg [2:0]txfsm   ;
reg      txbusy  ;
reg [7:0]regdin  ; 

reg [7:0]rx_reg  ;
// reg [7:0]data_out    ;
reg [7:0]rxbitcnt;
reg [2:0]rxfsm   ;
reg [2:0]rxfsm_d   ;
reg rxrdyi       ;
reg clrdiv       ;
reg rxerr        ;
reg rx_parity    ;
wire calc_parity    ;

wire [7:0]divi_tx;
wire [7:0]divi_rx;
wire [7:0]divisor;
wire parity;
wire parity_err;
localparam RX_IDLE  = 3'd1;
localparam START_RX = 3'd2;
localparam EDGE_RX  = 3'd3;
localparam SHIFT_RX = 3'd4;
localparam STOP_RX  = 3'd5;
localparam RX_OVF   = 3'd6;

localparam TX_IDLE  = 3'd1;
localparam LOAD_TX  = 3'd2;
localparam SHIFT_TX = 3'd3;
localparam STOP_TX  = 3'd4;


//Sample the rx data to remove metasatablility
always@(posedge core_clk)begin
    if(core_rst)begin
        rx_d    <= 1'b1;
        rx_2d   <= 1'b1;
        rx_3d   <= 1'b1;
	end else begin
        rx_d    <= rx;
        rx_2d   <= rx_d;
        rx_3d   <= rx_2d;
	end 
end 

    // // //////////////////////////
    // // Baud rate selection
    // // //////////////////////////
assign    divi_tx =  8'b00001001;   //  //9 (10 - 1) counting from 0 //9 bits of data needed to be sent
assign    divi_rx =  8'b00000100;   //  //4 ( 5 - 1) counting from 0
assign    divisor =  (CORE_CLK_FREQ/(10*BAUD_RATE) - 1);
 
// // //////////////////////////
// // Clk16 Clock Generation
// // //////////////////////////
always@(posedge core_clk)begin
    if (core_rst) begin
        enable_div16_rx      <=     1'b0;
        top16_rx             <=     1'b0;
        div16_rx             <=     8'd0;
    end else begin
        top16_rx             <=     1'b0;
        if (rx_3d == 1'b0 && rx_2d == 1'b1 && rxfsm == RX_IDLE) begin //Searching for posedge of RX DATA or start bit 
            enable_div16_rx <= 1'b1;
        end else if (rxfsm == RX_IDLE && rxfsm_d != RX_IDLE) begin
            enable_div16_rx <= 1'b0;
        end 
        
        if (enable_div16_rx ==  1'b1) begin
            if (div16_rx == divisor) begin
                div16_rx     <= 8'd0;
                top16_rx     <= 1'b1;
            end else
                div16_rx     <= div16_rx + 1;
            end 
        else
            div16_rx         <= divisor - 1;
        end     
end 
    
always@(posedge core_clk)begin
    if (core_rst) begin
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

always@(posedge core_clk)begin
    if (core_rst) begin
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

    // // //////////////////////////////
    // // Rx Sampling Clock Generation
    // // //////////////////////////////
always@(posedge core_clk)begin
        if (core_rst) begin
            toprx             <=     1'b0;
            rxdiv             <=     8'd0;
        end else begin
            toprx             <=     1'b0;
            if (clrdiv==1'b1) begin
                rxdiv         <=     8'd0;
            end else if (top16_rx==1'b1) begin
                if (rxdiv == divi_rx) begin
                    rxdiv     <=     8'd0;
                    toprx     <=     1'b1;
                end else
                    rxdiv     <=     rxdiv + 1;
                
            end
        end
    end

    // // //////////////////////////
    // // Transmit State Machine
    // // //////////////////////////
 assign   tx            =     tx_reg[0];
 assign   tx_busy       =     txbusy;
 assign   parity        =    ENABLE_PARITY ? (MAKE_PARITY_ODD ^ (data_in[0] ^ data_in[1] ^ data_in[2] ^ data_in[3] ^ data_in[4] ^ data_in[5] ^ data_in[6] ^ data_in[7]))
                                              : 1'b1;
    
always@(posedge core_clk)begin
        if (core_rst) begin
            tx_reg                     <=    9'd0;
            txbitcnt                   <=    8'd0;
            txfsm                      <=    TX_IDLE;
            txbusy                     <=    1'b0;
            regdin                     <=    8'd0;
        end else begin
            txbusy                     <=    1'b1; //// except  explicitly 1'b0
            case (txfsm)
             TX_IDLE :
                if (data_valid_in==1'b1) begin
                    // // latch the input data immediately.
                    regdin             <=    data_in;
                    txbusy             <=    1'b1;
                    txfsm              <=    LOAD_TX;
                end else
                    txbusy             <=    1'b0;
             LOAD_TX :
                if (toptx==1'b1) begin
                    txfsm              <=     SHIFT_TX;
                    txbitcnt           <=     8'd9 + ENABLE_PARITY; //// start + data
                    tx_reg             <=     {parity , regdin , 1'b1};//start + data + parity
                end 
             SHIFT_TX :
                if (toptx==1'b1) begin
                    txbitcnt           <=     txbitcnt - 1;
                    tx_reg             <=     {1'b0 , tx_reg [8:1]};
                    if (txbitcnt==1'b1) begin
                        txfsm          <=     STOP_TX;
                    end 
                end 
             STOP_TX :
                if (toptx==1'b1) begin
                    txfsm              <=     TX_IDLE;
                end 
             default :
                txfsm                  <=     TX_IDLE;
            endcase
        end 
    end 

    // // ////////////////////////
    // // RECEIVE State Machine
    // // ////////////////////////
 assign    calc_parity        =  ENABLE_PARITY ?  (MAKE_PARITY_ODD ^ (rx_reg[0] ^ rx_reg[1] ^ rx_reg[2] ^ rx_reg[3] ^ rx_reg[4] ^ rx_reg[5] ^ rx_reg[6] ^ rx_reg[7]) ) 
                                          :	1'b0;
    
always@(posedge core_clk)begin
    if (core_rst) begin
        rx_reg                 <=    8'd0;
        data_out                   <=    8'd0;
        rxbitcnt               <=    8'd0;
        rxfsm                  <=    RX_IDLE;
        rxrdyi                 <=    1'b0;
        clrdiv                 <=    1'b0;
        rxerr                  <=    1'b0;
        rx_parity              <=    1'b0;
    end else begin
        rxfsm_d                <=    rxfsm;
        clrdiv                 <=    1'b0; // default value
        // reset error  a word has been received ok:
        if (rxrdyi==1'b1) begin
            rxerr              <=    1'b0;
            rxrdyi             <=    1'b0;
        end
        case (rxfsm) 
         RX_IDLE :
		 begin //// wait on start bit
            rxbitcnt          <=    8'd0;
            if (top16_rx==1'b1) begin
                if (rx_2d==1'b1) begin //Synchronise on start bit detection
                    rxfsm     <=     EDGE_RX;
                    //clrdiv    <=    1'b1; // synchronize the divisor
                end  // else false start, stay in idle
            end 
         end 
         START_RX : 
		 begin// wait on first data bit
            if (toprx == 1'b1 && clrdiv == 1'b0) begin
                if (rx_2d==1'b1) begin // framing error
                    rxfsm     <=     RX_OVF;
                    //-report "start bit error." severity note;
                end else begin
                    rxfsm     <=     EDGE_RX;
                end 
            end 
         end 
         EDGE_RX : 
		 begin // should be near rx edge
            if (toprx == 1'b1 && clrdiv == 1'b0) begin
                rxfsm         <=     SHIFT_RX;
                if (rxbitcnt == 8'b00001000 | (rxbitcnt == 8'd0 & ENABLE_PARITY)) begin
                    rxfsm     <=     STOP_RX;
                end else begin
                    rxfsm     <=     SHIFT_RX;
                end 
            end 
         end 
         SHIFT_RX : 
		 begin // sample data !
            if (toprx == 1'b1 && clrdiv == 1'b0) begin
                rxbitcnt      <=     rxbitcnt + 1;
                // shift right :
                if (rxbitcnt[3] == 1'b0) begin
                    rx_reg    <=     {rx_2d , rx_reg [7:1]};
                end else begin
                    rx_parity <=    rx_2d;
                end 
                rxfsm         <=     EDGE_RX;
            end 
         end 
         STOP_RX : 
		 begin // during stop bit
               
            if (toprx == 1'b1 && clrdiv == 1'b0) begin
                data_out      <=     rx_reg;
                rxrdyi        <=    1'b1;
                rxfsm         <=     RX_IDLE;    
                clrdiv        <=    1'b1; // synchronize the divisor
            end 
         end 
         RX_OVF : 
		 begin // overflow / error
            rxerr             <=     1'b1;
            if (rx_2d==1'b1) begin
                rxfsm         <=     RX_IDLE;
                    clrdiv    <=    1'b1; // synchronize the divisor
            end 
         end 
         default :
            rxfsm             <=     RX_IDLE;
        endcase
    end 
end 
    
assign    overflow        =  rxerr;
assign    parity_err      =  (rx_parity != calc_parity) ? rxrdyi : 1'b0;
assign    data_valid_out  =  rxrdyi;
assign    debug_bus_u     =  {
                             data_out,       // 26:19
                             data_valid_out ,      //18
                             core_rst,      //17
                             rx_reg   ,      //16:9
                             toprx    ,      //8
                             rx_2d    ,      //7
                             top16_rx    ,      //6
                             rxrdyi   ,      //5
                             clrdiv   ,      //4
                             rxerr    ,      //3
                             rxfsm};          //2:0
endmodule        
