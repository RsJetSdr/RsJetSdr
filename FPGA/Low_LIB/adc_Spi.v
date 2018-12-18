module adc_spi #( parameter ADDR = 0)
(
 clock,    	//system clock
 reset,  	//asynchronous reset
 enable,   	//initiate transaction
 addr,
 data, 	   //data to transmit
 ss, 		   //slave select
 sclk, 		//spi clock
 mosi 		//master out, slave in
);

input clock;
input reset;
input enable;
input  [6:0]addr;
input [31:0]data;
output ss;
output sclk;
output mosi;


spi_master 
	#(
		.slaves  (1),
		.d_width (16)
		)
	spi_DAC (
	 .clock(clock),        //system clock
    .reset(reset),         //asynchronous reset
    .enable(enable & (addr==ADDR)),  //initiate transaction
    .cpol(0),            //spi clock polarity
    .cpha(0),            //spi clock phase
    .cont(0),   //continuous mode command
    .clk_div(4), //system clock cycles per 1/2 period of sclk
    .addr(0),    //address of slave
    .tx_data(data), //data to transmit
    .sclk(sclk), //spi clock
    .ss_n(ss), //slave select
    .mosi(mosi), //master out, slave in
    .busy()   // data ready signal
);
endmodule
