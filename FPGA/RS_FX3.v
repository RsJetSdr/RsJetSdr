module DC_Correct  #(parameter WIDTH = 16)
  (
   input clk,
   input [WIDTH-1:0] DataIn,
	output [WIDTH:0] DataOut
   );
	
// 1-z^{-2} filter

reg signed [WIDTH-1:0] cavity0=0;
reg signed [WIDTH-1:0] cavity1=0;
reg signed [WIDTH-1:0] cavity2=0;
reg signed [WIDTH:0] cav_nodc;

//always @(negedge adc_clk) begin
always @(posedge clk) begin
	cavity0 <= DataIn;  // waste one cycle getting data into fabric
	cavity1 <= cavity0;
	cavity2 <= cavity1;
	cav_nodc <= cavity0 - cavity2;
end 	
	
assign DataOut = cav_nodc;	

endmodule	


//`include "fpga_regs_standard.v"  

module RS_FX3(

//---- ADC ------
input 		adc_clk,
input 		adc_clk_n,
//==== to PLL (extern Hardware)
output      adc_clk_buf,
input       in_clk_pll,
//
input	[15:0] adc_data,
input	      adc_FO,
output 		adc_gain,
// ---------- 
output 		clk_FX3,
input 		clk_FX3_In,

input [1:0] FX3_flags,
//output	   FX3_slrd, //Pin 71 
//output		FX3_sloe, //Pin 69
output		FX3_pktend,
output	   FX3_slcs,
output	   FX3_slwr,
output  [1:0]FX3_fifoadr,	
inout  [15:0]FX3_fifodata,

//host spi
input      spi_ncs,
input      spi_sclk,
input      spi_mosi,
output     spi_miso,
//adc spi
output    adc_sclk,
output	 adc_mosi,
output	 adc_ncs,
//atn spi
output    atn_sclk,
output	 atn_mosi,
output	 atn_ncs,
//
output notEmpty,
input rx_in,tx_in,
output rx_out,tx_out,
output Led
//debug
//output	[15:0] debug_pins

);

assign rx_out=rx_in;
assign tx_out=tx_in;


wire rd_fifo;
wire double_clk;



wire adc_rand,adc_dither,local_reset_bit,run_ddc,sw_in;

wire [31:0] ctrl_reg;
assign adc_rand 		  =  ctrl_reg[0];  //RANDOM
assign adc_dither 	  =  ctrl_reg[1];  //DITHER
assign adc_gain        =  ctrl_reg[2];	 //GAIN
assign sw_in 			  =  ctrl_reg[30]; //ADC_BYPASS
assign local_reset_bit =  ctrl_reg[31]; //MASTER_RESET


//addres register
localparam MB_RX_CTRL 		= 7'h0; //Control
localparam MB_RAW_CTRL 		= 7'h1;

localparam MB_RX_FREQ_NCO  = 7'h2; 

localparam MB_RX_FREQ      = 7'h4;
localparam MB_RX_SCALE_IQ  = 7'h5;
localparam MB_RX_DECIM     = 7'h6;
localparam MB_RX_MUX       = 7'h7;

localparam MB_ADC_REG      = 7'h8;
localparam MB_ATTN_REG     = 7'h9;
//clock
//assign adc_clk_buf = adc_clk;
ddr ddr_clk(
	.datain_h(1),
	.datain_l(0),
	.outclock(adc_clk),
	.dataout(adc_clk_buf)
	);

//==========================================
wire local_reset;
wire m_reset;	
assign Led = local_reset;
reset_generator 
(
.clk(adc_clk),
.local_reset_bit_i(local_reset_bit),
.local_reset_o(local_reset),
.initial_reset_o(m_reset)
);
//===================================
setting_reg #(.my_addr(MB_RX_CTRL),.at_reset(32'h8000_0000))	control_reg
  ( 
    .clk(adc_clk),.rst(m_reset),
	 .strobe(bus_wr),.addr(bus_addr),.in(bus_wdata), 
	 .out(ctrl_reg), 
	 .changed()
);

//==========================================
wire clk_FX3_i;
wire clk_gpif=clk_FX3_i;
pll (
	.inclk0(in_clk_pll),
	.c0(clk_FX3_i),          // clk_FX3 = (adc_clk * 6)/5 => 1.2
//	.c1(double_clk),       // adc_clk * 2
	.locked()
);
//=============================================
ddr ddr_Fx3(
	.datain_h(0),
	.datain_l(1),
	.outclock(clk_FX3_i), //input
	.dataout(clk_FX3)
	);	

//==================================
wire [6:0]  bus_addr;
wire [31:0] bus_wdata;
wire 			bus_wr;
usbrx_spi (
		.clk(adc_clk),   			//    : in  std_logic;
		.reset(m_reset), 			//     : in  std_logic;
//-- SPI interface
		.spi_ncs(spi_ncs), 		//  : in  std_logic;
		.spi_sclk(spi_sclk),		//  : in  std_logic;
		.spi_mosi(spi_mosi),		//  : in  std_logic;
		.spi_miso(spi_miso),		//  : out std_logic;
//-- bus interface
		.bus_rena(), 				// : out std_logic;
		.bus_wena(bus_wr), 		// : out std_logic;
		.bus_addr(bus_addr), 	// : out unsigned(6 downto 0);
		.bus_rdata(),				// : in  std_logic_vector(31 downto 0);
		.bus_wdata(bus_wdata) 	// : out std_logic_vector(31 downto 0)
	);

//=====================================
wire [15:0] data_adc;
derandomizer 
(
	.clka(adc_clk),
	.local_reset(m_reset),
	.ADC_rand_i(adc_rand),
	.ADC_i(adc_data),
	.ADC_o(data_adc)
);

//====== DC offset Correct  ============
/*
wire [15:0] adc_correct;

dc_offset_correct (
	.clk(adc_clk),
	.reset(m_reset),
	.data_in(data_adc),
	.data_out(adc_correct),
	.dc_level_out()
);
*/

wire [16:0] cav_nodc;

DC_Correct #(.WIDTH(16))
(
	.clk(adc_clk),
   .DataIn(data_adc),
	.DataOut(cav_nodc)
);

//=============================================
ddc_chain #(.BASE(MB_RX_FREQ)) ddc(
   .clk(adc_clk),
//	.rst(m_reset),
   .rst(local_reset),
	.set_stb(bus_wr),.set_addr(bus_addr),.set_data(bus_wdata),
// From RX frontend

//Input real Data	
//  .rx_fe_i({adc_correct,8'h0}),
//  .rx_fe_q({adc_correct,8'h0}),
  
  
  .rx_fe_i({cav_nodc,7'h0}),
  .rx_fe_q({cav_nodc,7'h0}),
  
	
 //Output DDC Data
   .sample_i(rx_ddc_iq[31:16]),
   .sample_q(rx_ddc_iq[15:0]),
   .run(run_ddc),
   .strobe(strobe_ddc),
   .debug()
   );
//=====================================
/*
wire [31:0] q_iq;
Test_DataIQ(
	//.clk(adc_clk),
	.clk(strobe_ddc),
	.rst(m_reset),
	.q_IQ(q_iq)
);
//=====================================
wire [31:0] ddc_iq;
 sel_bus #(.WIDTH(32))
 
 (
 .sw_in(sw_in),
 .in_1(rx_ddc_iq),
 .in_2(q_iq),
 .out(ddc_iq)
 );
 */
 //====================================
 wire [7:0] debug_fifo;
wire [127:0] data127;
wire rdempty;
wire pktend;
wire change_task;
wire strobe_ddc;
wire [31:0] rx_ddc_iq;
 
ddc_fifo  #(.BASE(MB_RAW_CTRL)) source
  (
	.clk_wr(adc_clk),
//	.rst(m_reset),
	.rst(local_reset),
	.set_stb(bus_wr),.set_addr(bus_addr),.set_data(bus_wdata),
	.ddc_wr(strobe_ddc),

	.rx_ddc_iq(rx_ddc_iq),

	.clk_rd(clk_gpif),
	.strobe_rd(rd_fifo),
	.run_ddc(run_ddc),
	.q_data(data127),
	.cnt_en(notEmpty),
	.rdempty(rdempty),
	.change_task(change_task),
	.pktend(pktend),
	.debug(debug_fifo)
);
//====================================
wire [10:0]cnt_out;
usbwriter 
(
	.FX3_ifclk(clk_gpif),
//	.rst(m_reset),
	.rst(local_reset),
   .change(change_task),
	.pktend(pktend),        
	.DATA_i(data127),
	.fifo_empty(rdempty),
	.FX3_flags(FX3_flags),
 	.FX3_pktend(FX3_pktend),
	.FX3_slcs(FX3_slcs),
	.FX3_slwr( FX3_slwr),
	.FX3_fifoadr(FX3_fifoadr),
	.FX3_fifodata(FX3_fifodata),
	.rd_fifo(rd_fifo),
	.cnt_fifo(cnt_out)
);	

//========================================	
adc_spi #(.ADDR(MB_ADC_REG))
(
	.clock(adc_clk),.reset(m_reset),
	.enable(bus_wr),.addr(bus_addr),.data(bus_wdata),
	.ss(adc_ncs), 		   //slave select
	.sclk(adc_sclk), 		//spi clock
	.mosi(adc_mosi) 			//master out, slave in
);
//========================================	
atn_spi #(.ADDR(MB_ATTN_REG)) (
	.clock(adc_clk),.reset(m_reset), 
	.enable(bus_wr),.addr(bus_addr),.data(bus_wdata),
	.ss(atn_ncs), 		   //slave select
	.sclk(atn_sclk), 		//spi clock
	.mosi(atn_mosi) 		 //master out, slave in
);
 

 
wire Trigger =FX3_flags[0];
//==============================

assign debug_pins ={
  	debug_fifo[6:0],//7
	Trigger};       //1 

endmodule




module Test_DataIQ(
clk,
rst,
q_IQ
);
input clk,rst;
output reg [31:0] q_IQ;
reg [1:0] cnt_o;

always @(negedge clk or posedge rst)
begin
  if(rst) begin 
	  cnt_o <= 2'b0;
	  q_IQ<= 32'h7fff_0000;
	  end
	  else 
  begin 
  case (cnt_o)
   2'b00: begin q_IQ<=32'h7fff_0000;  end
	2'b01: begin q_IQ<=32'h0000_7fff;  end
   2'b10: begin q_IQ<=32'h8000_0000;  end
	2'b11: begin q_IQ<=32'h0000_8000;  end
   endcase

	cnt_o<=cnt_o+2'b01;
 end	
 end
 
endmodule
