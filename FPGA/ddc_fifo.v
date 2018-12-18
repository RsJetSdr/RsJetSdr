module ddc_fifo
 #(
    parameter BASE = 0,
    parameter WIDTH_IN  = 32,
    parameter WIDTH_OUT = 128
	 
  )
  (
 input clk_wr,input rst,
 input set_stb, input [6:0] set_addr, input [31:0] set_data,
 input ddc_wr,
 input [WIDTH_IN-1:0] rx_ddc_iq,
 input clk_rd,
 input strobe_rd,
 
 output run_ddc,
 
 output [WIDTH_OUT-1:0] q_data,
 //time domain 
 output cnt_en,
 output rdempty,
 output change_task,
 output pktend,      
 
 output [7:0]debug
);

localparam Size_cnt = 13; //2 ^ 13 = 8192 IQ Sample (max  buffer)
localparam Size_cmd = 3;


//
wire [Size_cmd-1:0]  cmd_fifo;
wire [Size_cnt-1:0]  wire_cnt;
wire  changed;                   //changed cmd_register

//==== global register
reg  [Size_cnt-1:0]  size_burst;
reg  [16:0]	reg_rst_fifo;



//----------------------------------------------------------
assign  run_ddc = 1; //;  									//Enable DDC 
assign  cnt_en  		= (size_burst!=13'd0);      			//Counter not empty
assign  pktend  		= {reg_pkte[7] & ~reg_pkte[6]};
assign  change_task  = {sync_ch[1]  & ~sync_ch[0]};   

//flagS
wire    wr_fifo = (ddc_wr & cnt_en & (burst | raw));
wire    raw   = cmd_fifo[0];  
wire    burst = cmd_fifo[1];
wire    rst_fifo   = reg_rst_fifo[16];         //reset FIFO

//>>>>>>>> Task clk read domain <<<<<<<<<<<<<<
//local register
reg [1:0] sync_ch;
reg [8:0] reg_pkte; 
//====

always @(posedge clk_rd or posedge rst)
begin
				if(rst) begin
				reg_pkte   <= 0;
				size_burst   <= 0;
				sync_ch   <= 0;
				end
				else begin 
				
				reg_pkte <= {reg_pkte[7:0],cnt_en}; // сброс буфера по длине
				sync_ch  <= {sync_ch[0], changed }; // синхронизация 
				
				if(change_task)
						size_burst <= wire_cnt; //Set
				else  
				if(strobe_rd & burst & cnt_en)
				            size_burst <= size_burst - 4; //читаем по 2 выборки
				end				
				
end

//>>>>>>>> Task clk Write domain <<<<<<<<<<<<<<
always @(posedge clk_wr or posedge rst) 
 begin
   if(rst) begin
	reg_rst_fifo <= 0;
	end 
	else
	if(changed) 
	begin  
			reg_rst_fifo <= 0;	
	end
   else
   begin	
		if(ddc_wr) 
			reg_rst_fifo <={reg_rst_fifo[15:0],1'b1};
 	end			
end		
//---------------------------------
wire rd_empty;
assign rdempty = (rd_empty | ~cnt_en /*~rst_fifo*/);
fifo_ddc fifo(

	.aclr(~rst_fifo),
	.data(rx_ddc_iq),

	.rdclk(clk_rd),
	.rdreq(strobe_rd),
	
	.wrclk(clk_wr),
	.wrreq(wr_fifo),
	
	.q(q_data),
	.rdempty(rd_empty),

	.rdusedw()
	);

//>>>>>> Command Register <<<<<<<
setting_reg #(.my_addr(BASE),.width(16),.at_reset(16'h0800)) sr_cnt 
     (.clk(clk_wr),.rst(rst),.strobe(set_stb),.addr(set_addr),
      .in(set_data),.out({cmd_fifo,wire_cnt}),.changed(changed));
	
//>>>>>> Debug Data <<<<<<<<<<
assign debug ={
  // 1'b0,1'b0,
   rst_fifo,
   rdempty,
   wr_fifo, 
	strobe_rd,
	change_task,
	pktend,
	cnt_en
};
	
endmodule
