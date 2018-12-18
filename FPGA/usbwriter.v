module usbwriter
(
	FX3_ifclk,
	rst,
	change,
	pktend,
	DATA_i,
	fifo_empty,
	FX3_flags,
	FX3_pktend,
	FX3_slcs,
	FX3_slwr,
	FX3_fifoadr,
	FX3_fifodata,
	rd_fifo,
	cnt_fifo
);



input FX3_ifclk;
input rst;
input change;
input pktend;
input [128:0] DATA_i;
input fifo_empty;
input [1:0] FX3_flags;


output reg FX3_slcs;
output reg FX3_pktend;
output reg FX3_slwr;

output [1:0] FX3_fifoadr;
inout  [15:0] FX3_fifodata;
output reg rd_fifo;
output reg [8:0] cnt_fifo; // buffer FX3 8192  =>
// ============================================
// USB WRITE PROCESS
//
//
// ============================================


//Only Mode WRITE 
//assign FX3_slrd    = 1'b1;  //not used read
//assign FX3_sloe    = 1'b1;  //only out
assign FX3_fifoadr = 2'b00; //only 0


//flag A Thread_n_DMA_Ready      4 cycles
//flag B Thread_n_DMA_Watermark  Dependent on watermark level

reg [4:0] sm_state;
localparam  ZLP		= 4'd0;
localparam  IDLE     = 4'd1;
localparam  RD_FIFO  = 4'd2; 
localparam  WR_WORD1 = 4'd3;
localparam  WR_WORD2 = 4'd4;
localparam  WR_WORD3 = 4'd5;
localparam  WR_WORD4 = 4'd6;
localparam  WR_WORD5 = 4'd7;
localparam  WR_WORD6 = 4'd8;
localparam  WR_WORD7 = 4'd9;
localparam  WR_WORD8 = 4'd10;
localparam  WAIT1 	= 4'd11;
localparam  WAIT2 	= 4'd12;
localparam  WAIT3 	= 4'd13;
localparam  WAIT4 	= 4'd14; 

reg [15:0] FX3_data;
assign FX3_fifodata =(~FX3_slcs)?FX3_data:16'dz;
								


wire [15:0] data0 = DATA_i[15:0];
wire [15:0] data1 = DATA_i[31:16];
wire [15:0] data2 = DATA_i[47:32];
wire [15:0] data3 = DATA_i[63:48];
wire [15:0] data4 = DATA_i[79:64];
wire [15:0] data5 = DATA_i[95:80];
wire [15:0] data6 = DATA_i[111:96];
wire [15:0] data7 = DATA_i[127:112];

//Paked End Task
wire rst_f = (rst | (sm_state == WAIT2));

reg  f_zlp;
/*
always @(posedge zlp or posedge rst_f)
		if(rst_f)  f_zlp = 0;
		else 		
		if(zlp) f_zlp = 1;
*/	
// writes the fifo data out to usb
always @(posedge FX3_ifclk or posedge rst)
begin
   	
	if (rst) begin
		FX3_slwr    <=  1;
		FX3_slcs    <=  1;
		rd_fifo  	<=  0;
		FX3_pktend 	<=  1;
		cnt_fifo    <=  0;
		FX3_data    <=  0;
		f_zlp			<=  0;
		sm_state 	<=  IDLE;	 
	end else begin

	   if(change) f_zlp  <=1;
		
		  FX3_pktend <= ~(pktend & FX3_flags[1]);	
		
		case (sm_state)
	
		ZLP:  
		begin
				f_zlp			 <=  0;	//clear f_zlp
				if(cnt_fifo) begin
					FX3_slcs    <=  0;	
					FX3_pktend 	<=  0;    //pkt_end
					cnt_fifo    <=  0; 
					sm_state    <= WAIT1;
			   end
			   else
			     sm_state    <= IDLE;
		end
		
		IDLE: 
		begin
				FX3_slcs    <=  1;
				FX3_pktend  <=  1;
				FX3_slwr    <=  1;   		// deassert write 
				if(f_zlp)  
				   sm_state <= ZLP;
				else
				if (~fifo_empty & FX3_flags[0]) 
				begin 	
					 rd_fifo  <= 1; 			//read FIFO
					 sm_state <= RD_FIFO; 
				end
		end
			
		RD_FIFO :begin
					rd_fifo  <= 0;
					FX3_slcs <= 0;
					if (FX3_flags[0])
					sm_state   <= WR_WORD1;
					end

		WR_WORD1 :begin
					cnt_fifo <= cnt_fifo +1;
					FX3_slwr <= 1'b0;
					FX3_data <= data0;   
					sm_state <= WR_WORD2;
				end
			
		WR_WORD2 :begin 
					FX3_data <= data1;   	
					
					sm_state <= WR_WORD3;
					end
		
		WR_WORD3 :begin
					FX3_data <= data2; 	
					
					sm_state <= WR_WORD4;
            	end
		
		WR_WORD4 :begin
					FX3_data <= data3; 	
					
					sm_state <= WR_WORD5;
            	end
		WR_WORD5 :begin
					FX3_data <= data4; 	
					
					sm_state <= WR_WORD6;
            	end
		WR_WORD6 :begin
					FX3_data <= data5; 	
					sm_state <= WR_WORD7;
            	end
	
		WR_WORD7 : begin
					FX3_data <= data6;
					sm_state <= WR_WORD8;
            	end
		
		
		WR_WORD8 :	begin
					FX3_data <= data7; 
					if(FX3_flags[1])
							sm_state <= IDLE;
					else
							sm_state <= WAIT1;
					end
//
		WAIT1:	begin
					FX3_slcs    <=  1;
					FX3_slwr    <=  1;
					FX3_pktend  <=  1;
					sm_state <= WAIT2;	
					end	
		WAIT2:	begin
					sm_state <= WAIT3;	
					end
		WAIT3:	begin
					sm_state <= WAIT4;	
					end
		WAIT4:	begin
					sm_state <= IDLE;	
					end		endcase
	end
end
//---------------------------------------------
/*

wire [15:0] data0 = data_test[15:0];
wire [15:0] data1 = data_test[31:16];
wire [15:0] data2 = data_test[47:32];
wire [15:0] data3 = data_test[63:48];

wire [63:0] data_test;
 test_data(
 .rst(rst),
 .clk(rd_fifo),
 .data_o(data_test)
 );
 */
endmodule

//===============================================================
/*
module test_data(
	rst,
	clk,
	data_o
);

input rst,clk;
output reg [63:0] data_o;

reg [1:0] cnt_o;

always @(posedge clk or posedge rst)
begin
  if(rst) begin 
	  cnt_o <= 2'b0;
	  data_o<= 64'h7fff_0000_0000_7fff;
	  end
	  else 
  begin case (cnt_o)
   2'b00: begin data_o<=64'h7fff_0000_0000_7fff;  end
   2'b01: begin data_o<=64'h8000_0000_0000_8000;  end
   2'b10: begin data_o<=64'h7fff_0000_0000_7fff;  end
	2'b11: begin data_o<=64'h8000_0000_0000_8000;  end
	endcase

	cnt_o<=cnt_o+2'b01;
	  
	  end
end


endmodule
*/
