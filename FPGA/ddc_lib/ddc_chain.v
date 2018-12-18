//
// Copyright 2011-2013 Ettus Research LLC
//


//! The USRP digital down-conversion chain

module ddc_chain
  #(
    parameter BASE = 0,
    parameter DSPNO = 0,
    parameter WIDTH = 24
  )
  (input clk, input rst,
   input 	     set_stb, input [6:0] set_addr, input [31:0] set_data,

   // From RX frontend
   input [WIDTH-1:0] rx_fe_i,
   input [WIDTH-1:0] rx_fe_q,

   // To RX control
   output [15:0] sample_i,
   output [15:0] sample_q,
   input 	     run,
   output 	     strobe,
   output [23:0]     debug
   );

	assign debug = i_cordic_pipe[23:0];
	//{enable_hb1, enable_hb2, run, strobe, strobe_cic, strobe_hb1, strobe_hb2};
	/*
	wire [21:0] out_data_I;
	wire [WIDTH-1:0] rx_fe_i;
	cordic(
  .clock(clk),
  .frequency(32'h0800_0000),
  .in_data(16'h3FFF),
  .out_data_I(out_data_I),
  .out_data_Q()
  );
  assign rx_fe_i={out_data_I,2'd0};
  */
   localparam  cwidth = 25;
   localparam  zwidth = 24;

   wire [31:0] phase_inc;
   reg [31:0]  phase;

   wire [17:0] scale_factor;
   wire [cwidth-1:0] to_cordic_i, to_cordic_q;
   wire [cwidth-1:0] i_cordic, q_cordic;
   reg  [WIDTH-1:0] i_cordic_pipe, q_cordic_pipe;
   wire [WIDTH-1:0] i_cic, q_cic;


   wire        strobe_cic, strobe_hb1, strobe_hb2;
   wire        enable_hb1, enable_hb2;
   wire [7:0]  cic_decim_rate;

   wire [WIDTH-1:0]  rx_fe_i_mux, rx_fe_q_mux;
   wire        realmode;
   wire        swap_iq;
   wire        invert_i;
   wire        invert_q;

   setting_reg #(.my_addr(BASE+0),.at_reset(32'h4000_0000)) sr_0 // 100kHz @ 10MHz MCR
     (.clk(clk),.rst(rst),.strobe(set_stb),.addr(set_addr),
      .in(set_data),.out(phase_inc),.changed());
//  rate_pow = 		
//  (1 << 15) * std::pow(2, ceil_log2(rate_pow))*2./(1.648*rate_pow)
   setting_reg #(.my_addr(BASE+1),.width(18),.at_reset(79438)) sr_1 // Should include CORDIC and CIC gain compensation.
     (.clk(clk),.rst(rst),.strobe(set_stb),.addr(set_addr),
      .in(set_data),.out(scale_factor),.changed());

   setting_reg #(.my_addr(BASE+2), .width(10),.at_reset((1<<9)|(1<<8)|1)) sr_2 // Decim control
     (.clk(clk),.rst(rst),.strobe(set_stb),.addr(set_addr),
      .in(set_data),.out({enable_hb1, enable_hb2, cic_decim_rate}),.changed());

   setting_reg #(.my_addr(BASE+3), .width(4),.at_reset(4'b0010)) sr_3
     (.clk(clk),.rst(rst),.strobe(set_stb),.addr(set_addr),
      .in(set_data),.out({invert_i,invert_q,realmode,swap_iq}),.changed());


   // MUX so we can do realmode signals on either input
	switchIQ #(.WIDTH(WIDTH)) mux
(
	.clk(clk),.invert_i(invert_i),.invert_q(invert_q),.realmode(realmode),.swap_iq(swap_iq),
	.rx_fe_i(rx_fe_i),
   .rx_fe_q(rx_fe_q),
	.rx_fe_i_mux(rx_fe_i_mux), 
	.rx_fe_q_mux(rx_fe_q_mux)
);

   // NCO
   always @(posedge clk or posedge rst)
     if(rst)
       phase <= 0;
     else if(~run)
       phase <= 0;
     else
       phase <= phase + phase_inc;

   // CORDIC  24-bit I/O
   // (Algorithmic gain through CORDIC => 1.647 * 0.5 = 0.8235)
   // (Worst case gain through rotation => SQRT(2) = 1.4142)
   // Total worst case gain => 0.8235 * 1.4142 = 1.1646
   // So add an extra MSB bit for word growth.

   sign_extend #(.bits_in(WIDTH), .bits_out(cwidth)) sign_extend_cordic_i (.in(rx_fe_i_mux), .out(to_cordic_i));
   sign_extend #(.bits_in(WIDTH), .bits_out(cwidth)) sign_extend_cordic_q (.in(rx_fe_q_mux), .out(to_cordic_q));

   cordic_z24 #(.bitwidth(cwidth))
   cordic(.clock(clk), .reset(rst), .enable(run),
	  .xi(to_cordic_i),. yi(to_cordic_q), .zi(phase[31:32-zwidth]),
	  .xo(i_cordic),.yo(q_cordic),.zo() );

   always @(posedge clk) begin
      i_cordic_pipe[23:0] <= i_cordic[24:1];
      q_cordic_pipe[23:0] <= q_cordic[24:1];
   end


   // CIC decimator  24 bit I/O
   // Applies crude 1/(2^N) right shift gain compensation internally to prevent excesive downstream word growth.
   // Output gain is = algo_gain/(POW(2,CEIL(LOG2(algo_gain))) where algo_gain is = cic_decim_rate^4
   // Thus output gain is <= 1.0 and no word growth occurs.
   cic_strober cic_strober(.clock(clk),.reset(rst),.enable(run),.rate(cic_decim_rate),
			   .strobe_fast(1'b1),.strobe_slow(strobe_cic) );

   cic_decim #(.bw(WIDTH))
     decim_i (.clock(clk),.reset(rst),.enable(run),
	      .rate(cic_decim_rate),.strobe_in(1'b1),.strobe_out(strobe_cic),
	      .signal_in(i_cordic_pipe),.signal_out(i_cic));

   cic_decim #(.bw(WIDTH))
     decim_q (.clock(clk),.reset(rst),.enable(run),
	      .rate(cic_decim_rate),.strobe_in(1'b1),.strobe_out(strobe_cic),
	      .signal_in(q_cordic_pipe),.signal_out(q_cic));

	///////////////////////////////////////////////
	 //
	 // Legacy Decimation Filters from USRP2
	 //
	 ///////////////////////////////////////////////
	 wire [WIDTH-1:0] i_hb1, q_hb1;
	 wire [WIDTH-1:0] i_hb2, q_hb2;
	 // First (small) halfband  24 bit I/O
	 small_hb_dec #(.WIDTH(WIDTH)) small_hb_i
	   (.clk(clk),.rst(rst),.bypass(~enable_hb1),.run(run),
	    .stb_in(strobe_cic),.data_in(i_cic),.stb_out(strobe_hb1),.data_out(i_hb1));

	 small_hb_dec #(.WIDTH(WIDTH)) small_hb_q
	   (.clk(clk),.rst(rst),.bypass(~enable_hb1),.run(run),
	    .stb_in(strobe_cic),.data_in(q_cic),.stb_out(),.data_out(q_hb1));

	 // Second (large) halfband  24 bit I/O
	 wire [8:0] 	  cpi_hb = enable_hb1 ? {cic_decim_rate,1'b0} : {1'b0,cic_decim_rate};
	 hb_dec #(.WIDTH(WIDTH)) hb_i
	   (.clk(clk),.rst(rst),.bypass(~enable_hb2),.run(run),.cpi(cpi_hb),
	    .stb_in(strobe_hb1),.data_in(i_hb1),.stb_out(strobe_hb2),.data_out(i_hb2));

	 hb_dec #(.WIDTH(WIDTH)) hb_q
	   (.clk(clk),.rst(rst),.bypass(~enable_hb2),.run(run),.cpi(cpi_hb),
	    .stb_in(strobe_hb1),.data_in(q_hb1),.stb_out(),.data_out(q_hb2));

	 // Need to clip 1 bit here or we loose small signal performance out the truncated LSB's for worst case CIC gain cases.
	 wire strobe_unscaled_clip;
	 wire [17:0] i_unscaled_clip, q_unscaled_clip;

	 clip_reg #(.bits_in(19), .bits_out(18), .STROBED(1)) unscaled_clip_i
	   (.clk(clk), .in(i_hb2[WIDTH-1:WIDTH-19]), .strobe_in(strobe_hb2), .out(i_unscaled_clip[17:0]), .strobe_out(strobe_unscaled_clip));
	 clip_reg #(.bits_in(19), .bits_out(18), .STROBED(1)) unscaled_clip_q
	   (.clk(clk), .in(q_hb2[WIDTH-1:WIDTH-19]), .strobe_in(strobe_hb2), .out(q_unscaled_clip[17:0]), .strobe_out());

	 //scalar operation (gain of 6 bits)
	 wire [35:0] 	  prod_i, prod_q;

	 MULT_MACRO #(
		      .LATENCY(1),         // Desired clock cycle latency, 0-4
		      .WIDTH_A(18),        // Multiplier A-input bus width, 1-25
		      .WIDTH_B(18))        // Multiplier B-input bus width, 1-18
	 mult_i (.P(prod_i),             // Multiplier output bus, width determined by WIDTH_P parameter
		.A(i_unscaled_clip),// Multiplier input A bus, width determined by WIDTH_A parameter
		.B(scale_factor),       // Multiplier input B bus, width determined by WIDTH_B parameter
		.CE(strobe_unscaled_clip),        // 1-bit active high input clock enable
		.CLK(clk),              // 1-bit positive edge clock input
		.RST(rst));             // 1-bit input active high reset

	 MULT_MACRO #(
		      .LATENCY(1),         // Desired clock cycle latency, 0-4
		      .WIDTH_A(18),        // Multiplier A-input bus width, 1-25
		      .WIDTH_B(18))        // Multiplier B-input bus width, 1-18
	 mult_q (.P(prod_q),             // Multiplier output bus, width determined by WIDTH_P parameter
		.A(q_unscaled_clip),// Multiplier input A bus, width determined by WIDTH_A parameter
		.B(scale_factor),       // Multiplier input B bus, width determined by WIDTH_B parameter
		.CE(strobe_unscaled_clip),        // 1-bit active high input clock enable
		.CLK(clk),              // 1-bit positive edge clock input
		.RST(rst));             // 1-bit input active high reset

	 reg 		  		strobe_scaled;
	 wire 		  	strobe_clip;
	 wire [32:0] 	i_clip, q_clip;

	 always @(posedge clk)  strobe_scaled <= strobe_unscaled_clip;

	 clip_reg #(.bits_in(36), .bits_out(33), .STROBED(1)) clip_i
	   (.clk(clk), .in(prod_i[35:0]), .strobe_in(strobe_scaled), .out(i_clip), .strobe_out(strobe_clip));
	 clip_reg #(.bits_in(36), .bits_out(33), .STROBED(1)) clip_q
	   (.clk(clk), .in(prod_q[35:0]), .strobe_in(strobe_scaled), .out(q_clip), .strobe_out());

	 round_sd #(.WIDTH_IN(33), .WIDTH_OUT(16)) round_i
	   (.clk(clk), .reset(rst), .in(i_clip), .strobe_in(strobe_clip), .out(sample_i), .strobe_out(strobe));
	 round_sd #(.WIDTH_IN(33), .WIDTH_OUT(16)) round_q
	   (.clk(clk), .reset(rst), .in(q_clip), .strobe_in(strobe_clip), .out(sample_q), .strobe_out());
endmodule // ddc_chain

//================================================================================
module switchIQ
#(
    parameter WIDTH = 24
  )
(
	input clk,invert_i,invert_q,realmode,swap_iq,
	input [WIDTH-1:0] rx_fe_i,
   input [WIDTH-1:0] rx_fe_q,
	output reg [WIDTH-1:0]  rx_fe_i_mux, 
	output reg [WIDTH-1:0]  rx_fe_q_mux
);
  // MUX so we can do realmode signals on either input
   always @(posedge clk)
     if(swap_iq)
       begin
         rx_fe_i_mux <= invert_i ? ~rx_fe_q : rx_fe_q;
         rx_fe_q_mux <= realmode ? {WIDTH{1'b0}} : invert_q ? ~rx_fe_i : rx_fe_i;
       end
     else
       begin
			rx_fe_i_mux <= invert_i ? ~rx_fe_i : rx_fe_i;
         rx_fe_q_mux <= realmode ? {WIDTH{1'b0}} : invert_q ? ~rx_fe_q : rx_fe_q;
       end
endmodule
