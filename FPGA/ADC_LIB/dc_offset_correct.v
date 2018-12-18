/*

dc_offset_correct.v DC Offset Correction module for the QS1R Receiver DDC
http://www.srl-llc.com

Copyright (C) 2007, 2008, 2009 Philip A Covington

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

The author can be reached by email at
p.covington@gmail.com

Why follow me to higher ground?
Lost as you swear I am.

*/

module dc_offset_correct(
input clk,
input reset,
input signed [15:0] data_in,
output signed [15:0] data_out,
output signed [15:0] dc_level_out
);

wire signed [15:0] dc_level = accumulator[31:16];
wire signed [15:0] corrected = data_in[15:0] - dc_level[15:0];

reg signed [31:0] accumulator;

always @(posedge clk) begin
	if (reset == 1'b1) begin
		accumulator <= 32'd0;		
	end else begin
		accumulator <= accumulator + {{16{corrected[15]}}, corrected[15:0]};
	end
end

assign data_out[15:0] = corrected[15:0];
assign dc_level_out = dc_level;

endmodule
