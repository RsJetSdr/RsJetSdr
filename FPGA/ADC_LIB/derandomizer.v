/*

derandomizer.v Derandomizer module for the QS1R Receiver DDC
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

module derandomizer
(
clka,
local_reset,
ADC_rand_i,
ADC_i,
ADC_o
);

input clka;
input local_reset;
input ADC_rand_i;
input  [15:0] ADC_i;
output reg [15:0] ADC_o;

// ============================================
// ADC Data Register and De-Randomizer
//
// The adc bus is registered to prevent
// data glitches
// ============================================

//reg [15:0] ADC_r;

always @(posedge clka, posedge local_reset)
begin
	if (local_reset) begin
		ADC_o <= 16'd0;
	end else begin
		
		if (ADC_rand_i) begin
			
			if (ADC_i[0]) begin
						ADC_o <= {~ADC_i[15:1], ADC_i[0]};
			end else begin
						ADC_o <= ADC_i[15:0];
			end		
		end else begin
						ADC_o <= ADC_i[15:0];
		end
	end
end

//assign ADC_o = ADC_r;

//---------------------------------------------

endmodule
