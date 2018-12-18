/*
resetgenerator.v Reset Generator module 
*/

module reset_generator
(
clk,
local_reset_bit_i,
local_reset_o,
initial_reset_o
);

input clk;
input local_reset_bit_i;
output local_reset_o;
output reg initial_reset_o;

// ============================================
// Local Reset Control
// ============================================

reg local_reset;
reg [31:0] local_reset_counter=0;

always @(posedge clk )
begin
		if (local_reset_counter >= 32'h800000) begin
			local_reset <= local_reset_bit_i;
			initial_reset_o <= 1'b0;
		end else begin
			local_reset_counter <= local_reset_counter + 1;
			local_reset <= 1'b1;
			initial_reset_o <= 1'b1;
		end
end

assign local_reset_o = local_reset;

//---------------------------------------------

endmodule
