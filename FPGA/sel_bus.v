module sel_bus(sw_in,in_1,in_2,out);
parameter WIDTH = 16;  
input sw_in;
input [WIDTH-1:0] in_1,in_2;
output [WIDTH-1:0] out;

	assign out = (sw_in) ? in_2 : in_1;

endmodule
