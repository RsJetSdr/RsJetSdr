// replacing Xilinx's macro with Altera's LPM

`timescale 1ps/1ps

module MULT_MACRO (

  CE,
  RST,
  CLK,
  A,
  B,
  P);

  parameter   LATENCY = 1;
  parameter   WIDTH_A = 16;
  parameter   WIDTH_B = 16;

  localparam  WIDTH_P = WIDTH_A + WIDTH_B;

  input                   CE;
  input                   RST;
  input                   CLK;

  input   [WIDTH_A-1:0]   A;
  input   [WIDTH_B-1:0]   B;
  output  [WIDTH_P-1:0]   P;

  lpm_mult #(
    .lpm_type ("lpm_mult"),
    .lpm_widtha (WIDTH_A),
    .lpm_widthb (WIDTH_B),
    .lpm_widthp (WIDTH_P),
    .lpm_representation ("SIGNED"),
    .lpm_pipeline (LATENCY))
  i_lpm_mult (
    .clken (CE),
    .aclr (RST),
    .sum (1'b0),
    .clock (CLK),
    .dataa (A),
    .datab (B),
    .result (P));

endmodule