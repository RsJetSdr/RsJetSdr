
// synopsys translate_off
`timescale 1 ps / 1 ps
// synopsys translate_on
module ddr (
	datain_h,
	datain_l,
	outclock,
	dataout);

	input	[0:0]  datain_h;
	input	[0:0]  datain_l;
	input	  outclock;
	output	[0:0]  dataout;

	wire [0:0] sub_wire0;
	wire [0:0] dataout = sub_wire0[0:0];

	altddio_out	ALTDDIO_OUT_component (
				.datain_h (datain_h),
				.datain_l (datain_l),
				.outclock (outclock),
				.dataout (sub_wire0),
				.aclr (1'b0),
				.aset (1'b0),
				.oe (1'b1),
				.oe_out (),
				.outclocken (1'b1),
				.sclr (1'b0),
				.sset (1'b0));
	defparam
		ALTDDIO_OUT_component.extend_oe_disable = "OFF",
		ALTDDIO_OUT_component.intended_device_family = "Cyclone IV",
		ALTDDIO_OUT_component.invert_output = "OFF",
		ALTDDIO_OUT_component.lpm_hint = "UNUSED",
		ALTDDIO_OUT_component.lpm_type = "altddio_out",
		ALTDDIO_OUT_component.oe_reg = "UNREGISTERED",
		ALTDDIO_OUT_component.power_up_high = "OFF",
		ALTDDIO_OUT_component.width = 1;


endmodule

// ============================================================
// CNX file retrieval info
// ============================================================
// Retrieval info: LIBRARY: altera_mf altera_mf.altera_mf_components.all
// Retrieval info: PRIVATE: INTENDED_DEVICE_FAMILY STRING "Cyclone III"
// Retrieval info: CONSTANT: EXTEND_OE_DISABLE STRING "OFF"
// Retrieval info: CONSTANT: INTENDED_DEVICE_FAMILY STRING "Cyclone III"
// Retrieval info: CONSTANT: INVERT_OUTPUT STRING "OFF"
// Retrieval info: CONSTANT: LPM_HINT STRING "UNUSED"
// Retrieval info: CONSTANT: LPM_TYPE STRING "altddio_out"
// Retrieval info: CONSTANT: OE_REG STRING "UNREGISTERED"
// Retrieval info: CONSTANT: POWER_UP_HIGH STRING "OFF"
// Retrieval info: CONSTANT: WIDTH NUMERIC "1"
// Retrieval info: USED_PORT: datain_h 0 0 1 0 INPUT NODEFVAL "datain_h[0..0]"
// Retrieval info: CONNECT: @datain_h 0 0 1 0 datain_h 0 0 1 0
// Retrieval info: USED_PORT: datain_l 0 0 1 0 INPUT NODEFVAL "datain_l[0..0]"
// Retrieval info: CONNECT: @datain_l 0 0 1 0 datain_l 0 0 1 0
// Retrieval info: USED_PORT: dataout 0 0 1 0 OUTPUT NODEFVAL "dataout[0..0]"
// Retrieval info: CONNECT: dataout 0 0 1 0 @dataout 0 0 1 0
// Retrieval info: USED_PORT: outclock 0 0 0 0 INPUT_CLK_EXT NODEFVAL "outclock"
// Retrieval info: CONNECT: @outclock 0 0 0 0 outclock 0 0 0 0
// Retrieval info: GEN_FILE: TYPE_NORMAL ddr.v TRUE FALSE
// Retrieval info: GEN_FILE: TYPE_NORMAL ddr.qip TRUE FALSE
// Retrieval info: GEN_FILE: TYPE_NORMAL ddr.bsf TRUE TRUE
// Retrieval info: GEN_FILE: TYPE_NORMAL ddr_inst.v TRUE TRUE
// Retrieval info: GEN_FILE: TYPE_NORMAL ddr_bb.v TRUE TRUE
// Retrieval info: GEN_FILE: TYPE_NORMAL ddr.inc TRUE TRUE
// Retrieval info: GEN_FILE: TYPE_NORMAL ddr.cmp TRUE TRUE
// Retrieval info: GEN_FILE: TYPE_NORMAL ddr.ppf TRUE FALSE
// Retrieval info: LIB_FILE: altera_mf
