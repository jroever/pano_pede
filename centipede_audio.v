//
// Atari Centipede, adapted from the original schematics
//
// Original implementation by Brad Parker <brad@heeltoe.com> 10/2015
// Synchronous rewrite by Jens Roever 3/2019
//

module centipede_audio(
	input           clk_cpu_4x, // 6 MHz    (real clock)
	input           clk_cpu_2x, // 3 MHz	(clock enable, 50% duty cycle)
	input           clk_cpu,    // 1.5 MHz	(clock enable, 25% duty cycle)

    input           reset_cpu,

    input           cpu_read,
    input  [15:0]   cpu_addr,
    input  [7:0]    cpu_wr_data,

    output [7:0]    audio_rd_data,

    output [7:0]    audio_o
);


	// =========================
	// Audio output circuitry
	// =========================

    // pokey has combinatorial read mux

    assign sel_pokey = (cpu_addr[13:10]==4'b0100);

	pokey pokey(
		.phi2	( clk_cpu				),
		.reset	( cpu_reset				),

	    .a		( cpu_addr[3:0]			),
		.cs0_n	( 1'b0					),
		.cs1_n	( ~sel_pokey			),
		.r_w_n	( cpu_read				),
		.d_in	( cpu_wr_data[7:0]		),
		.d_out	( pokey_out				),
		.p		( 8'b0					),

		.aud	( audio_o				));
   
	assign audio_rd_data = sel_pokey ? pokey_out : 8'h00; 

endmodule
