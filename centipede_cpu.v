//
// Atari Centipede, adapted from the original schematics
//
// Original implementation by Brad Parker <brad@heeltoe.com> 10/2015
// Synchronous rewrite by Jens Roever 3/2019
//

module centipede_cpu(
	input           clk_cpu_4x, // 6 MHz    (real clock)
	input           clk_cpu_2x, // 3 MHz	(clock enable, 50% duty cycle)
	input           clk_cpu,    // 1.5 MHz	(clock enable, 25% duty cycle)

	input           reset_cpu,
	input           cpu_irq,

	output          cpu_read,
	output [15:0]   cpu_addr,
	output [7:0]    cpu_wr_data,

	input  [7:0]    ext_rd_data
);

	// ================
	// CPU CORE
	// ================

	wire [7:0] 		cpu_rd_data;


	bc6502 u_cpu(
		.reset		( reset_cpu 		),
		.clk		( clk_cpu_4x  		),
		.nmi		( 1'b0				),
		.irq		( cpu_irq  			),
		.so			( 1'b0				),
		.rdy		( clk_cpu			),
		.rw	  	    ( cpu_read			),
		.ma			( cpu_addr[15:0]	),
		.di			( cpu_rd_data[7:0]	),
		.dout		( cpu_wr_data[7:0]	)
	);

    reg  [7:0]  rom_rd_data;
    reg  [7:0]  ram_rd_data;

	assign cpu_rd_data = ram_rd_data
					   | rom_rd_data
					   | ext_rd_data;


	// ================
	// CPU ROM
	// ================

	wire        rom_cs_n;
    wire [7:0]  rom_out;

	assign rom_cs_n = ~(!cpu_read && cpu_addr[13]);

	rom u_rom
	  (
	   .clk	  	    ( clk_cpu_4x		),
	   .reset		( reset_cpu			),
	   .a			( cpu_addr[12:0]	),
	   .dout		( rom_out[7:0]		),
	   .cs_n		( rom_cs_n			)
	   );

	// registered rom read data at 4x clock will be sampled at next validated clk_cpu cycle
	always @(posedge clk_cpu_4x) rom_rd_data <= rom_cs_n ? 8'h00 : rom_out;


    // ================
    // CPU RAM
    // ================

	wire        ram_cs_n;
    wire [7:0]  ram_out;

    assign ram_cs_n = ~(cpu_addr[13:10] == 4'b0000);
	assign ram_we_n = clk_cpu_2x && !clk_cpu; 		// write in the middle between two cpu_clk cycles
	
    ram u_ram
	  (
	   .clk			( clk_cpu_4x		),
	   .reset		( reset_cpu			),
	   .a			( cpu_addr[9:0]		),
	   .din		    ( cpu_wr_data[7:0]	),
	   .dout		( ram_out[7:0]		),
	   .cs_n		( ram_cs_n			),
	   .we_n		( ram_we_n 			)
	   );

	// registered ram read data at 4x clock will be sampled at next validated clk_cpu cycle
	always @(posedge clk_cpu_4x) ram_rd_data <= ram_cs_n ? 8'h00 : ram_out;


endmodule
