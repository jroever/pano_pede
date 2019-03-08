//
// Atari Centipede, adapted from the original schematics
//
// Original implementation by Brad Parker <brad@heeltoe.com> 10/2015
// Synchronous rewrite by Jens Roever 3/2019
//

module centipede_pf(
	input           clk_cpu_4x, // 6 MHz    (real clock)
	input           clk_cpu_2x, // 3 MHz	(clock enable, 50% duty cycle)
	input           clk_cpu,    // 1.5 MHz	(clock enable, 25% duty cycle)

    input           reset_cpu,

    input           cpu_read,
    input  [15:0]   cpu_addr,
    input  [7:0]    cpu_wr_data,

    output [7:0]    pf_rd_data,
    output          pf_irq_set,

    output [8:0]    cga_rgb,
    output          cga_hsync,
    output          cga_vsync
);

	assign clk_cga = clk_cpu_4x; // CGA pixel clock is equal to 4x CPU clock
    assign reset   = reset_cpu;

    // =========================
	// reference timing counter
	// =========================

	// Note: h_count lsb from original 12 MHz implementation dropped since we are using 6MHz counter with clock enables for equivalent sub-phase decodes

	wire 			s_1h, s_2h, s_4h, s_8h, s_16h, s_32h, s_64h, s_128h, s_256h;
	wire 			s_1v, s_2v, s_4v, s_8v, s_16v, s_32v, s_64v, s_128v;

	reg [11:1] 		h_count;
	reg [7:0] 		v_count;
	
    wire            vreset;

	always @(posedge clk_cga or posedge reset) begin
		if (reset) begin
			h_count <= 0;
			v_count <= 0;
		end
		else begin
			if (h_count == 11'h7ff) begin
				h_count <= 11'h680; // 0x680 - 0x7ff = 128 hblank + 256 active (rotated display! H=V)
				if (vreset) begin
				    v_count <= 0; 
				end
				else begin
  				    v_count <= v_count + 1'b1;
				end
			end
			else begin
				h_count <= h_count + 1'b1;
			end 
		end 
	end 


    // =========================
	// reference timing counter
	// =========================

	assign {s_256h, s_128h, s_64h, s_32h, s_16h, s_8h, s_4h, s_2h, s_1h} = h_count[9:1];
	assign {        s_128v, s_64v, s_32v, s_16v, s_8v, s_4v, s_2v, s_1v} = v_count[7:0];

	wire		vblank;
	wire [7:0]	vprom_addr;
	wire [3:0]	vprom_out;
	reg  [3:0]	vprom_reg;
	reg 		s_256hd, vblankd;

    // todo: replace vprom with simple count decoder
	// NOTE: orig. hw has 17 lines vblank from 239 to 255 & 0!
	// VROM is traversed from 0 - 7F, F0-FF, 80, 1-7F ... (due to vblank being address MSB!)
	assign vprom_addr = {vblank, s_128v, s_64v, s_32v, s_8v, s_4v, s_2v, s_1v};

	vprom vprom(
		.clk	( clg_cga		),
		.reset	( reset			),
        .a		( vprom_addr	),
        .d		( vprom_out		));


	// @(posedge s_256h_n) - looked in at same time s_1v toggles -> effective at next line
	always @(posedge clk_cga)
	  if (h_count == 11'h7ff) begin
          vprom_reg <= vprom_out;
	  end

	assign vsync  = vprom_reg[0];
	assign vreset = vprom_reg[2];
	assign vblank = vprom_reg[3];

	
	always @(posedge clk_cga or posedge reset) begin
	  if (reset) begin
		  //s_256h2d <= 1'b0;
		  s_256hd <= 1'b0;
		  vblankd <= 1'b0;
	  end
	  else if (h_count[3:1]==3'b111) begin
		  //s_256h2d <= s_256hd;
		  s_256hd <= s_256h;
		  vblankd <= vblank;
	  end
	end


    // =============================
    // Interrupt Control
    // =============================

	reg			hsync;

	always @(posedge clk_cga or posedge reset) begin
       if (reset) begin
		   hsync <= 1'b0;
	   end
       else begin
		   if (h_count[9:1]==9'h027) begin
			   hsync <= 1'b1;
		   end
		   else if (h_count[9:1]==10'h05f) begin
			   hsync <= 1'b0;
		   end
	   end
	end


    // =============================
    // Interrupt Control
    // =============================

	reg			irq_set;

	// irq_set pulse @(posedge s_16v && s_32v)

	always @(posedge clk_cga or posedge reset) begin
		if (reset) 
		  irq_set <= 1'b0; 
		else 
		  irq_set <= !reset && (h_count == 11'h7ff) && (v_count[5:0] == 47); 
	end

	assign pf_irq_set = irq_set;


	// ===========================
	// motion objects (vertical)
	// ===========================

	// the motion object circuitry (vertical) receives pf data and vertical inputs from the
	// sync generator circuitry to generate the vertical component of the motion object video. PFD8-
	// 15 from the playfield memory and 1v-128v from the sync generator are compared at F6 and H6.
	// The output is gated by A7 when a motion object is on one of the sixteen vertical lines and is
	// latched by E6 and AND gate B7.  A low on B7 pin 8 indicates the presence of a motion object on
	// one of the vertical lines during non-active video time.  The signal (MATCH) enables the multi-
	// plexers in the picture data circuitry.
	//
	// when 256h goes high, 1v,2v,4v and pic0 are selected. When 256h goes low,
	// the latched output of E6 is selected. The output if D7 is EXCLUSIVE OR gated at E7 and is
	// sent to the picture data selector circuitry as motion graphics address (MGA0-MGA3). The other
	// input to EXCLUSIVE OR gate E7 is PIC7 from the playfield code multiplexer circuitry. PIC7
	// when high causes the output of E7 to be complimented.  For example, if MGA0..3 are low,
	// pic7 causes MGA0..3 to go high.  This causes the motion object video to be inverted top
	// to bottom.

	// mga0..3 (motion graphics address) from the motion object circuitry,
	//  256h and 256h_n from the sync generator
	// pic0..5 represents the code for the object to be displayed
	// mga0..3 set on of 8 different combinations of the 8-line by
	//  8-bit blocks of picture video or the 16 line by 8 bit blocks of
	//  motion object video
	//
	// 256h when high selects the playfield picture color codes to be addressed.
	// 256h when low selects the motion object color codes to be addressed

	reg  [5:0]			match_sum_hold;
	reg [7:0] 			pic;

	wire [31:0]			pfd;
	wire [7:0]			match_line, match_sum;
	wire [3:0] 			match_mux, mga;
	
	assign match_line    = { s_128v, s_64v, s_32v, s_16v, s_8v, s_4v, s_2v, s_1v };
	assign match_sum     = match_line + pfd[15:8];
	assign match_sum_top = ~(match_sum[7] & match_sum[6] & match_sum[5] & match_sum[4]);

	// @(posedge s_4h_n)
	always @(posedge clk_cga)
      if (h_count[4:1] == 4'b1111) 
		match_sum_hold <= { match_sum_top, 1'b0, match_sum[3:0] };

	assign match_mux = s_256h ? { pic[0], s_4v, s_2v, s_1v } : match_sum_hold[3:0];
	assign match_n   = match_sum_hold[5] & ~s_256h;

	// for debug only
	assign match_true = (~match_n & ~s_256h) && pfd != 0;

	assign mga = { match_mux[3] ^ (pic[7] & ~s_256h),
				   match_mux[2] ^ pic[7],
				   match_mux[1] ^ pic[7],
				   match_mux[0] ^ pic[7] };
   

	// ===========================
	// motion objects (horizontal)
	// ===========================

	// the motion object circuitry (horizontal) receives playfield data and horizontal inputs from
	// the sync generator circuitry. pfd16..23 from the pf memory determine the horizontal
	// position of the motion object.  pfd24..29 from the pf memory determine the indirect
	// color of the motion object.   pfd16:23 are latched and loaded into the horizontal position
	// counter.

	reg  [29:16]		pfd_hold, pfd_hold2;
	reg  [1:0]			area, gry;

    wire [1:0] 			y;


	// @(posedge s_4h)
	always @(posedge clk_cga) begin
		if (h_count[4:1] == 4'b0111) begin
			pfd_hold [29:16] <= pfd     [29:16];
			pfd_hold2[29:16] <= pfd_hold[29:16];
		end
	end
	
	assign y[1] =
				 (area == 2'b00) ? (s_256hd ? 1'b0 : gry[1]) :
				 (area == 2'b01) ? (s_256hd ? 1'b0 : pfd_hold2[25]) : 
				 (area == 2'b10) ? (s_256hd ? 1'b0 : pfd_hold2[27]) :
				 (area == 2'b11) ? (s_256hd ? 1'b0 : pfd_hold2[29]) :
				 1'b0;

	assign y[0] =
				 (area == 2'b00) ? (s_256hd ? 1'b0 : gry[0]) :
				 (area == 2'b01) ? (s_256hd ? 1'b0 : pfd_hold2[24]) : 
				 (area == 2'b10) ? (s_256hd ? 1'b0 : pfd_hold2[26]) :
				 (area == 2'b11) ? (s_256hd ? 1'b0 : pfd_hold2[28]) :
				 1'b0;

	assign pload_n = ~(s_1h & s_2h & s_4h);

	assign line_ram_ctr_load = ~(pload_n | s_256h); // active if pload and s_256==0
	assign line_ram_ctr_clr  = ~(pload_n | ~(s_256h & ~s_256hd)); // active if pload and posedge s_256
   
	reg [7:0]	line_ram_ctr;

	always @(posedge clk_cga)
      if (reset)
		line_ram_ctr <= 0;
      else
		begin
			if (line_ram_ctr_clr)
			  line_ram_ctr <= 0;
			else
			  if (line_ram_ctr_load) 
				line_ram_ctr <= pfd_hold[23:16];
			  else
				line_ram_ctr <= line_ram_ctr + 8'b1;
		end
   
	reg [1:0] line_ram [0:255];

	always @(posedge clk_cga) line_ram[line_ram_ctr] <= y;
	always @(posedge clk_cga) gry <= line_ram[line_ram_ctr];

   
	// ===========================
	//  playfield multiplexer
	// ===========================

	// The playfield multiplexer receives playfield data from the pf memory
	// (PFD0-PFD31) and the output (pf0..7) is a code that determines what is 1) dis-
	// played on the monitor or 2) read or updated by the MPU.
	//
	// When 256H is low and 4H is high, AB4 and AB5 from the MPU address bus is the
	// select output from P6.   The output is applied to multiplexers k6, l6, m6 and n6
	// as select inputs.  When the MPU is accessing the playfield code multiplexer, the
	// playfield data is either being read or updated by the MPU.  When 256H is high and 4H
	// is low, the inputs frmo the sync generator (128H and 8V) are the selected outputs.
	// These signals then select which bits of the data PFD0-PFD31 are send out via K6, L6
	// M6, and N6 for the playfield codes that eventually are displayed on the monitor.

	wire [7:0]			pf;
	wire [7:0]			pf_rom0_addr;
	wire [7:0]			pf_rom0_out;
	wire [7:0]			pf_rom1_addr;
	wire [7:0]			pf_rom1_out;

	always @(posedge clk_cga) if (h_count[4:1] == 4'd15) pic <= pf[7:0]; // update every 16 pixels fixme: timing
   
	pf_rom1 pf_rom1(
        .clk	( clk_cga		),
        .reset	( reset			),
        .a		( pf_rom1_addr	),
        .d		( pf_rom1_out	));

	pf_rom0 pf_rom0(
	    .clk	( clk_cga		),
        .reset	( reset			),
        .a		( pf_rom0_addr	),
        .d		( pf_rom0_out	));


	// a guess, based on millipede schematics

	wire [7:0] 			pf_rom0_out_rev, pf_rom1_out_rev, pf_mux0, pf_mux1;
	reg  [7:0]			pf_shift1, pf_shift0;


	assign pf_romx_haddr = ~s_256h & pic[0];


`ifdef force_pf  // debug
    wire [7:0]			ca;
    assign ca[7:0] = {s_16h, s_8h, s_128v, s_64v, s_32v, s_16v, s_8v};

    assign pf_rom0_addr[7:0] = { ca[7:1], { ca[0], s_4v, s_2v, s_1v} };
    assign pf_rom1_addr[7:0] = pf_rom0_addr;
`else
    assign pf_rom1_addr[7:0] = { pf_romx_haddr, s_256h, pic[5:1], mga };
    assign pf_rom0_addr[7:0] = { pf_romx_haddr, s_256h, pic[5:1], mga };
`endif


	assign pf_rom0_out_rev[7:0] = { pf_rom0_out[0], pf_rom0_out[1], pf_rom0_out[2], pf_rom0_out[3],
                                    pf_rom0_out[4], pf_rom0_out[5], pf_rom0_out[6], pf_rom0_out[7] };
   
	assign pf_rom1_out_rev[7:0] = { pf_rom1_out[0], pf_rom1_out[1], pf_rom1_out[2], pf_rom1_out[3],
                                    pf_rom1_out[4], pf_rom1_out[5], pf_rom1_out[6], pf_rom1_out[7] };
   
	assign pf_mux0[7:0] = match_n ? 8'b0 : (pic[6] ? pf_rom0_out_rev : pf_rom0_out);
	assign pf_mux1[7:0] = match_n ? 8'b0 : (pic[6] ? pf_rom1_out_rev : pf_rom1_out);


	always @(posedge clk_cga) pf_shift1 <= (~pload_n) ? pf_mux1 : { pf_shift1[6:0], 1'b0 };
	always @(posedge clk_cga) pf_shift0 <= (~pload_n) ? pf_mux0 : { pf_shift0[6:0], 1'b0 };
	always @(posedge clk_cga) area      <= { pf_shift1[7], pf_shift0[7] };


	wire 				pf_addr_stamp;

   // we ignore the cpu, as pf ram is now dp and cpu has it's own port
   assign pf_sel = pf_addr_stamp ? 2'b00 : { s_8v, s_128h };
   
   assign pf =
          (pf_sel == 2'b00) ? pfd[7:0] :
          (pf_sel == 2'b01) ? pfd[15:8] :
          (pf_sel == 2'b10) ? pfd[23:16] :
          (pf_sel == 2'b11) ? pfd[31:24] :
          8'b0;


	// ===========================
	// playfield address selector
	// ===========================

	wire 				nxt_pf_n, nxt_write_n;

	// when s_4h_n is low the pf addr selector receives 8h, 16, 32h & 64h and
	//  16v, 32v, 64v and 128v from the sync generator. these signals enable the sync
	//  generator circuits to access the playfield memory
	//
	// when s_4h_n goes high the game mpu addresses the pf memory
	// during horizontal blanking pfa4..7 are held high enabling the motion object
	// circuitry to access the playfield memory for the motion objects to be displayed

	assign pf_addr_stamp = ~s_256h;

	// force pf address to "stamp area" during hblank
	assign pfa7654 = pf_addr_stamp ? 4'b1111 : { s_128v, s_64v, s_32v, s_16v };
	assign pfa3210 = { s_64h, s_32h, s_16h, s_8h };
	assign pfa     = { pfa7654, pfa3210 };

	assign pf_ce4_n = 4'b0;

	assign {pfwr3_n, pfwr2_n, pfwr1_n, pfwr0_n} =
		({nxt_pf_n, nxt_write_n, cpu_addr[5:4]} == 4'b0000) ? 4'b1110 :
		({nxt_pf_n, nxt_write_n, cpu_addr[5:4]} == 4'b0001) ? 4'b1101 :
		({nxt_pf_n, nxt_write_n, cpu_addr[5:4]} == 4'b0010) ? 4'b1011 :
		({nxt_pf_n, nxt_write_n, cpu_addr[5:4]} == 4'b0011) ? 4'b0111 :
        4'b1111;

	assign {pfcs3_n, pfcs2_n, pfcs1_n, pfcs0_n} =
		(cpu_addr[5:4] == 2'b00) ? 4'b1110 :
		(cpu_addr[5:4] == 2'b01) ? 4'b1101 :
		(cpu_addr[5:4] == 2'b10) ? 4'b1011 :
		(cpu_addr[5:4] == 2'b11) ? 4'b0111 :
		4'b1111;


    assign pf_cs = (cpu_addr[13:10] == 4'b0001);

	wire [7:0] 			pf_out;
	reg  [7:0] 			pf_out_data;

	dp_ram pf_ram(
				  .clkb	( clk_cpu				),
				  .addrb	( {cpu_addr[9:6], cpu_addr[3:0],cpu_addr[5:4]}),
				  .dinb	( cpu_wr_data[7:0]		),
				  .doutb	( pf_out				),
				  .enb	( ~nxt_pf_n				),
				  .web	( ~nxt_write_n			),

				  .clka	( clk_cga				),
				  .addra	( pfa					),
				  .dina	( 32'h0000				),
				  .douta	( pfd					),
				  .ena	( 1'b1					),
				  .wea	( 1'b0					)
				  );

	always @(posedge clk_cpu_4x) pf_out_data <= pf_cs ? pf_out : 8'h00;


	// =======================
    // Video output circuitry
	// =======================

    // The video output circuit receives motion object, playfield, address and data inputs 
    //  and produces a video output to be displayed on the game monitor.
    // when the alternate color bit is active, an alternate shade of blue or green is available   
   

    assign coloram_cs   = (cpu_addr[13:9] == 5'b01010);
    assign coloram_we   = (coloram_cs && !cpu_read);
	assign coloram_we_n = ~(coloram_we && clk_cpu_2x && !clk_cpu); 		// write in the middle between two cpu_clk cycles

	// remap color index

	assign gry0_or_1 = gry[1] | gry[0];
	assign rama =  gry0_or_1 ?
          { {gry0_or_1, 1'b1}, gry[1:0] } :
          { {gry0_or_1, 1'b1}, area[1:0] };
   

	wire [7:0] 			coloram_rgbi;

	color_ram color_ram(
	    .clk_a	( clk_cpu				),
        .reset	( reset					),

        .addr_a	( cpu_addr[3:0]			),
        .dout_a	( coloram_out			),
        .din_a	( cpu_wr_data[3:0]		),
        .we_n_a	( coloram_we_n	  	    ),

        .clk_b	( s_6mhz_n				),
        .addr_b	( rama					),
        .dout_b	( coloram_rgbi[7:0]		));


	reg [7:0] 			coloram_rd_data;
	reg [7:0] 			rgbi;

	always @(posedge clk_cpu_4x) coloram_rd_data <= coloram_cs ? coloram_out : 8'h00;
	always @(posedge clk_cga)    rgbi <= coloram_rgbi;


	assign pf_rd_data = coloram_rd_data | pf_out_data;

	// =======================
    // output to the top level
	// =======================

   // bbb_ggg_rrr
   
`define no_colormap
`ifdef no_colormap
	assign cga_rgb =
        gry == 2'b00 & area[1:0] == 2'b00 ? 9'b000_000_000 :
        gry == 2'b00 & area[1:0] == 2'b01 ? 9'b000_000_111 :
        gry == 2'b00 & area[1:0] == 2'b10 ? 9'b000_111_000 :
        gry == 2'b00 & area[1:0] == 2'b11 ? 9'b111_000_000 :
        gry == 2'b01 ? 9'b000_000_111 :
        gry == 2'b10 ? 9'b000_111_000 :
        gry == 2'b11 ? 9'b111_000_000 :
        0;
`else
	assign cga_rgb = 
	    rgbi == 4'b0000 ? 9'b000_000_000 :
	    rgbi == 4'b0001 ? 9'b000_000_100 :
	    rgbi == 4'b0010 ? 9'b000_100_000 :
	    rgbi == 4'b0011 ? 9'b000_100_100 :
	    rgbi == 4'b0100 ? 9'b100_000_000 :
	    rgbi == 4'b0101 ? 9'b100_000_100 :
	    rgbi == 4'b0110 ? 9'b100_100_000 :
	    rgbi == 4'b0111 ? 9'b100_100_100 :
	    rgbi == 4'b1000 ? 9'b000_000_000 :
	    rgbi == 4'b1001 ? 9'b000_000_111 :
	    rgbi == 4'b1010 ? 9'b000_111_000 :
	    rgbi == 4'b1011 ? 9'b000_111_111 :
	    rgbi == 4'b1100 ? 9'b111_000_000 :
	    rgbi == 4'b1101 ? 9'b111_000_111 :
	    rgbi == 4'b1110 ? 9'b111_111_000 :
	    rgbi == 4'b1111 ? 9'b111_111_111 :
	    0;
`endif // !`ifdef no_colormap


	assign cga_vsync = vsync;
	assign cga_hsync = hsync;

endmodule
