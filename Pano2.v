module Pano2 (
      input   osc_clk,

      input   pano_button,

      output  vo_clk,
      output  vo_vsync,
      output  vo_hsync,
      output  vo_blank_,

      output [7:0] vo_r,
      output [7:0] vo_g,
      output [7:0] vo_b,

      input   led_green,
      input   led_blue,
      output  led_red,

      output  audio_mclk,
      output  audio_bclk,
      output  audio_dacdat,
      output  audio_daclrc,
      input   audio_adcdat,
      output  audio_adclrc,

      inout  vo_scl,
      inout  vo_sda,

      inout  audio_scl,
      inout  audio_sda);

    // =============================
    // Clock PLLs
    // =============================

	Pacman_clk pacman_clk_1_ 
	  ( 
		.CLKIN_IN           ( osc_clk       ),
		.CLKFX_OUT          ( clk_160m      ),
		.CLKIN_IBUFG_OUT    (               ),
		.CLK0_OUT           (               ) 
		);

	assign vo_clk = clk_160m;

	Pacman_clk1 pacman_clk1_1_ 
	  ( 
		.CLKIN_IN           ( clk_160m      ),
		.CLKDV_OUT          ( clk_24m       ),
		.CLK0_OUT           (               ) 
		);


    // =============================
    // Clock Divs and Reset
    // =============================

    (* buffer_type = "bufg" *) reg       core_clk12;
    reg 	 core_clk6;
    reg 	 core_clk3;
    (* buffer_type = "bufg" *) reg       core_clk1p5;
    reg 	 core_reset_;

	reg [4:0] core_rst_cntr;
	reg [3:0] core_clk_cntr;

	(* keep = "true" *) reg       core_reset_ub_;     

	always @ (posedge clk_24m) begin
		if (core_rst_cntr!= 5'b11111) begin
			core_rst_cntr <= (core_rst_cntr + (5'b0001));
			core_reset_ub_ <= 1'b0;
            core_clk_cntr <= 4'b0000;
		end
		else begin
            core_reset_ub_ <= 1'b1;
            core_clk_cntr <= core_clk_cntr + 4'b0001;
		end
	end


	always @ (posedge clk_24m) begin
		core_clk12  <= core_clk_cntr[0];
		core_clk6   <= (core_clk_cntr[0] && (! core_clk_cntr[1]));
		core_clk3   <= ((core_clk_cntr[0] && (! core_clk_cntr[1])) && (! core_clk_cntr[2]));
		core_clk1p5 <= (((core_clk_cntr[0] && (! core_clk_cntr[1])) && (! core_clk_cntr[2])) && (! core_clk_cntr[3]));
		core_reset_ <= core_reset_ub_;
	end


	wire [7:0]	audio_o;
	wire [8:0] 	rgb_o;


    // =============================
	// Centipede CPU
    // =============================

	wire [15:0] cpu_addr;
	wire [7:0] 	ext_rd_data;
	wire [7:0] 	misc_rd_data;
	wire [7:0] 	audio_rd_data;
	wire [7:0] 	pf_rd_data;
	wire [7:0] 	cpu_wr_data;


	(* S = "true" *) centipede_cpu u_centipede_cpu 
		   ( 
			 .clk_cpu            ( core_clk1p5   ),
			 .clk_cpu_2x         ( core_clk3     ),
			 .clk_cpu_4x         ( core_clk6     ),
			 .reset_cpu          ( ~core_reset_  ),

			 .cpu_irq            ( cpu_irq       ),

			 .cpu_read           ( cpu_read      ),
			 .cpu_addr           ( cpu_addr      ),
			 .cpu_wr_data        ( cpu_wr_data   ),

			 .ext_rd_data        ( ext_rd_data   )
			 );

	assign ext_rd_data = misc_rd_data
						 | audio_rd_data
 	 					 | pf_rd_data;


    // =============================
	// Centipede Miscellaneous
    // =============================

	(* S = "true" *) centipede_misc u_centipede_misc 
		   ( 
			 .clk_cpu            ( core_clk1p5   ),
			 .clk_cpu_2x         ( core_clk3     ),
			 .clk_cpu_4x         ( core_clk6     ),
			 .reset_cpu          ( ~core_reset_  ),

			 .pf_irq_set         ( pf_irq_set    ),
			 .cpu_irq            ( cpu_irq       ),

			 .cpu_read           ( cpu_read      ),
			 .cpu_addr           ( cpu_addr      ),
			 .cpu_wr_data        ( cpu_wr_data   ),

			 .misc_rd_data       ( misc_rd_data  ),

			 .playerinput_i      ( 10'b1111011111),
			 .trakball_i         ( 8'h00         ),
			 .joystick_i         ( 8'h00         ),
			 .sw1_i              ( 8'h54         ),
			 .sw2_i              ( 8'h00         ),
			 .led_o              (               )
			 );

    // =============================
	// Centipede Audio
    // =============================

	(* S = "true" *) centipede_audio u_centipede_audio 
		   ( 
			 .clk_cpu            ( core_clk1p5   ),
			 .clk_cpu_2x         ( core_clk3     ),
			 .clk_cpu_4x         ( core_clk6     ),
			 .reset_cpu          ( ~core_reset_  ),

			 .cpu_read           ( cpu_read      ),
			 .cpu_addr           ( cpu_addr      ),
			 .cpu_wr_data        ( cpu_wr_data   ),

			 .audio_rd_data      ( audio_rd_data ),

			 .audio_o            ( audio_o[7:0]  ) 
			 );


	assign audio_sample = {8'h00,audio_o};



    // =============================
	// Centipede Play Field
    // =============================

	(* S = "true" *) centipede_pf u_centipede_pf 
		   ( 
			 .clk_cpu            ( core_clk1p5   ),
			 .clk_cpu_2x         ( core_clk3     ),
			 .clk_cpu_4x         ( core_clk6     ),
			 .reset_cpu          ( ~core_reset_  ),

			 .cpu_read           ( cpu_read      ),
			 .cpu_addr           ( cpu_addr      ),
			 .cpu_wr_data        ( cpu_wr_data   ),

			 .pf_rd_data         ( pf_rd_data    ),
			 .pf_irq_set		 ( pf_irq_set    ),

			 .cga_rgb            ( rgb_o[8:0]    ),
			 .cga_hsync          ( hsync_o       ),
			 .cga_vsync          ( vsync_o       )
			 );


	assign vo_r      = ({5'd0,rgb_o[8 : 6]} <<< 5);
	assign vo_g      = ({6'd0,rgb_o[5 : 3]} <<< 5);
	assign vo_b      = ({5'd0,rgb_o[2 : 0]} <<< 5);

	assign vo_vsync  = !vsync_o;
	assign vo_hsync  = !hsync_o;
	assign vo_blank_ = 1'b1;


endmodule

