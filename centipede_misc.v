//
// Atari Centipede, adapted from the original schematics
//
// Original implementation by Brad Parker <brad@heeltoe.com> 10/2015
// Synchronous rewrite by Jens Roever 3/2019
//

module centipede_misc (

    input           clk_cpu_4x, // 6 MHz    (real clock)
    input           clk_cpu_2x, // 3 MHz    (clock enable, 50% duty cycle)
    input           clk_cpu,    // 1.5 MHz  (clock enable, 25% duty cycle)

    input           reset_cpu,
					   
    input  [9:0]    playerinput_i,
    input  [7:0]    trakball_i,
    input  [7:0]    joystick_i,
    input  [7:0]    sw1_i,
    input  [7:0]    sw2_i,

    input           pf_irq_set,
    output reg      cpu_irq,

    input           cpu_read,
    input  [15:0]   cpu_addr,
    input  [7:0]    cpu_wr_data,

    output reg [7:0]    misc_rd_data,

    output [4:1]    led_o
);


	assign reset = reset_cpu;

    // =============================
	// cpu address decodes
    // =============================

	wire [7:0] 		hs_out, playerin_out, joystick_out, switch_out;

	assign sel_steerclr = (cpu_addr[13:10]==9) && !cpu_read;
	assign sel_watchdog = (cpu_addr[13:10]==8) && !cpu_read;
	assign sel_out0     = (cpu_addr[13:10]==7) && !cpu_read;
	assign sel_irqres   = (cpu_addr[13:10]==6) && !cpu_read;
	assign sel_in0      = (cpu_addr[13:10]==3) && !cpu_addr[1];
	assign sel_in1      = (cpu_addr[13:10]==3) &&  cpu_addr[1];
	assign sel_swrd     = (cpu_addr[13:10]==2);

    assign sel_hs_addr  = (cpu_addr[13:10]==5) && (cpu_addr[9:7]==3'b100);
    assign sel_hs_ctrl  = (cpu_addr[13:10]==5) && (cpu_addr[9:7]==3'b101);
    assign sel_hs_read  = (cpu_addr[13:10]==5) && (cpu_addr[9:7]==3'b110);


	always @(posedge clk_cpu_4x) begin
		misc_rd_data <= sel_hs_read ? hs_out 
						: sel_in0     ? playerin_out
						: sel_in1     ? joystick_out 
						: sel_swrd    ? switch_out
						:               8'h00;
	end


    // =============================
    // Watchdog Timer
    // =============================

	// not implemented


    // =============================
    // Interrupt Control
    // =============================

    // playfield timer will send irq_set pulse every 64 lines (posedge s_16v && s_32v)

    always @(posedge clk_cpu_4x or posedge reset) begin
        if (reset) begin
			cpu_irq <= 1'b0;
		end
        else if (clk_cpu && !cpu_read && sel_irqres) begin
			cpu_irq <= 1'b0;
		end
		else if (pf_irq_set) begin
			cpu_irq <= 1'b1;
		end
	end


    // =============================
    // High Score Memory Circuitry
    // =============================

    reg [5:0]       hs_addr;
    reg [7:0]       hs_data;
    reg [3:0]       hs_ctrl;

    always @(posedge clk_cpu_4x) begin

        if (clk_cpu && !cpu_read) begin
            if (sel_hs_addr) begin
                hs_addr <= cpu_addr[5:0];
                hs_data <= cpu_wr_data[7:0];
            end
            if (sel_hs_ctrl) begin
                hs_ctrl <= cpu_wr_data[3:0];
            end
        end
    end

    hs_ram hs_ram(
        .clk    ( hs_ctrl[0]    ),
        .reset  ( reset_cpu     ),
        .a      ( hs_addr       ),
        .dout   ( hs_out        ),
        .din    ( hs_data       ),
        .c1     (~hs_ctrl[1]    ),
        .c2     ( hs_ctrl[2]    ),
        .cs1    ( hs_ctrl[3]    ));


	// ======================
    // Joystick Circuitry
	// ======================

	wire		dir2, trb;

    assign js1_right = joystick_i[7];
    assign js1_left  = joystick_i[6];
    assign js1_down  = joystick_i[5];
    assign js1_up    = joystick_i[4];
    assign js2_right = joystick_i[3];
    assign js2_left  = joystick_i[2];
    assign js2_down  = joystick_i[1];
    assign js2_up    = joystick_i[0];

    assign joystick_out = cpu_addr[0] ?
                          { js1_right, js1_left, js1_down, js1_up, js2_right, js2_left, js2_down, js2_up } :
                          { dir2, 3'b0, trb };


	// ======================
	// Option Input Circuitry
	// ======================

   assign switch_out = cpu_addr[0] ? sw2_i : sw1_i;


   // ======================
   // Player Input Circuitry
   // ======================

	wire		coin_ctr_r_drive, coin_ctr_c_drive, coin_ctr_l_drive;

	assign coin_r    = coin_ctr_r_drive ? coin_ctr_r_drive : playerinput_i[9];
	assign coin_c    = coin_ctr_c_drive ? coin_ctr_c_drive : playerinput_i[8];
	assign coin_l    = coin_ctr_l_drive ? coin_ctr_l_drive : playerinput_i[7];

	assign self_test = playerinput_i[6];
	assign cocktail  = playerinput_i[5];
	assign slam      = playerinput_i[4];
	assign start1    = playerinput_i[3];
	assign start2    = playerinput_i[2];
	assign fire2     = playerinput_i[1];
	assign fire1     = playerinput_i[0];


	wire [7:0] playerin_out0;
	wire [7:0] playerin_out1;

	wire 	   dir1, vblank, tra;

	assign playerin_out1 = { coin_r, coin_c, coin_l, slam, fire2, fire1, start1, start2 };
	assign playerin_out0 = { dir1, vblank, self_test, cocktail, tra };

	assign playerin_out = cpu_addr[0] ? playerin_out1 : playerin_out0;
   

   // ======================
   // Coin Counter Output
   // ======================

	reg [7:0]  cc_latch;

    always @(posedge clk_cpu_4x) begin
		if (clk_cpu && !cpu_read && sel_out0) begin
			cc_latch[ cpu_addr[2:0] ] <= cpu_wr_data[7];
		end
	end

	assign flip			    = cc_latch[7];
	assign led_o[4]		    = cc_latch[6];
	assign led_o[3]		    = cc_latch[5];
	assign led_o[2]		    = cc_latch[4];
	assign led_o[1]		    = cc_latch[3];
	assign coin_ctr_r_drive = cc_latch[2];
	assign coin_ctr_c_drive = cc_latch[1];
	assign coin_ctr_l_drive = cc_latch[1];

	
	// ======================
	// Mini-Trak Ball inputs
	// ======================

	wire [3:0] tb_mux;

	reg        tb_h_reg, tb_v_reg;
	reg [3:0]  tb_h_ctr, tb_v_ctr;

	reg 	   steerclr_strb;
	
	assign s_1_horiz_dir = trakball_i[7];
	assign s_2_horiz_dir = trakball_i[6];
	assign s_1_horiz_ck  = trakball_i[5];
	assign s_2_horiz_ck  = trakball_i[4];
	assign s_1_vert_dir  = trakball_i[3];
	assign s_2_vert_dir  = trakball_i[2];
	assign s_1_vert_ck   = trakball_i[1];
	assign s_2_vert_ck   = trakball_i[0];

	assign tb_mux = flip ?
					{ s_1_horiz_dir, s_1_horiz_ck, s_1_vert_dir, s_1_vert_ck } :
					{ s_2_horiz_dir, s_2_horiz_ck, s_2_vert_dir, s_2_vert_ck };

	assign tb_h_dir = tb_mux[3];
	assign tb_h_ck  = tb_mux[2];
	assign tb_v_dir = tb_mux[1];
	assign tb_v_ck  = tb_mux[0];


	// make the steel clear reset registerd so we don't trigger it by accident
    always @(posedge clk_cpu_4x) begin
		steerclr_strb <= sel_steerclr && !cpu_read;
	end


   // H-Axis Quadrature Encoder
   always @(posedge tb_h_ck or posedge reset_cpu)
     if (reset_cpu)
       tb_h_reg <= 0;
     else
       tb_h_reg <= tb_h_dir;

   assign tb_h_ctr_clr = reset_cpu | steerclr_strb;
   
   always @(posedge tb_h_ck or posedge tb_h_ctr_clr)
     if (tb_h_ctr_clr)
       tb_h_ctr <= 0;
     else
       if (tb_h_reg)
     tb_h_ctr <= tb_h_ctr + 4'd1;
       else
     tb_h_ctr <= tb_h_ctr - 4'd1;

   // V-Axis Quadrature Encoder
   always @(posedge tb_v_ck or posedge reset_cpu)
     if (reset_cpu)
       tb_v_reg <= 0;
     else
       tb_v_reg <= tb_v_dir;

   assign tb_v_ctr_clr = reset_cpu | steerclr_strb;
   
   always @(posedge tb_v_ck or posedge tb_v_ctr_clr)
     if (tb_v_ctr_clr)
       tb_v_ctr <= 0;
     else
       if (tb_v_reg)
     tb_v_ctr <= tb_v_ctr + 4'd1;
       else
     tb_v_ctr <= tb_v_ctr - 4'd1;

   assign tra  = tb_h_ctr;
   assign trb  = tb_v_ctr;
   assign dir1 = tb_h_reg;
   assign dir2 = tb_v_reg;
   



endmodule
